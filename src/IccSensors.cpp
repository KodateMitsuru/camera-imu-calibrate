#include <IccSensors.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/cameras/CameraGeometry.hpp>
#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>
#include <aslam/cameras/GridCalibrationTargetBase.hpp>
#include <aslam/cameras/GridCalibrationTargetCheckerboard.hpp>
#include <aslam/cameras/GridCalibrationTargetCirclegrid.hpp>
#include <cmath>
#include <iostream>
#include <kalibr_backend/MatrixDesignVariable.hpp>
#include <kalibr_backend/ScalarDesignVariable.hpp>
#include <kalibr_backend/TransformationDesignVariable.hpp>
#include <kalibr_common/TargetExtractor.hpp>
#include <kalibr_errorterms/AccelerometerError.hpp>
#include <kalibr_errorterms/GyroscopeError.hpp>
#include <memory>
#include <opencv2/highgui.hpp>
#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/transformations.hpp>

#include "kalibr_common/ConfigReader.hpp"

namespace kalibr {

constexpr int CALIBRATION_GROUP_ID = 0;
constexpr int HELPER_GROUP_ID = 1;

// ============================================================================
// IccCamera Implementation
// ============================================================================

IccCamera::IccCamera(const CameraParameters& camConfig,
                     const CalibrationTargetParameters& targetConfig,
                     const ImageDatasetReader& dataset,
                     double reprojectionSigma, bool showCorners,
                     bool showReproj, bool showOneStep)
    : dataset_(dataset),
      reprojectionSigma_(reprojectionSigma),
      showCorners_(showCorners),
      showReproj_(showReproj),
      showOneStep_(showOneStep) {
  // Corner uncertainty
  cornerUncertainty_ = reprojectionSigma;

  // Set the extrinsic prior to default (identity)
  T_extrinsic_ = sm::kinematics::Transformation();

  // Initialize timeshift prior to zero
  timeshiftCamToImuPrior_ = 0.0;

  // Initialize the camera from parameters
  camera_ = AslamCamera::fromParameters(camConfig);

  // Extract corners
  setupCalibrationTarget(targetConfig, showCorners, showReproj, showOneStep);

  bool multithreading = !(showCorners || showReproj || showOneStep);
  observations_ = extractCornersFromDataset(dataset_, *detector_,
                                            multithreading,  // multithreading
                                            0,     // auto-detect num processes
                                            true,  // clear images
                                            false  // no transformation
  );

  // An estimate of the gravity in the world coordinate frame
  gravity_w_ = Eigen::Vector3d(9.80655, 0.0, 0.0);

  std::cout << "Camera initialized with " << observations_.size()
            << " target observations" << std::endl;
}

void IccCamera::setupCalibrationTarget(
    const CalibrationTargetParameters& targetConfig, bool showExtraction,
    bool showReproj, bool imageStepping) {
  std::cout << "Setting up calibration target..." << std::endl;

  // Load the calibration target configuration

  auto targetType = targetConfig.getTargetType();

  // Create the calibration target grid based on type
  std::shared_ptr<aslam::cameras::GridCalibrationTargetBase> grid;

  if (targetType == TargetType::Checkerboard) {
    auto targetParams = targetConfig.getCheckerboardParams();
    aslam::cameras::GridCalibrationTargetCheckerboard::CheckerboardOptions
        options;
    options.filterQuads = true;
    options.normalizeImage = true;
    options.useAdaptiveThreshold = true;
    options.performFastCheck = false;
    options.windowWidth = 5;
    options.showExtractionVideo = showExtraction;

    grid = std::make_shared<aslam::cameras::GridCalibrationTargetCheckerboard>(
        targetParams.rows, targetParams.cols, targetParams.rowSpacing,
        targetParams.colSpacing, options);

  } else if (targetType == TargetType::Circlegrid) {
    auto targetParams = targetConfig.getCirclegridParams();
    aslam::cameras::GridCalibrationTargetCirclegrid::CirclegridOptions options;
    options.showExtractionVideo = showExtraction;
    options.useAsymmetricCirclegrid = targetParams.asymmetric;

    grid = std::make_shared<aslam::cameras::GridCalibrationTargetCirclegrid>(
        targetParams.rows, targetParams.cols, targetParams.spacing, options);

  } else if (targetType == TargetType::Aprilgrid) {
    auto targetParams = targetConfig.getAprilgridParams();
    aslam::cameras::GridCalibrationTargetAprilgrid::AprilgridOptions options;
    options.showExtractionVideo = showExtraction;
    options.minTagsForValidObs =
        std::max(targetParams.tagRows, targetParams.tagCols) + 1;

    grid = std::make_shared<aslam::cameras::GridCalibrationTargetAprilgrid>(
        targetParams.tagRows, targetParams.tagCols, targetParams.tagSize,
        targetParams.tagSpacing, options);

  } else {
    throw std::runtime_error("Unknown calibration target type");
  }

  // Create grid detector
  aslam::cameras::GridDetector::GridDetectorOptions detectorOptions;
  detectorOptions.imageStepping = imageStepping;
  detectorOptions.plotCornerReprojection = showReproj;
  detectorOptions.filterCornerOutliers = true;

  detector_ = std::make_shared<aslam::cameras::GridDetector>(camera_, grid,
                                                             detectorOptions);

  std::cout << "Calibration target setup completed" << std::endl;
}

void IccCamera::findOrientationPriorCameraToImu(const IccImu& imu) {
  std::cout << "\nEstimating imu-camera rotation prior" << std::endl;

  // Build the optimization problem
  std::shared_ptr<aslam::backend::OptimizationProblem> problem =
      std::make_shared<aslam::backend::OptimizationProblem>();

  // Add the rotation as design variable
  auto q_i_c_Dv =
      std::make_shared<aslam::backend::RotationQuaternion>(T_extrinsic_.q());
  q_i_c_Dv->setActive(true);
  problem->addDesignVariable(q_i_c_Dv);

  // Add the gyro bias as design variable
  auto gyroBiasDv =
      std::make_shared<aslam::backend::EuclideanPoint>(Eigen::Vector3d::Zero());
  gyroBiasDv->setActive(true);
  problem->addDesignVariable(gyroBiasDv);

  // Initialize a pose spline using the camera poses
  bsplines::BSplinePose poseSpline = initPoseSplineFromCamera(
      6, 100, 0.0);  // splineOrder, knotsPerSecond, no padding

  // Add error terms for each IMU measurement
  const auto& imuData = imu.getImuData();
  aslam::backend::EuclideanExpression bias(Eigen::Vector3d::Zero());
  for (const auto& im : imuData) {
    double tk = im.timestamp.toSec();

    if (tk > poseSpline.t_min() && tk < poseSpline.t_max()) {
      // DV expressions
      auto R_i_c = q_i_c_Dv->toExpression();
      auto bias = gyroBiasDv->toExpression();

      // Get the vision predicted omega and measured omega (IMU)
      Eigen::Vector3d omega_spline = poseSpline.angularVelocityBodyFrame(tk);
      auto omega_predicted =
          R_i_c * aslam::backend::EuclideanExpression(omega_spline);
      Eigen::Vector3d omega_measured = im.omega;

      // Create gyroscope error term
      auto gerr = std::make_shared<kalibr_errorterms::GyroscopeError>(
          omega_measured, im.omega.inverse(), omega_predicted, bias);
      problem->addErrorTerm(gerr);
    }
  }

  if (problem->numErrorTerms() == 0) {
    throw std::runtime_error(
        "Failed to obtain orientation prior. "
        "Please make sure that your sensors are synchronized correctly.");
  }

  // Define the optimization options
  aslam::backend::Optimizer2Options options;
  options.verbose = false;
  options.nThreads = 2;
  options.convergenceDeltaX = 1e-4;
  options.convergenceDeltaJ = 1.0;
  options.maxIterations = 50;

  // Run the optimization
  aslam::backend::Optimizer2 optimizer(options);
  optimizer.setProblem(problem);

  try {
    optimizer.optimize();
  } catch (const std::exception& e) {
    throw std::runtime_error("Failed to obtain orientation prior: " +
                             std::string(e.what()));
  }

  // Overwrite the external rotation prior (keep translation)
  Eigen::Matrix3d R_i_c = q_i_c_Dv->toRotationMatrix().transpose();
  T_extrinsic_ = sm::kinematics::Transformation(
      sm::kinematics::rt2Transform(R_i_c, T_extrinsic_.t()));

  // Estimate gravity as the mean specific force
  std::vector<Eigen::Vector3d> a_w_samples;
  for (const auto& im : imuData) {
    double tk = im.timestamp.toSec();
    if (tk > poseSpline.t_min() && tk < poseSpline.t_max()) {
      Eigen::Matrix3d C_w_b = poseSpline.orientation(tk);
      Eigen::Vector3d a_w = C_w_b * (R_i_c * (-im.alpha));
      a_w_samples.push_back(a_w);
    }
  }

  // Compute mean
  Eigen::Vector3d mean_a_w = Eigen::Vector3d::Zero();
  for (const auto& a : a_w_samples) {
    mean_a_w += a;
  }
  mean_a_w /= a_w_samples.size();

  // Normalize to standard gravity
  gravity_w_ = mean_a_w.normalized() * 9.80655;

  std::cout << "  Gravity was initialized to: " << gravity_w_.transpose()
            << " [m/s^2]" << std::endl;

  // Get gyro bias
  Eigen::Vector3d b_gyro = bias.toEuclidean();

  // Update IMU gyro bias prior (using recursive average if multiple cameras)
  // Note: This modifies the IMU object - in practice you may need a different
  // approach

  std::cout << "  Orientation prior camera-imu found as: (T_i_c)" << std::endl;
  std::cout << R_i_c << std::endl;
  std::cout << "  Gyro bias prior found as: (b_gyro)" << std::endl;
  std::cout << b_gyro.transpose() << std::endl;
}

Eigen::Vector3d IccCamera::getEstimatedGravity() const { return gravity_w_; }

double IccCamera::findTimeshiftCameraImuPrior(const IccImu& imu, bool verbose) {
  std::cout << "Estimating time shift camera to imu:" << std::endl;

  // Fit a spline to the camera observations
  bsplines::BSplinePose poseSpline =
      initPoseSplineFromCamera(6, 100, 0.0);  // no padding

  // Collect angular velocity norms
  std::vector<double> t;
  std::vector<double> omega_measured_norm;
  std::vector<double> omega_predicted_norm;

  const auto& imuData = imu.getImuData();
  for (const auto& im : imuData) {
    double tk = im.timestamp.toSec();

    if (tk > poseSpline.t_min() && tk < poseSpline.t_max()) {
      // Get IMU measurements
      Eigen::Vector3d omega_meas = im.omega;

      // Get spline predicted angular velocity
      Eigen::Vector3d omega_pred = poseSpline.angularVelocityBodyFrame(tk);

      // Store timestamp and norms
      t.push_back(tk);
      omega_measured_norm.push_back(omega_meas.norm());
      omega_predicted_norm.push_back(omega_pred.norm());
    }
  }

  if (omega_predicted_norm.empty() || omega_measured_norm.empty()) {
    throw std::runtime_error(
        "The time ranges of the camera and IMU do not overlap. "
        "Please make sure that your sensors are synchronized correctly.");
  }

  // Compute cross-correlation
  // Note: This is a simplified version - you may want to use a proper
  // cross-correlation library like FFT-based correlation
  int n_pred = omega_predicted_norm.size();
  int n_meas = omega_measured_norm.size();
  int max_lag = n_meas - 1;

  std::vector<double> corr(2 * max_lag + 1);
  double max_corr = -std::numeric_limits<double>::infinity();
  int best_lag = 0;

  for (int lag = -max_lag; lag <= max_lag; ++lag) {
    double sum = 0.0;
    int count = 0;

    for (int i = 0; i < n_pred; ++i) {
      int j = i + lag;
      if (j >= 0 && j < n_meas) {
        sum += omega_predicted_norm[i] * omega_measured_norm[j];
        count++;
      }
    }

    double corr_val = (count > 0) ? sum / count : 0.0;
    corr[lag + max_lag] = corr_val;

    if (corr_val > max_corr) {
      max_corr = corr_val;
      best_lag = lag;
    }
  }

  int discrete_shift = best_lag;

  // Get continuous time shift
  double dT = 0.0;
  if (imuData.size() > 1) {
    std::vector<double> times;
    for (const auto& im : imuData) {
      times.push_back(im.timestamp.toSec());
    }

    // Compute mean dt
    double sum_dt = 0.0;
    for (size_t i = 1; i < times.size(); ++i) {
      sum_dt += times[i] - times[i - 1];
    }
    dT = sum_dt / (times.size() - 1);
  }

  double shift = -discrete_shift * dT;

  if (verbose) {
    std::cout << "  Discrete time shift: " << discrete_shift << std::endl;
    std::cout << "  Continuous time shift: " << shift << std::endl;
    std::cout << "  dT: " << dT << std::endl;

    // TODO: Add plotting if needed
  }

  // Store the timeshift (t_imu = t_cam + timeshiftCamToImuPrior)
  timeshiftCamToImuPrior_ = shift;

  std::cout << "  Time shift camera to imu (t_imu = t_cam + shift):"
            << std::endl;
  std::cout << "  " << timeshiftCamToImuPrior_ << " seconds" << std::endl;

  return timeshiftCamToImuPrior_;
}

bsplines::BSplinePose IccCamera::initPoseSplineFromCamera(
    int splineOrder, int poseKnotsPerSecond, double timeOffsetPadding) {
  std::cout << "Initializing pose spline from camera observations..."
            << std::endl;

  // Get the extrinsic transformation
  Eigen::Matrix4d T_c_b = T_extrinsic_.T();

  // Create pose spline
  bsplines::BSplinePose pose(
      splineOrder, std::make_shared<sm::kinematics::RotationVector>());

  // Get times and poses from observations
  int numObs = observations_.size();
  if (numObs == 0) {
    throw std::runtime_error(
        "No observations available for spline initialization");
  }

  Eigen::VectorXd times(numObs);
  Eigen::Matrix<double, 6, Eigen::Dynamic> curve(6, numObs);

  // Extract times and transform observations to curve values
  for (int i = 0; i < numObs; ++i) {
    const auto& obs = observations_[i];
    times(i) = obs.time().toSec() + timeshiftCamToImuPrior_;

    // T_t_c: target to camera transformation
    // T_c_b: camera to body transformation
    // T_w_b = T_t_c^T * T_c_b (world/target frame to body frame)
    Eigen::Matrix4d T_w_b = obs.T_t_c().T() * T_c_b;
    curve.col(i) = pose.transformationToCurveValue(T_w_b);
  }

  // Check for NaN values
  if (curve.hasNaN()) {
    throw std::runtime_error("NaN values in curve");
  }

  // Add padding on both ends
  Eigen::VectorXd times_padded(numObs + 2);
  Eigen::Matrix<double, 6, Eigen::Dynamic> curve_padded(6, numObs + 2);

  times_padded(0) = times(0) - (timeOffsetPadding * 2.0);
  times_padded.segment(1, numObs) = times;
  times_padded(numObs + 1) = times(numObs - 1) + (timeOffsetPadding * 2.0);

  curve_padded.col(0) = curve.col(0);
  curve_padded.block(0, 1, 6, numObs) = curve;
  curve_padded.col(numObs + 1) = curve.col(numObs - 1);

  // Fix rotation vector wrapping to prevent discontinuities
  // The rotation vector is in rows 3-5 (indices 3, 4, 5)
  for (int i = 1; i < curve_padded.cols(); ++i) {
    Eigen::Vector3d previousRotationVector = curve_padded.block<3, 1>(3, i - 1);
    Eigen::Vector3d r = curve_padded.block<3, 1>(3, i);

    double angle = r.norm();
    if (angle < 1e-10) {
      // Very small rotation, keep as is
      continue;
    }

    Eigen::Vector3d axis = r / angle;
    Eigen::Vector3d best_r = r;
    double best_dist = (best_r - previousRotationVector).norm();

    // Try wrapping the angle by multiples of 2π
    for (int s = -3; s <= 3; ++s) {
      if (s == 0) continue;  // Already checked

      Eigen::Vector3d aa = axis * (angle + M_PI * 2.0 * s);
      double dist = (aa - previousRotationVector).norm();

      if (dist < best_dist) {
        best_r = aa;
        best_dist = dist;
      }
    }

    curve_padded.block<3, 1>(3, i) = best_r;
  }

  // Calculate number of knots
  double seconds = times_padded(times_padded.size() - 1) - times_padded(0);
  int knots = static_cast<int>(std::round(seconds * poseKnotsPerSecond));

  std::cout << "Initializing a pose spline with " << knots << " knots ("
            << poseKnotsPerSecond << " knots per second over " << seconds
            << " seconds)" << std::endl;

  // Initialize sparse pose spline
  pose.initPoseSplineSparse(times_padded, curve_padded, knots, 1e-4);

  return pose;
}

void IccCamera::addDesignVariables(aslam::backend::OptimizationProblem& problem,
                                   bool noTimeCalibration,
                                   bool estimateExtrinsics) {
  // Add transformation design variables
  if (estimateExtrinsics) {
    dv_T_c_b_ = std::make_shared<aslam::backend::TransformationDesignVariable>(
        T_extrinsic_);
    dv_T_c_b_->setActive(true);
    problem.addDesignVariable(dv_T_c_b_);
    std::cout << "  Camera-IMU extrinsics enabled" << std::endl;
  } else {
    dv_T_c_b_ = std::make_shared<aslam::backend::TransformationDesignVariable>(
        T_extrinsic_);
    dv_T_c_b_->setActive(false);
    std::cout << "  Camera-IMU extrinsics fixed" << std::endl;
  }

  // Add time offset design variable
  if (!noTimeCalibration) {
    dv_time_offset_ = std::make_shared<aslam::backend::ScalarDesignVariable>(
        timeshiftCamToImuPrior_);
    dv_time_offset_->setActive(true);
    problem.addDesignVariable(dv_time_offset_);
    std::cout << "  Camera time offset enabled" << std::endl;
  } else {
    dv_time_offset_ = std::make_shared<aslam::backend::ScalarDesignVariable>(
        timeshiftCamToImuPrior_);
    dv_time_offset_->setActive(false);
    std::cout << "  Camera time offset fixed" << std::endl;
  }
}

void IccCamera::addCameraErrorTerms(
    aslam::backend::OptimizationProblem& problem,
    const std::shared_ptr<aslam::splines::BSplinePoseDesignVariable>& poseDv,
    bool useBlakeZissermanMest) {
  std::cout << "  Adding camera error terms ("
            << (useBlakeZissermanMest ? "M-estimator" : "squared") << ")..."
            << std::endl;

  int num_error_terms = 0;

  // Loop over all observations
  for (const auto& obs : observations_) {
    double t_cam = obs.time().toSec();

    // Get time with offset
    auto time_expression = dv_time_offset_->toExpression();
    double t_corrected = t_cam;  // + time offset will be handled in expression

    // Skip if outside spline range
    if (t_corrected < poseDv->spline().getMinTime() ||
        t_corrected > poseDv->spline().getMaxTime()) {
      continue;
    }

    // Get the transformation from target to body at this time
    // T_b_w = pose_spline(t)
    auto T_b_w_expr =
        poseDv->toExpression(t_corrected, 0);  // 0 = position + orientation

    // Get camera-to-body transformation expression
    auto T_c_b_expr = dv_T_c_b_->toExpression();

    // Compute T_c_w = T_c_b * T_b_w
    auto T_c_w_expr = T_c_b_expr * T_b_w_expr;

    // Get observed corners
    const auto& imagePoints = obs.imagePoints();
    const auto& targetPoints = obs.targetPoints();

    // Add reprojection error term for each corner
    for (size_t i = 0; i < imagePoints.size(); ++i) {
      Eigen::Vector2d y = imagePoints[i];  // measured image point
      Eigen::Vector4d p_target =
          targetPoints[i];  // 3D target point (homogeneous)

      // Create reprojection error
      // This projects p_target through T_c_w and compares to y
      auto err = std::make_shared<kalibr_errorterms::ReprojectionError>(
          y, cornerUncertainty_, T_c_w_expr, p_target.head<3>(), geometry_);

      // Add M-estimator if requested
      if (useBlakeZissermanMest) {
        auto mestimator =
            std::make_shared<aslam::backend::BlakeZissermanMEstimator>(
                2.0 * cornerUncertainty_);  // threshold
        err->setMEstimatorPolicy(mestimator);
      }

      problem.addErrorTerm(err);
      num_error_terms++;
    }
  }

  std::cout << "    Added " << num_error_terms << " camera error terms"
            << std::endl;
}

// ============================================================================
// IccCameraChain Implementation
// ============================================================================

IccCameraChain::IccCameraChain(
    const std::vector<CameraParameters>& camConfigs,
    const TargetParameters& targetConfig,
    const std::vector<ImageDatasetReader>& datasets) {
  if (camConfigs.size() != datasets.size()) {
    throw std::runtime_error(
        "Number of camera configs must match number of datasets");
  }

  // Create camera instances
  for (size_t i = 0; i < camConfigs.size(); ++i) {
    auto cam =
        std::make_shared<IccCamera>(camConfigs[i], targetConfig, datasets[i]);
    camList_.push_back(cam);
  }

  std::cout << "Initialized camera chain with " << camList_.size() << " cameras"
            << std::endl;
}

void IccCameraChain::addDesignVariables(
    aslam::backend::OptimizationProblem& problem, bool noTimeCalibration,
    bool noChainExtrinsics) {
  // Add transformations between cameras
  for (size_t i = 1; i < camList_.size(); ++i) {
    // Create transformation design variable for cam[i] relative to cam[i-1]
    auto T = std::make_shared<aslam::backend::TransformationDesignVariable>();
    T->setActive(!noChainExtrinsics);
    problem.addDesignVariable(T, CALIBRATION_GROUP_ID);
    transformations_.push_back(T);
  }

  // Add time offset design variables
  if (!noTimeCalibration) {
    for (size_t i = 0; i < camList_.size(); ++i) {
      auto timeOffset =
          std::make_shared<aslam::backend::ScalarDesignVariable>(0.0);
      timeOffset->setActive(true);
      problem.addDesignVariable(timeOffset, CALIBRATION_GROUP_ID);
      timeOffsets_.push_back(timeOffset);
    }
  }
}

void IccCameraChain::initFromCameraChain() {
  // Initialize camera chain parameters
  std::cout << "Initializing camera chain..." << std::endl;
}

// ============================================================================
// IccImu Implementation
// ============================================================================

IccImu::IccImu(const ImuParameters& imuConfig, const ImuDatasetReader& dataset)
    : dataset_(dataset), timeOffset_(0.0) {
  // Load IMU data
  imuData_ = dataset_.getAllMessages();

  std::cout << "Initialized IMU with " << imuData_.size() << " measurements"
            << std::endl;
}

void IccImu::addDesignVariables(aslam::backend::OptimizationProblem& problem,
                                const aslam::splines::BSplinePose& poseSpline) {
  // Add transformation from IMU to body frame
  T_i_b_Dv_ = std::make_shared<aslam::backend::TransformationDesignVariable>();
  T_i_b_Dv_->setActive(true);
  problem.addDesignVariable(T_i_b_Dv_, CALIBRATION_GROUP_ID);

  // Add time offset design variable
  timeOffsetDv_ =
      std::make_shared<aslam::backend::ScalarDesignVariable>(timeOffset_);
  timeOffsetDv_->setActive(true);
  problem.addDesignVariable(timeOffsetDv_, CALIBRATION_GROUP_ID);
}

void IccImu::addErrorTerms(
    aslam::backend::OptimizationProblem& problem,
    const aslam::backend::BSplinePoseDesignVariable& poseDv,
    const aslam::backend::EuclideanExpression& gravityExpression,
    double gyroNoiseScale, double accelNoiseScale, double huberGyro,
    double huberAccel) {
  std::cout << "Adding IMU error terms..." << std::endl;

  // Add gyroscope error terms
  for (const auto& measurement : imuData_) {
    // Create gyro error term
    // This would use GyroscopeError from kalibr_errorterms
  }

  // Add accelerometer error terms
  for (const auto& measurement : imuData_) {
    // Create accel error term
    // This would use AccelerometerError from kalibr_errorterms
  }
}

void IccImu::initBiasSplines(const aslam::splines::BSplinePose& poseSpline,
                             int biasKnotsPerSecond, int splineOrder) {
  std::cout << "Initializing bias splines..." << std::endl;

  double t_min = poseSpline.getMinTime();
  double t_max = poseSpline.getMaxTime();

  // Create gyro bias spline (3D Euclidean spline)
  // gyroBiasSpline_ = createBiasSpline(t_min, t_max, biasKnotsPerSecond,
  // splineOrder);

  // Create accel bias spline (3D Euclidean spline)
  // accelBiasSpline_ = createBiasSpline(t_min, t_max, biasKnotsPerSecond,
  // splineOrder);
}

// ============================================================================
// IccScaledMisalignedImu Implementation
// ============================================================================

IccScaledMisalignedImu::IccScaledMisalignedImu(const ImuParameters& imuConfig,
                                               const ImuDatasetReader& dataset)
    : IccImu(imuConfig, dataset) {
  // Initialize scale and misalignment matrices to identity
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

  M_gyro_ = std::make_shared<aslam::backend::MatrixDesignVariable>(I);
  M_accel_ = std::make_shared<aslam::backend::MatrixDesignVariable>(I);
}

void IccScaledMisalignedImu::addDesignVariables(
    aslam::backend::OptimizationProblem& problem,
    const aslam::splines::BSplinePose& poseSpline) {
  // Call base class
  IccImu::addDesignVariables(problem, poseSpline);

  // Add scale/misalignment matrices
  if (M_gyro_) {
    M_gyro_->setActive(true);
    problem.addDesignVariable(M_gyro_, CALIBRATION_GROUP_ID);
  }

  if (M_accel_) {
    M_accel_->setActive(true);
    problem.addDesignVariable(M_accel_, CALIBRATION_GROUP_ID);
  }
}

void IccScaledMisalignedImu::addErrorTerms(
    aslam::backend::OptimizationProblem& problem,
    const aslam::backend::BSplinePoseDesignVariable& poseDv,
    const aslam::backend::EuclideanExpression& gravityExpression,
    double gyroNoiseScale, double accelNoiseScale, double huberGyro,
    double huberAccel) {
  std::cout << "Adding scaled/misaligned IMU error terms..." << std::endl;

  // Similar to base class but multiply measurements by M matrices
}

// ============================================================================
// IccScaledMisalignedSizeEffectImu Implementation
// ============================================================================

IccScaledMisalignedSizeEffectImu::IccScaledMisalignedSizeEffectImu(
    const ImuParameters& imuConfig, const ImuDatasetReader& dataset)
    : IccScaledMisalignedImu(imuConfig, dataset) {
  // Initialize eccentric mounting vectors to zero
  r_x_ = std::make_shared<aslam::backend::EuclideanDesignVariable>(
      Eigen::Vector3d::Zero());
  r_y_ = std::make_shared<aslam::backend::EuclideanDesignVariable>(
      Eigen::Vector3d::Zero());
  r_z_ = std::make_shared<aslam::backend::EuclideanDesignVariable>(
      Eigen::Vector3d::Zero());
}

void IccScaledMisalignedSizeEffectImu::addDesignVariables(
    aslam::backend::OptimizationProblem& problem,
    const aslam::splines::BSplinePose& poseSpline) {
  // Call base class
  IccScaledMisalignedImu::addDesignVariables(problem, poseSpline);

  // Add size effect parameters
  r_x_->setActive(true);
  problem.addDesignVariable(r_x_, CALIBRATION_GROUP_ID);

  r_y_->setActive(true);
  problem.addDesignVariable(r_y_, CALIBRATION_GROUP_ID);

  r_z_->setActive(true);
  problem.addDesignVariable(r_z_, CALIBRATION_GROUP_ID);
}

void IccScaledMisalignedSizeEffectImu::addErrorTerms(
    aslam::backend::OptimizationProblem& problem,
    const aslam::backend::BSplinePoseDesignVariable& poseDv,
    const aslam::backend::EuclideanExpression& gravityExpression,
    double gyroNoiseScale, double accelNoiseScale, double huberGyro,
    double huberAccel) {
  std::cout << "Adding size effect IMU error terms..." << std::endl;

  // Use AccelerometerErrorEccentric which includes size effect terms
  // See AccelerometerError.cpp for reference
}

}  // namespace kalibr
