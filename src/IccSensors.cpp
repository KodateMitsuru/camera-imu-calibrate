#include <ceres/ceres.h>
#include <ceres/numeric_diff_cost_function.h>

#include <Eigen/Core>
#include <IccSensors.hpp>
#include <algorithm>
#include <any>
#include <aslam/backend/BSplineMotionError.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/RotationExpression.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/SimpleReprojectionError.hpp>
#include <aslam/backend/TransformationExpression.hpp>
#include <aslam/cameras/CameraGeometry.hpp>
#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>
#include <aslam/cameras/GridCalibrationTargetBase.hpp>
#include <aslam/cameras/GridCalibrationTargetCheckerboard.hpp>
#include <aslam/cameras/GridCalibrationTargetCirclegrid.hpp>
#include <aslam/splines/BSplineDesignVariable.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <bsplines/BSplinePose.hpp>
#include <cmath>
#include <functional>
#include <iostream>
#include <kalibr_backend/TransformationDesignVariable.hpp>
#include <kalibr_common/TargetExtractor.hpp>
#include <kalibr_errorterms/AccelerometerError.hpp>
#include <kalibr_errorterms/GyroscopeError.hpp>
#include <memory>
#include <opencv2/highgui.hpp>
#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/transformations.hpp>
#include <string>
#include <unordered_map>

#include "aslam/Time.hpp"
#include "aslam/backend/BlockCholeskyLinearSystemSolver.hpp"
#include "aslam/backend/MEstimatorPolicies.hpp"
#include "aslam/backend/Scalar.hpp"
#include "bsplines/BSpline.hpp"
#include "kalibr_common/ConfigReader.hpp"
#include "kalibr_errorterms/EuclideanError.hpp"
#include "sm/kinematics/Transformation.hpp"
#include "sm/kinematics/quaternion_algebra.hpp"

namespace kalibr {
// ============================================================================
// IccCamera Implementation
// ============================================================================

IccCamera::IccCamera(const CameraParameters& camConfig,
                     const CalibrationTargetParameters& targetConfig,
                     const ImageDatasetReader& dataset,
                     double reprojectionSigma, bool showCorners,
                     bool showReproj, bool showOneStep)
    : dataset_(dataset), camConfig_(camConfig), targetConfig_(targetConfig) {
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
  targetObservations_ =
      extractCornersFromDataset(dataset_, *detector_,
                                multithreading,  // multithreading
                                0,               // auto-detect num processes
                                true,            // clear images
                                false            // no transformation
      );

  // An estimate of the gravity in the world coordinate frame
  gravity_w_ = Eigen::Vector3d(9.80655, 0.0, 0.0);

  std::cout << "Camera initialized with " << targetObservations_.size()
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

  detector_ = std::make_shared<aslam::cameras::GridDetector>(
      camera_->getGeometry(), grid, detectorOptions);

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
    double tk = im.stamp.toSec();

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
          omega_measured, im.omegaInvR, omega_predicted, bias);
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
    double tk = im.stamp.toSec();
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
    double tk = im.stamp.toSec();

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
      times.push_back(im.stamp.toSec());
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
  int numObs = targetObservations_.size();
  if (numObs == 0) {
    throw std::runtime_error(
        "No observations available for spline initialization");
  }

  Eigen::VectorXd times(numObs);
  Eigen::Matrix<double, 6, Eigen::Dynamic> curve(6, numObs);

  // Extract times and transform observations to curve values
  for (int i = 0; i < numObs; ++i) {
    const auto& obs = targetObservations_[i];
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

void IccCamera::addCameraErrorTerms(
    aslam::backend::OptimizationProblemBase& problem,
    const std::shared_ptr<aslam::splines::BSplinePoseDesignVariable>&
        poseSplineDv,
    const aslam::backend::TransformationExpression& T_cN_b,
    double blakeZissermanDf, double timeOffsetPadding) {
  std::cout << "\nAdding camera error terms (" << dataset_.getTopic() << ")"
            << std::endl;

  // Clear previous reprojection errors
  allReprojectionErrors_.clear();
  int totalErrorTerms = 0;

  // Get reprojection error type from camera
  // In C++, we use the camera geometry directly

  // Loop over all observations
  for (const auto& obs : targetObservations_) {
    // Build a transformation expression for the time
    // frameTime = cameraTimeToImuTimeDv + obs.time() + timeshiftCamToImuPrior
    auto frameTime = cameraTimeToImuTimeDv_->toExpression() +
                     obs.time().toSec() + timeshiftCamToImuPrior_;
    double frameTimeScalar = frameTime.toScalar();

    // As we are applying an initial time shift outside the optimization,
    // we need to make sure that we don't add data outside the spline definition
    if (frameTimeScalar <= poseSplineDv->spline().t_min() ||
        frameTimeScalar >= poseSplineDv->spline().t_max()) {
      continue;
    }

    // Get T_w_b: transformation from world to body at this time
    auto T_w_b = poseSplineDv->transformationAtTime(
        frameTime, timeOffsetPadding, timeOffsetPadding);
    auto T_b_w = T_w_b.inverse();

    // Calibration target coords to camera N coords
    // T_b_w: from world to imu coords
    // T_cN_b: from imu to camera N coords
    // T_c_w = T_cN_b * T_b_w
    auto T_c_w = T_cN_b * T_b_w;

    // Get the image and target points corresponding to the frame
    std::vector<cv::Point2f> imageCornerPoints;
    std::vector<cv::Point3f> targetCornerPoints;
    obs.getCornersImageFrame(imageCornerPoints);
    obs.getCornersTargetFrame(targetCornerPoints);

    // Corner uncertainty (inverse covariance)
    Eigen::Matrix2d R =
        Eigen::Matrix2d::Identity() * cornerUncertainty_ * cornerUncertainty_;
    Eigen::Matrix2d invR = R.inverse();

    // Setup an aslam frame (handles the distortion)
    // Note: In C++ aslam, frame handling is done through the camera geometry
    // We create error terms directly

    // Store reprojection errors for this observation
    std::vector<std::shared_ptr<aslam::backend::ErrorTerm>> reprojectionErrors;

    // Add error term for each corner point
    for (size_t pidx = 0; pidx < imageCornerPoints.size(); ++pidx) {
      // Get measured image point
      Eigen::Vector2d y(imageCornerPoints[pidx].x, imageCornerPoints[pidx].y);

      // Get target point (add homogeneous coordinate)
      Eigen::Vector4d targetPoint;
      targetPoint << targetCornerPoints[pidx].x, targetCornerPoints[pidx].y,
          targetCornerPoints[pidx].z, 1.0;

      // Transform target point to camera frame
      auto p = T_c_w * aslam::backend::HomogeneousExpression(targetPoint);

      // Create reprojection error using camera's factory method
      // 这里使用了之前存储的具体类型工厂 - 无需反射！
      auto rerr = camera_->createReprojectionError(y, invR, p);

      // Add Blake-Zisserman M-estimator if requested
      if (blakeZissermanDf > 0.0) {
        auto mest = std::make_shared<aslam::backend::BlakeZissermanMEstimator>(
            blakeZissermanDf);
        rerr->setMEstimatorPolicy(mest);
      }

      problem.addErrorTerm(rerr);
      reprojectionErrors.push_back(rerr);
      totalErrorTerms++;
    }

    allReprojectionErrors_.push_back(reprojectionErrors);
  }

  std::cout << "  Added " << totalErrorTerms << " camera error terms"
            << std::endl;
}

// ============================================================================
// IccCameraChain Implementation
// ============================================================================

IccCameraChain::IccCameraChain(const CameraChainParameters& chainConfig,
                               const CalibrationTargetParameters& targetConfig,
                               const std::vector<ImageDatasetReader>& datasets)
    : chainConfig_(chainConfig) {
  if (chainConfig.numCameras() != datasets.size()) {
    throw std::runtime_error(
        "Number of camera configs must match number of datasets");
  }

  // Create camera instances
  for (size_t i = 0; i < chainConfig.numCameras(); ++i) {
    auto camConfig = chainConfig.getCameraParameters(i);
    auto& dataset = datasets[i];
    camList_.push_back(std::make_shared<IccCamera>(
        camConfig, targetConfig, dataset, camConfig->getReprojectionSigma()));
  }

  this->findCameraTimespan();
  this->initializeBaselines();

  std::cout << "Initialized camera chain with " << camList_.size() << " cameras"
            << std::endl;
}

void IccCameraChain::initializeBaselines() {
  for (size_t i = 1; i < camList_.size(); ++i) {
    camList_[i]->getExtrinsic() = chainConfig_.getExtrinsicsLastCamToHere(i);
    std::cout << "Baseline between cam" << i - 1 << " and cam" << i
              << " set to:" << std::endl;
    std::cout << "T= " << camList_[i]->getExtrinsic().T() << std::endl;
    std::cout << "Baseline: " << camList_[i]->getExtrinsic().t().norm()
              << " [m]" << std::endl;
  }
}

bsplines::BSplinePose IccCameraChain::initializePoseSplineFromCameraChain(
    int splineOrder, int poseKnotsPerSecond, double timeOffsetPadding) {
  if (camList_.empty()) {
    throw std::runtime_error("No cameras in chain");
  }

  // Use the first camera to initialize the spline
  return camList_[0]->initPoseSplineFromCamera(splineOrder, poseKnotsPerSecond,
                                               timeOffsetPadding);
}

void IccCameraChain::findCameraTimespan() {
  if (camList_.empty()) {
    throw std::runtime_error("No cameras in chain");
  }

  auto tStart = aslam::Time(0.0);
  auto tEnd = aslam::Time(0.0);

  for (auto cam : camList_) {
    if (!cam->getObservations().empty()) {
      auto tStartCam = cam->getObservations().front().time();
      auto tEndCam = cam->getObservations().back().time();

      if (tStart.toSec() > tStartCam.toSec()) {
        tStart = tStartCam;
      }
      if (tEnd.toSec() < tEndCam.toSec()) {
        tEnd = tEndCam;
      }
    }
  }

  timeStart_ = tStart;
  timeEnd_ = tEnd;
}

void IccCameraChain::findOrientationPriorCameraChainToImu(const IccImu& imu) {
  camList_[0]->findOrientationPriorCameraToImu(imu);
}

Eigen::Vector3d IccCameraChain::getEstimatedGravity() {
  if (camList_.empty()) {
    throw std::runtime_error("No cameras in chain");
  }
  return camList_[0]->getEstimatedGravity();
}

std::pair<sm::kinematics::Transformation, double>
IccCameraChain::getResultBaseline(int fromCamNr, int toCamNr) const {
  int idx = std::max(fromCamNr, toCamNr);

  auto T_cb_ca =
      sm::kinematics::Transformation(camList_[idx]->getExtrinsicDv()->T());

  if (fromCamNr > toCamNr) {
    T_cb_ca = T_cb_ca.inverse();
  }

  auto baseline = T_cb_ca.t().norm();
  return {T_cb_ca, baseline};
}

sm::kinematics::Transformation IccCameraChain::getResultTrafoImuToCam(
    int camNr) const {
  auto T_c0_i =
      sm::kinematics::Transformation(camList_[0]->getExtrinsicDv()->T());
  auto T_cN_imu = T_c0_i;
  for (auto cam : std::ranges::subrange(camList_.begin() + 1,
                                        camList_.begin() + camNr + 1)) {
    auto T_cNplus1_cN =
        sm::kinematics::Transformation(cam->getExtrinsicDv()->T());
    T_cN_imu = T_cNplus1_cN * T_cN_imu;
  }

  return T_cN_imu;
}

double IccCameraChain::getResultTimeshift(int camNr) const {
  return camList_[camNr]->getTimeOffsetDv()->toScalar() +
         camList_[camNr]->getTimeshiftPrior();
}

void IccCameraChain::addCameraChainerrorTerms(
    aslam::backend::OptimizationProblemBase& problem,
    const std::shared_ptr<aslam::splines::BSplinePoseDesignVariable>&
        poseSplineDv,
    double blakeZissermanDf, double timeOffsetPadding) {
  for (auto [camNr, cam] : std::ranges::views::enumerate(camList_)) {
    aslam::backend::TransformationExpression T_chain;
    if (camNr == 0) {
      // First camera: use T_i_b
      T_chain = cam->getExtrinsicDv()->toExpression();
    } else {
      T_chain = cam->getExtrinsicDv()->toExpression() * T_chain;
    }

    auto T_cN_b = T_chain;

    cam->addCameraErrorTerms(problem, poseSplineDv, T_cN_b, blakeZissermanDf,
                             timeOffsetPadding);

  }
}

// ============================================================================
// IccImu Implementation
// ============================================================================

IccImu::ImuParameters::ImuParameters(const ::kalibr::ImuParameters& imuConfig,
                                     int imuNr)
    : ::kalibr::ImuParameters("", true) {
  data_ = imuConfig.getYamlDict();
  data_["model"] = "calibrated";
  imuNr_ = imuNr;
}

void IccImu::ImuParameters::setImuPose(
    const sm::kinematics::Transformation& T_i_b) {
  Eigen::MatrixXd t = T_i_b.T();
  data_["T_i_b"] = std::vector<double>(t.data(), t.data() + t.size());
}

void IccImu::ImuParameters::setTimeOffset(double time_offset) {
  data_["time_offset"] = time_offset;
}

std::string kalibr::IccImu::ImuParameters::formatIndented(
    const std::string& indent, std::vector<double> array) const {
  // indent + str(np.array_str(np_array)).replace('\n',"\n"+indent)
  std::string s = indent + "[";
  bool first = true;
  for (const auto& v : array) {
    if (!first) {
      s += ", ";
    }
    s += std::to_string(v);
    first = false;
  }
  s += "]";
  return s;
}

void IccImu::ImuParameters::printDetails(std::ostream& out) const {
  out << "  Model: " << std::any_cast<std::string>(data_.at("model"))
      << std::endl;
  ::kalibr::ImuParameters::printDetails(out);
  out << "  T_ib (imu0 to imu" << imuNr_ << ")" << std::endl;
  out << formatIndented("    ",
                        std::any_cast<std::vector<double>>(data_.at("T_i_b")))
      << std::endl;
  out << "  time offset with respect to IMU0: "
      << std::any_cast<double>(data_.at("time_offset")) << " [s]" << std::endl;
}

void IccImu::updateImuConfig() {
  // Update the imuConfig_ from the current priors
  imuConfig_.setImuPose(getTransformationFromBodyToImu());
  imuConfig_.setTimeOffset(timeOffset_);
}

const IccImu::ImuParameters& IccImu::getImuConfig() {
  updateImuConfig();
  return imuConfig_;
}

void IccImu::loadImuData() {
  std::cout << "Loading IMU data..." << std::endl;
  Eigen::Matrix3d Rgyro = Eigen::Matrix3d::Identity() *
                          this->gyroUncertaintyDiscrete_ *
                          this->gyroUncertaintyDiscrete_;
  Eigen::Matrix3d Raccel = Eigen::Matrix3d::Identity() *
                           this->accelUncertaintyDiscrete_ *
                           this->accelUncertaintyDiscrete_;

  std::vector<ImuMeasurement> data;
  for (auto [timestamp, omega, alpha] : dataset_.getAllMessages()) {
    data.emplace_back(timestamp, omega, alpha, Rgyro, Raccel);
  }
  imuData_.swap(data);

  if (imuData_.empty()) {
    throw std::runtime_error("No IMU data loaded");
  } else {
    std::cout << "  Read " << imuData_.size() << " imu readings over "
              << (imuData_.back().stamp.toSec() -
                  imuData_.front().stamp.toSec())
              << " seconds" << std::endl;
  }
}

IccImu::IccImu(const ::kalibr::ImuParameters& imuConfig,
               const ImuDatasetReader& dataset, bool isReferenceImu,
               bool estimateTimeDelay, int imuNr)
    : isReferenceImu_(isReferenceImu),
      estimateTimeDelay_(estimateTimeDelay),
      dataset_(dataset),
      imuConfig_(IccImu::ImuParameters(imuConfig, imuNr)) {
  auto accelerometerStatistics = imuConfig.getAccelerometerStatistics();
  accelUncertaintyDiscrete_ = std::get<0>(accelerometerStatistics);
  accelRandomWalk_ = std::get<1>(accelerometerStatistics);
  accelUncertainty_ = std::get<2>(accelerometerStatistics);
  auto gyroscopeStatistics = imuConfig.getGyroscopeStatistics();
  gyroUncertaintyDiscrete_ = std::get<0>(gyroscopeStatistics);
  gyroRandomWalk_ = std::get<1>(gyroscopeStatistics);
  gyroUncertainty_ = std::get<2>(gyroscopeStatistics);
  this->loadImuData();
}



void IccImu::addAccelerometerErrorTerms(
    aslam::backend::OptimizationProblemBase& problem,
    aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
    const aslam::backend::EuclideanExpression& g_w, double mSigma,
    double accelNoiseScale) {
  std::cout << "\nAdding accelerometer error terms..." << std::endl;
  double weight = 1.0 / (accelNoiseScale);
  std::vector<std::shared_ptr<aslam::backend::ErrorTerm>> accelErrors;
  int num_skipped = 0;

  std::shared_ptr<aslam::backend::MEstimator> mest;
  if (mSigma > 0.0) {
    mest = std::make_shared<aslam::backend::HuberMEstimator>(mSigma);
  } else {
    mest = std::make_shared<aslam::backend::NoMEstimator>();
  }

  for (const auto& im : imuData_) {
    double tk = im.stamp.toSec() + timeOffset_;

    if (tk > poseSplineDv.spline().t_min() &&
        tk < poseSplineDv.spline().t_max()) {
      auto C_b_w = poseSplineDv.orientation(tk).inverse();
      auto a_w = poseSplineDv.linearAcceleration(tk);
      auto b_i =
          std::get<
              std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable>>(
              accelBiasDv_)
              ->toEuclideanExpression(tk, 0);
      auto w_b = poseSplineDv.angularVelocityBodyFrame(tk);
      auto w_dot_b = poseSplineDv.angularAccelerationBodyFrame(tk);
      auto C_i_b = q_i_b_Dv_->toExpression();
      auto r_b = r_b_Dv_->toExpression();
      auto a = C_i_b * (C_b_w * (a_w - g_w) + w_dot_b.cross(r_b) +
                        w_b.cross(w_b.cross(r_b)));
      auto aerr = std::make_shared<kalibr_errorterms::EuclideanError>(
          im.alpha, im.alphaInvR * weight, a + b_i);
      aerr->setMEstimatorPolicy(mest);
      accelErrors.push_back(aerr);
      problem.addErrorTerm(aerr);
    } else {
      num_skipped++;
    }
  }
  std::cout << "  Added " << imuData_.size() - num_skipped << " of "
            << imuData_.size() << " accelerometer error terms (skipped "
            << num_skipped << " out-of-bounds measurements)" << std::endl;
  accelErrors_.swap(accelErrors);
}

void IccImu::addGyroscopeErrorTerms(
    aslam::backend::OptimizationProblemBase& problem,
    aslam::splines::BSplinePoseDesignVariable& poseSplineDv, double mSigma,
    double gyroNoiseScale, const aslam::backend::EuclideanExpression&) {
  std::cout << "\nAdding gyroscope error terms..." << std::endl;
  double weight = 1.0 / (gyroNoiseScale);
  std::vector<std::shared_ptr<aslam::backend::ErrorTerm>> gyroErrors;
  int num_skipped = 0;

  std::shared_ptr<aslam::backend::MEstimator> mest;
  if (mSigma > 0.0) {
    mest = std::make_shared<aslam::backend::HuberMEstimator>(mSigma);
  } else {
    mest = std::make_shared<aslam::backend::NoMEstimator>();
  }

  for (const auto& im : imuData_) {
    double tk = im.stamp.toSec() + timeOffset_;

    if (tk > poseSplineDv.spline().t_min() &&
        tk < poseSplineDv.spline().t_max()) {
      auto b_i =
          std::get<
              std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable>>(
              gyroBiasDv_)
              ->toEuclideanExpression(tk, 0);
      auto w_b = poseSplineDv.angularVelocityBodyFrame(tk);
      auto C_i_b = q_i_b_Dv_->toExpression();
      auto w = C_i_b * w_b;
      auto gerr = std::make_shared<kalibr_errorterms::EuclideanError>(
          im.omega, im.omegaInvR * weight, w + b_i);
      gerr->setMEstimatorPolicy(mest);
      gyroErrors.push_back(gerr);
      problem.addErrorTerm(gerr);
    } else {
      num_skipped++;
    }
  }
  std::cout << "  Added " << imuData_.size() - num_skipped << " of "
            << imuData_.size() << " accelerometer error terms (skipped "
            << num_skipped << " out-of-bounds measurements)" << std::endl;
  gyroErrors_.swap(gyroErrors);
}

void IccImu::initBiasSplines(const bsplines::BSplinePose& poseSpline,
                             int splineOrder, int biasKnotsPerSecond) {
  auto start = poseSpline.t_min();
  auto end = poseSpline.t_max();
  auto seconds = end - start;
  int knots = static_cast<int>(std::round(seconds * biasKnotsPerSecond));
  std::cout << "\nInitializing the bias splines with " << knots << " knots"
            << std::endl;

  gyroBias_ = std::make_shared<bsplines::BSpline>(splineOrder);
  gyroBias_->initConstantSpline(start, end, knots, GyroBiasPrior_);

  accelBias_ = std::make_shared<bsplines::BSpline>(splineOrder);
  accelBias_->initConstantSpline(start, end, knots, Eigen::Vector3d::Zero());
}

void IccImu::addBiasMotionTerms(aslam::backend::OptimizationProblemBase& problem) {
  Eigen::Matrix3d Wgyro =
      Eigen::Matrix3d::Identity() / (gyroRandomWalk_ * gyroRandomWalk_);
  Eigen::Matrix3d Waccel =
      Eigen::Matrix3d::Identity() / (accelRandomWalk_ * accelRandomWalk_);
  auto gyroBiasMotionErr = std::make_shared<aslam::backend::BSplineMotionError<
      aslam::splines::EuclideanBSplineDesignVariable>>(
      std::get<std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable>>(
          gyroBiasDv_)
          .get(),
      Wgyro);
  problem.addErrorTerm(gyroBiasMotionErr);
  auto accelBiasMotionErr = std::make_shared<aslam::backend::BSplineMotionError<
      aslam::splines::EuclideanBSplineDesignVariable>>(
      std::get<std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable>>(
          accelBiasDv_)
          .get(),
      Waccel);
  problem.addErrorTerm(accelBiasMotionErr);
}

sm::kinematics::Transformation IccImu::getTransformationFromBodyToImu() const {
  if (isReferenceImu_) {
    return sm::kinematics::Transformation();
  } else {
    return sm::kinematics::Transformation(
        sm::kinematics::r2quat(q_i_b_Dv_->toRotationMatrix()),
        -(q_i_b_Dv_->toRotationMatrix() * r_b_Dv_->toEuclidean()));
  }
}

void IccImu::findOrientationPrior(const IccImu& referenceImu) {
  std::cout << "\nEstimating imu-imu rotation initial guess..." << std::endl;
  auto problem = std::make_shared<aslam::backend::OptimizationProblem>();
  auto q_i_b_Dv = std::make_shared<aslam::backend::RotationQuaternion>(
      Eigen::Matrix3d::Identity().eval());
  q_i_b_Dv->setActive(true);
  problem->addDesignVariable(q_i_b_Dv);

  auto startTime = imuData_.front().stamp.toSec();
  auto endTime = imuData_.back().stamp.toSec();
  int knotsPerSecond = 50;
  int knots =
      static_cast<int>(std::round((endTime - startTime) * knotsPerSecond));

  auto angularVelocity = bsplines::BSpline(3);
  angularVelocity.initConstantSpline(startTime, endTime, knots,
                                     Eigen::Vector3d::Zero());
  auto angularVelocityDv =
      std::make_shared<aslam::splines::EuclideanBSplineDesignVariable>(
          angularVelocity);

  for (size_t i = 0; i < angularVelocityDv->numDesignVariables(); ++i) {
    std::shared_ptr<aslam::backend::DesignVariable> dv(
        angularVelocityDv->designVariable(i));
    dv->setActive(true);
    problem->addDesignVariable(dv);
  }
  auto referenceGyroBiasDv =
      std::make_shared<aslam::backend::EuclideanPoint>(Eigen::Vector3d::Zero());
  referenceGyroBiasDv->setActive(true);
  problem->addDesignVariable(referenceGyroBiasDv);

  for (const auto& im : referenceImu.imuData_) {
    double tk = im.stamp.toSec();
    if (tk > angularVelocity.t_min() && tk < angularVelocity.t_max()) {
      auto bias = referenceGyroBiasDv->toExpression();

      auto omega_predicted = angularVelocityDv->toEuclideanExpression(tk, 0);
      auto omega_measured = im.omega;

      auto gerr = std::make_shared<kalibr_errorterms::GyroscopeError>(
          omega_measured, im.omegaInvR, omega_predicted, bias);
      problem->addErrorTerm(gerr);
    }
  }

  auto options = aslam::backend::Optimizer2Options();
  options.verbose = false;
  options.linearSystemSolver =
      std::make_shared<aslam::backend::BlockCholeskyLinearSystemSolver>();
  options.nThreads = 2;
  options.convergenceDeltaX = 1e-4;
  options.convergenceDeltaJ = 1;
  options.maxIterations = 50;

  auto optimizer = aslam::backend::Optimizer2(options);
  optimizer.setProblem(problem);

  try {
    optimizer.optimize();
  } catch (std::exception& e) {
    std::cerr << "Failed to obtain initial guess for the relative orientation!"
              << std::endl;
    throw e;
  }

  std::function<Eigen::VectorXd(double)> referenceAbsoluteOmega =
      [this, &angularVelocity, &angularVelocityDv](double dt = 0.0) {
        std::vector<double> norms;
        for (const auto& im : imuData_) {
          double t = im.stamp.toSec() + dt;
          if (t > angularVelocity.t_min() && t < angularVelocity.t_max()) {
            norms.push_back(angularVelocityDv->toEuclidean(t, 0).norm());
          }
        }
        return Eigen::Map<Eigen::VectorXd>(norms.data(), norms.size());
      };

  std::function<Eigen::VectorXd(double)> absoluteOmega =
      [this, &angularVelocity](double dt = 0.0) {
        std::vector<double> norms;
        for (const auto& im : imuData_) {
          double t = im.stamp.toSec() + dt;
          if (t > angularVelocity.t_min() && t < angularVelocity.t_max()) {
            norms.push_back(im.omega.norm());
          }
        }
        return Eigen::Map<Eigen::VectorXd>(norms.data(), norms.size());
      };
  if (referenceAbsoluteOmega(0.0).size() == 0 ||
      absoluteOmega(0.0).size() == 0) {
    throw std::runtime_error(
        "No overlapping IMU data for orientation prior estimation");
  }

  auto corr = [](const Eigen::VectorXd& a, const Eigen::VectorXd& v) {
    const int n = a.size();
    const int m = v.size();
    const int outLen = n + m - 1;

    Eigen::VectorXd out = Eigen::VectorXd::Zero(outLen);

    // 滑动点积
    for (int lag = 0; lag < outLen; ++lag) {
      int i0 = std::max(0, lag - m + 1);  // a 的起始下标
      int j0 = std::max(0, m - lag - 1);  // v 的起始下标
      int span = std::min(lag + 1, n) - i0;

      for (int k = 0; k < span; ++k) {
        out[lag] += a[i0 + k] * v[j0 + k];
      }
    }
    return out;
  }(referenceAbsoluteOmega(0.0), absoluteOmega(0.0));

  Eigen::Index discrete_shift;
  corr.maxCoeff(&discrete_shift);

  std::vector<double> times;
  for (auto im : imuData_) {
    times.push_back(im.stamp.toSec());
  }
  auto dT = (times.back() - times.front()) / (times.size() - 1);
  auto shift = discrete_shift * dT;

  if (estimateTimeDelay_ && !isReferenceImu_) {
    ceres::Problem problem;
    problem.AddResidualBlock(
        new ceres::NumericDiffCostFunction<ShiftCost, ceres::CENTRAL, 1, 1>(
            new ShiftCost(referenceAbsoluteOmega, absoluteOmega)),
        nullptr, &shift);
    auto options = ceres::Solver::Options();
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    timeOffset_ = shift;
  }

  std::cout << "Temporal correction with respect to reference IMU "
            << std::endl;
  std::cout << timeOffset_ << "[s]";
  if (!estimateTimeDelay_) {
    std::cout << " (this offset is not accounted for in the calibration)"
              << std::endl;
  } else {
    std::cout << std::endl;
  }

  gyroBiasDv_ =
      std::make_shared<aslam::backend::EuclideanPoint>(Eigen::Vector3d::Zero());
  std::get<std::shared_ptr<aslam::backend::EuclideanPoint>>(gyroBiasDv_)
      ->setActive(true);
  problem->addDesignVariable(
      std::get<std::shared_ptr<aslam::backend::EuclideanPoint>>(gyroBiasDv_));
  for (auto& im : imuData_) {
    auto tk = im.stamp.toSec() + timeOffset_;
    if (tk > angularVelocity.t_min() && tk < angularVelocity.t_max()) {
      auto C_i_b = q_i_b_Dv->toExpression();
      auto bias =
          std::get<std::shared_ptr<aslam::backend::EuclideanPoint>>(gyroBiasDv_)
              ->toExpression();

      auto omega_predicted =
          C_i_b * angularVelocityDv->toEuclideanExpression(tk, 0);
      auto omega_measured = im.omega;

      auto gerr = std::make_shared<kalibr_errorterms::GyroscopeError>(
          omega_measured, im.omegaInvR, omega_predicted, bias);
      problem->addErrorTerm(gerr);
    }
  }

  try {
    optimizer.optimize();
  } catch (std::exception& e) {
    std::cerr << "Failed to obtain initial guess for the relative orientation!"
              << std::endl;
    throw e;
  }

  std::cout << "Estimated imu to reference imu Rotation: " << std::endl;
  std::cout << q_i_b_Dv->toRotationMatrix() << std::endl;

  q_i_b_Prior_ = sm::kinematics::r2quat(q_i_b_Dv->toRotationMatrix());
};

// ============================================================================
// IccScaledMisalignedImu Implementation
// ============================================================================

IccScaledMisalignedImu::ImuParameters::ImuParameters(
    const ::kalibr::ImuParameters& imuConfig, int imuNr)
    : IccImu::ImuParameters(imuConfig, imuNr) {
  data_ = imuConfig.getYamlDict();
  data_["model"] = "scale-misalignment";
}

void IccScaledMisalignedImu::ImuParameters::printDetails(
    std::ostream& out) const {
  IccImu::ImuParameters::printDetails(out);
  auto gyroscopes = std::any_cast<std::unordered_map<std::string, std::any>>(
      data_.at("gyroscopes"));
  out << "  Gyroscope: " << std::endl;
  out << "    M:" << std::endl;
  out << formatIndented("      ",
                        std::any_cast<std::vector<double>>(gyroscopes.at("M")))
      << std::endl;
  out << "    A [(rad/s)/(m/s^2)]:" << std::endl;
  out << formatIndented("    ",
                        std::any_cast<std::vector<double>>(gyroscopes.at("A")))
      << std::endl;
  out << "    C_gyro_i:" << std::endl;
  out << formatIndented("    ", std::any_cast<std::vector<double>>(
                                    gyroscopes.at("C_gyro_i")))
      << std::endl;
  out << "  Accelerometer: " << std::endl;
  out << "    M:" << std::endl;
  auto accelerometers =
      std::any_cast<std::unordered_map<std::string, std::any>>(
          data_.at("accelerometers"));
  out << formatIndented("      ", std::any_cast<std::vector<double>>(
                                      accelerometers.at("M")))
      << std::endl;
}

void IccScaledMisalignedImu::ImuParameters::setIntrisicsMatrices(
    const Eigen::Matrix3d& M_accel, const Eigen::Matrix3d& C_gyro_i,
    const Eigen::Matrix3d& M_gyro, const Eigen::Matrix3d& Ma_gyro) {
  std::unordered_map<std::string, std::any> accelerometers;
  accelerometers["M"] =
      std::vector<double>(M_accel.data(), M_accel.data() + M_accel.size());
  data_["accelerometers"] = accelerometers;
  std::unordered_map<std::string, std::any> gyroscopes;
  gyroscopes["M"] =
      std::vector<double>(M_gyro.data(), M_gyro.data() + M_gyro.size());
  gyroscopes["A"] =
      std::vector<double>(Ma_gyro.data(), Ma_gyro.data() + Ma_gyro.size());
  gyroscopes["C_gyro_i"] =
      std::vector<double>(C_gyro_i.data(), C_gyro_i.data() + C_gyro_i.size());
  data_["gyroscopes"] = gyroscopes;
}

IccScaledMisalignedImu::IccScaledMisalignedImu(
    const ::kalibr::ImuParameters& imuConfig, const ImuDatasetReader& dataset,
    bool isReferenceImu, bool estimateTimeDelay, int imuNr)
    : IccImu(imuConfig, dataset, isReferenceImu, estimateTimeDelay, imuNr),
      imuConfig_(imuConfig, imuNr) {}

void IccScaledMisalignedImu::updateImuConfig() {
  IccImu::updateImuConfig();
  imuConfig_.setIntrisicsMatrices(
      M_accel_Dv_->toMatrix3x3(), q_gyro_i_Dv_->toRotationMatrix(),
      M_gyro_Dv_->toMatrix3x3(), M_accel_gyro_Dv_->toMatrix3x3());
}

const IccScaledMisalignedImu::ImuParameters&
IccScaledMisalignedImu::getImuConfig() {
  updateImuConfig();
  return imuConfig_;
}



void IccScaledMisalignedImu::addAccelerometerErrorTerms(
    aslam::backend::OptimizationProblemBase& problem,
    aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
    const aslam::backend::EuclideanExpression& g_w, double mSigma,
    double accelNoiseScale) {
  std::cout << "\nAdding accelerometer error terms..." << std::endl;
  double weight = 1.0 / (accelNoiseScale);
  std::vector<std::shared_ptr<aslam::backend::ErrorTerm>> accelErrors;
  int num_skipped = 0;

  std::shared_ptr<aslam::backend::MEstimator> mest;
  if (mSigma > 0.0) {
    mest = std::make_shared<aslam::backend::HuberMEstimator>(mSigma);
  } else {
    mest = std::make_shared<aslam::backend::NoMEstimator>();
  }

  for (const auto& im : imuData_) {
    double tk = im.stamp.toSec() + timeOffset_;

    if (tk > poseSplineDv.spline().t_min() &&
        tk < poseSplineDv.spline().t_max()) {
      auto C_b_w = poseSplineDv.orientation(tk).inverse();
      auto a_w = poseSplineDv.linearAcceleration(tk);
      auto b_i =
          std::get<
              std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable>>(
              accelBiasDv_)
              ->toEuclideanExpression(tk, 0);
      auto M = M_accel_Dv_->toExpression();
      auto w_b = poseSplineDv.angularVelocityBodyFrame(tk);
      auto w_dot_b = poseSplineDv.angularAccelerationBodyFrame(tk);
      auto C_i_b = q_i_b_Dv_->toExpression();
      auto r_b = r_b_Dv_->toExpression();
      auto a = M * (C_i_b * (C_b_w * (a_w - g_w) + w_dot_b.cross(r_b) +
                             w_b.cross(w_b.cross(r_b))));
      auto aerr = std::make_shared<kalibr_errorterms::EuclideanError>(
          im.alpha, im.alphaInvR * weight, a + b_i);
      aerr->setMEstimatorPolicy(mest);
      accelErrors.push_back(aerr);
      problem.addErrorTerm(aerr);
    } else {
      num_skipped++;
    }
  }
  std::cout << "  Added " << imuData_.size() - num_skipped << " of "
            << imuData_.size() << " accelerometer error terms (skipped "
            << num_skipped << " out-of-bounds measurements)" << std::endl;
  accelErrors_.swap(accelErrors);
}

void IccScaledMisalignedImu::addGyroscopeErrorTerms(
    aslam::backend::OptimizationProblemBase& problem,
    aslam::splines::BSplinePoseDesignVariable& poseSplineDv, double mSigma,
    double gyroNoiseScale, const aslam::backend::EuclideanExpression& g_w) {
  std::cout << "\nAdding gyroscope error terms..." << std::endl;
  double weight = 1.0 / (gyroNoiseScale);
  std::vector<std::shared_ptr<aslam::backend::ErrorTerm>> gyroErrors;
  int num_skipped = 0;

  std::shared_ptr<aslam::backend::MEstimator> mest;
  if (mSigma > 0.0) {
    mest = std::make_shared<aslam::backend::HuberMEstimator>(mSigma);
  } else {
    mest = std::make_shared<aslam::backend::NoMEstimator>();
  }

  for (const auto& im : imuData_) {
    double tk = im.stamp.toSec() + timeOffset_;

    if (tk > poseSplineDv.spline().t_min() &&
        tk < poseSplineDv.spline().t_max()) {
      auto w_b = poseSplineDv.angularVelocityBodyFrame(tk);
      auto w_dot_b = poseSplineDv.angularAccelerationBodyFrame(tk);
      auto b_i =
          std::get<
              std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable>>(
              gyroBiasDv_)
              ->toEuclideanExpression(tk, 0);
      auto C_b_w = poseSplineDv.orientation(tk).inverse();
      auto a_w = poseSplineDv.linearAcceleration(tk);
      auto r_b = r_b_Dv_->toExpression();
      auto a_b =
          C_b_w * (a_w - g_w) + w_dot_b.cross(r_b) + w_b.cross(w_b.cross(r_b));
      auto C_i_b = q_i_b_Dv_->toExpression();
      auto C_gyro_i = q_gyro_i_Dv_->toExpression();
      auto C_gyro_b = C_gyro_i * C_i_b;
      auto M = M_gyro_Dv_->toExpression();
      auto Ma = M_accel_gyro_Dv_->toExpression();
      auto w = M * (C_gyro_b * w_b) + Ma * (C_gyro_b * a_b);
      auto gerr = std::make_shared<kalibr_errorterms::EuclideanError>(
          im.omega, im.omegaInvR * weight, w + b_i);
      gerr->setMEstimatorPolicy(mest);
      gyroErrors.push_back(gerr);
      problem.addErrorTerm(gerr);
    } else {
      num_skipped++;
    }
  }
  std::cout << "  Added " << imuData_.size() - num_skipped << " of "
            << imuData_.size() << " accelerometer error terms (skipped "
            << num_skipped << " out-of-bounds measurements)" << std::endl;
  gyroErrors_.swap(gyroErrors);
}

// ============================================================================
// IccScaledMisalignedSizeEffectImu Implementation
// ============================================================================

IccScaledMisalignedSizeEffectImu::ImuParameters::ImuParameters(
    const ::kalibr::ImuParameters& imuConfig, int imuNr)
    : IccScaledMisalignedImu::ImuParameters(imuConfig, imuNr) {
  data_ = imuConfig.getYamlDict();
  data_["model"] = "scale-misalignment-size-effect";
}

void IccScaledMisalignedSizeEffectImu::ImuParameters::printDetails(
    std::ostream& out) const {
  IccScaledMisalignedImu::ImuParameters::printDetails(out);
  auto accelerometers =
      std::any_cast<std::unordered_map<std::string, std::any>>(
          data_.at("accelerometers"));
  out << "    rx_i [(m)]: " << std::endl;
  out << formatIndented("      ", std::any_cast<std::vector<double>>(
                                      accelerometers.at("rx_i")))
      << std::endl;
  out << "    ry_i [(m)]: " << std::endl;
  out << formatIndented("      ", std::any_cast<std::vector<double>>(
                                      accelerometers.at("ry_i")))
      << std::endl;
  out << "    rz_i [(m)]: " << std::endl;
  out << formatIndented("      ", std::any_cast<std::vector<double>>(
                                      accelerometers.at("rz_i")))
      << std::endl;
}

void IccScaledMisalignedSizeEffectImu::ImuParameters::setAccelerometerLeverArms(
    const Eigen::Vector3d& rx_i, const Eigen::Vector3d& ry_i,
    const Eigen::Vector3d& rz_i) {
  std::unordered_map<std::string, std::any> accelerometers;
  accelerometers["rx_i"] =
      std::vector<double>(rx_i.data(), rx_i.data() + rx_i.size());
  accelerometers["ry_i"] =
      std::vector<double>(ry_i.data(), ry_i.data() + ry_i.size());
  accelerometers["rz_i"] =
      std::vector<double>(rz_i.data(), rz_i.data() + rz_i.size());
  data_["accelerometers"] = accelerometers;
}

IccScaledMisalignedSizeEffectImu::IccScaledMisalignedSizeEffectImu(
    const ::kalibr::ImuParameters& imuConfig, const ImuDatasetReader& dataset,
    bool isReferenceImu, bool estimateTimeDelay, int imuNr)
    : IccScaledMisalignedImu(imuConfig, dataset, isReferenceImu,
                             estimateTimeDelay, imuNr),
      imuConfig_(imuConfig, imuNr) {}



void IccScaledMisalignedSizeEffectImu::updateImuConfig() {
  IccScaledMisalignedImu::updateImuConfig();
  imuConfig_.setAccelerometerLeverArms(rx_i_Dv_->toEuclidean(),
                                       ry_i_Dv_->toEuclidean(),
                                       rz_i_Dv_->toEuclidean());
}

const IccScaledMisalignedSizeEffectImu::ImuParameters&
IccScaledMisalignedSizeEffectImu::getImuConfig() {
  updateImuConfig();
  return imuConfig_;
}

void IccScaledMisalignedSizeEffectImu::addAccelerometerErrorTerms(
    aslam::backend::OptimizationProblemBase& problem,
    aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
    const aslam::backend::EuclideanExpression& g_w, double mSigma,
    double accelNoiseScale) {
  std::cout << "\nAdding accelerometer error terms..." << std::endl;
  double weight = 1.0 / (accelNoiseScale);
  std::vector<std::shared_ptr<aslam::backend::ErrorTerm>> accelErrors;
  int num_skipped = 0;

  std::shared_ptr<aslam::backend::MEstimator> mest;
  if (mSigma > 0.0) {
    mest = std::make_shared<aslam::backend::HuberMEstimator>(mSigma);
  } else {
    mest = std::make_shared<aslam::backend::NoMEstimator>();
  }

  for (const auto& im : imuData_) {
    double tk = im.stamp.toSec() + timeOffset_;

    if (tk > poseSplineDv.spline().t_min() &&
        tk < poseSplineDv.spline().t_max()) {
      auto C_b_w = poseSplineDv.orientation(tk).inverse();
      auto a_w = poseSplineDv.linearAcceleration(tk);
      auto b_i =
          std::get<
              std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable>>(
              accelBiasDv_)
              ->toEuclideanExpression(tk, 0);
      auto M = M_accel_Dv_->toExpression();
      auto w_b = poseSplineDv.angularVelocityBodyFrame(tk);
      auto w_dot_b = poseSplineDv.angularAccelerationBodyFrame(tk);
      auto C_i_b = q_i_b_Dv_->toExpression();
      auto rx_b =
          r_b_Dv_->toExpression() + C_i_b.inverse() * rx_i_Dv_->toExpression();
      auto ry_b =
          r_b_Dv_->toExpression() + C_i_b.inverse() * ry_i_Dv_->toExpression();
      auto rz_b =
          r_b_Dv_->toExpression() + C_i_b.inverse() * rz_i_Dv_->toExpression();
      auto Ix = Ix_Dv_->toExpression();
      auto Iy = Iy_Dv_->toExpression();
      auto Iz = Iz_Dv_->toExpression();
      auto a =
          M *
          (C_i_b * (C_b_w * (a_w - g_w)) +
           Ix * (C_i_b * (w_dot_b.cross(rx_b) + w_b.cross(w_b.cross(rx_b)))) +
           Iy * (C_i_b * (w_dot_b.cross(ry_b) + w_b.cross(w_b.cross(ry_b)))) +
           Iz * (C_i_b * (w_dot_b.cross(rz_b) + w_b.cross(w_b.cross(rz_b)))));
      auto aerr = std::make_shared<kalibr_errorterms::EuclideanError>(
          im.alpha, im.alphaInvR * weight, a + b_i);
      aerr->setMEstimatorPolicy(mest);
      accelErrors.push_back(aerr);
      problem.addErrorTerm(aerr);
    } else {
      num_skipped++;
    }
  }
  std::cout << "  Added " << imuData_.size() - num_skipped << " of "
            << imuData_.size() << " accelerometer error terms (skipped "
            << num_skipped << " out-of-bounds measurements)" << std::endl;
  accelErrors_.swap(accelErrors);
}
}  // namespace kalibr
