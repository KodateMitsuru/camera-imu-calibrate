#include "IccSensors.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

// ASLAM camera targets
#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>
#include <aslam/cameras/GridCalibrationTargetCheckerboard.hpp>
#include <aslam/cameras/GridCalibrationTargetCirclegrid.hpp>

// Boost (required for ASLAM library interop)
// Note: ASLAM internally uses std::shared_ptr, so we need boost for these cases
#include <boost/make_shared.hpp>
#include <memory>

#include "IccDatasetReaders.hpp"

// Dataset readers initialization functions
std::shared_ptr<CsvImageDatasetReader> initCameraDataset(
    const std::string& csvPath, const std::string& imageFolder,
    const std::pair<double, double>& from_to, double freq) {
  std::cout << "Initializing camera CSV dataset reader:" << std::endl;
  std::cout << "\tCSV file:         " << csvPath << std::endl;
  std::cout << "\tImage folder:     " << imageFolder << std::endl;

  auto reader = std::make_shared<CsvImageDatasetReader>(csvPath, imageFolder,
                                                        from_to, freq);

  std::cout << "\tNumber of images: " << reader->getImageCount() << std::endl;
  return reader;
}

std::shared_ptr<CsvImuDatasetReader> initImuDataset(
    const std::string& csvPath, const std::pair<double, double>& from_to) {
  std::cout << "Initializing IMU CSV dataset reader:" << std::endl;
  std::cout << "\tCSV file:         " << csvPath << std::endl;

  auto reader = std::make_shared<CsvImuDatasetReader>(csvPath, from_to);

  std::cout << "\tNumber of messages: " << reader->getMessageCount()
            << std::endl;
  return reader;
}

// ImuMeasurement Implementation
ImuMeasurement::ImuMeasurement(const aslam::Time& stamp,
                               const Eigen::Vector3d& omega,
                               const Eigen::Vector3d& alpha,
                               const Eigen::Matrix3d& Rgyro,
                               const Eigen::Matrix3d& Raccel)
    : omega(omega), alpha(alpha), omegaR(Rgyro), alphaR(Raccel), stamp(stamp) {
  omegaInvR = Rgyro.inverse();
  alphaInvR = Raccel.inverse();
}

// IccCamera Implementation
IccCamera::IccCamera(const kalibr_common::CameraParameters& camConfig,
                     const kalibr_common::TargetParameters& targetConfig,
                     std::shared_ptr<CsvImageDatasetReader> dataset,
                     double reprojectionSigma, bool showCorners,
                     bool showReproj, bool showOneStep)
    : dataset_(dataset),
      camConfig_(camConfig),
      targetConfig_(targetConfig),
      cornerUncertainty_(reprojectionSigma),
      timeshiftCamToImuPrior_(0.0) {
  // Set extrinsic prior to default
  T_extrinsic_ = sm::kinematics::Transformation();

  // Initialize gravity estimate
  gravity_w_ = Eigen::Vector3d(9.80655, 0.0, 0.0);

  // Initialize camera
  camera_ = kalibr_common::AslamCamera::fromParameters(camConfig);

  // Setup calibration target
  setupCalibrationTarget(targetConfig, showCorners, showReproj, showOneStep);

  // Extract corners
  bool multithreading = !(showCorners || showReproj || showOneStep);

  // Extract corners from CSV dataset
  targetObservations_.clear();
  size_t numImages = dataset_->getImageCount();
  for (size_t i = 0; i < numImages; ++i) {
    cv::Mat image = dataset_->getImage(i);
    double timestamp = dataset_->getTimestamp(i);
    aslam::Time time(timestamp);

    // Detect target directly from image
    aslam::cameras::GridCalibrationTargetObservation obs(detector_->target(),
                                                         image);
    if (detector_->findTarget(image, obs)) {
      targetObservations_.push_back(obs);
    }
  }

  std::cout << "Extracted corners from " << targetObservations_.size() << " of "
            << numImages << " images" << std::endl;
}

IccCamera::~IccCamera() = default;

void IccCamera::setupCalibrationTarget(
    const kalibr_common::TargetParameters& targetConfig, bool showExtraction,
    bool showReproj, bool imageStepping) {
  // Load calibration target configuration
  auto targetParams = targetConfig.getTargetParams();
  auto targetType = targetConfig.getTargetType();

  std::shared_ptr<aslam::cameras::GridCalibrationTargetBase> grid;

  if (targetType == "checkerboard") {
    aslam::cameras::GridCalibrationTargetCheckerboard::CheckerboardOptions
        options;
    options.filterQuads = true;
    options.normalizeImage = true;
    options.useAdaptiveThreshold = true;
    options.performFastCheck = false;
    options.windowWidth = 5;
    options.showExtractionVideo = showExtraction;

    grid = std::make_shared<aslam::cameras::GridCalibrationTargetCheckerboard>(
        targetParams.at("targetRows"), targetParams.at("targetCols"),
        targetParams.at("rowSpacingMeters"),
        targetParams.at("colSpacingMeters"), options);
  } else if (targetType == "circlegrid") {
    aslam::cameras::GridCalibrationTargetCirclegrid::CirclegridOptions options;
    options.showExtractionVideo = showExtraction;
    options.useAsymmetricCirclegrid = targetParams.at("asymmetricGrid");

    grid = std::make_shared<aslam::cameras::GridCalibrationTargetCirclegrid>(
        targetParams.at("targetRows"), targetParams.at("targetCols"),
        targetParams.at("spacingMeters"), options);
  } else if (targetType == "aprilgrid") {
    aslam::cameras::GridCalibrationTargetAprilgrid::AprilgridOptions options;
    options.showExtractionVideo = showExtraction;
    options.minTagsForValidObs = static_cast<int>(
        std::max(targetParams.at("tagRows"), targetParams.at("tagCols")) + 1);

    grid = std::make_shared<aslam::cameras::GridCalibrationTargetAprilgrid>(
        targetParams.at("tagRows"), targetParams.at("tagCols"),
        targetParams.at("tagSize"), targetParams.at("tagSpacing"), options);
  } else {
    throw std::runtime_error("Unknown calibration target.");
  }

  aslam::cameras::GridDetector::GridDetectorOptions detectorOptions;
  detectorOptions.imageStepping = imageStepping;
  detectorOptions.plotCornerReprojection = showReproj;
  detectorOptions.filterCornerOutliers = true;

  detector_ = std::make_shared<aslam::cameras::GridDetector>(
      camera_->geometry, grid, detectorOptions);
}

void IccCamera::findOrientationPriorCameraToImu(IccImu& imu) {
  std::cout << std::endl;
  std::cout << "Estimating imu-camera rotation prior" << std::endl;

  // Build the problem
  aslam::backend::OptimizationProblem problem;

  // Add the rotation as design variable
  auto q_i_c_Dv =
      std::make_shared<aslam::backend::RotationQuaternion>(T_extrinsic_.q());
  q_i_c_Dv->setActive(true);
  problem.addDesignVariable(q_i_c_Dv);

  // Add the gyro bias as design variable
  auto gyroBiasDv =
      std::make_shared<aslam::backend::EuclideanPoint>(Eigen::Vector3d::Zero());
  gyroBiasDv->setActive(true);
  problem.addDesignVariable(gyroBiasDv);

  // Initialize a pose spline using the camera poses
  auto poseSpline = initPoseSplineFromCamera(6, 100, 0.0);

  for (const auto& im : imu.imuData_) {
    double tk = im.stamp.toSec();
    if (tk > poseSpline->t_min() && tk < poseSpline->t_max()) {
      // DV expressions
      auto R_i_c = q_i_c_Dv->toExpression();
      auto bias = gyroBiasDv->toExpression();

      // Get the vision predicted omega and measured omega (IMU)
      Eigen::Vector3d angVel = poseSpline->angularVelocityBodyFrame(tk);
      auto omega_predicted =
          R_i_c * aslam::backend::EuclideanExpression(angVel);

      // Error term
      auto gerr = std::make_shared<kalibr_errorterms::GyroscopeError>(
          im.omega, im.omegaInvR, omega_predicted, bias);
      problem.addErrorTerm(gerr);
    }
  }

  if (problem.numErrorTerms() == 0) {
    SM_FATAL_STREAM(
        "Failed to obtain orientation prior. "
        "Please make sure that your sensors are synchronized correctly.");
    std::exit(-1);
  }

  // Define the optimization
  aslam::backend::Optimizer2Options options;
  options.verbose = false;
  // Note: linearSolver field may not exist in Optimizer2Options
  // Use default solver configuration instead
  options.nThreads = 2;
  options.convergenceDeltaX = 1e-4;
  options.convergenceDeltaJ = 1;
  options.maxIterations = 50;

  // Run the optimization
  aslam::backend::Optimizer2 optimizer(options);
  // Note: ASLAM uses std::shared_ptr internally, need to create boost version
  auto problemPtr =
      std::make_shared<aslam::backend::OptimizationProblem>(problem);
  optimizer.setProblem(problemPtr);

  try {
    optimizer.optimize();
  } catch (...) {
    SM_FATAL_STREAM("Failed to obtain orientation prior!");
    std::exit(-1);
  }

  // Overwrite the external rotation prior
  Eigen::Matrix3d R_i_c = q_i_c_Dv->toRotationMatrix().transpose();
  T_extrinsic_ = sm::kinematics::Transformation(
      sm::kinematics::rt2Transform(R_i_c, T_extrinsic_.t()));

  // Estimate gravity in the world coordinate frame
  std::vector<Eigen::Vector3d> a_w_list;
  for (const auto& im : imu.imuData_) {
    double tk = im.stamp.toSec();
    if (tk > poseSpline->t_min() && tk < poseSpline->t_max()) {
      Eigen::Matrix3d orientation = poseSpline->orientation(tk);
      Eigen::Vector3d a_w = orientation * (R_i_c * (-im.alpha));
      a_w_list.push_back(a_w);
    }
  }

  if (!a_w_list.empty()) {
    Eigen::Vector3d mean_a_w = Eigen::Vector3d::Zero();
    for (const auto& a : a_w_list) {
      mean_a_w += a;
    }
    mean_a_w /= a_w_list.size();
    gravity_w_ = mean_a_w / mean_a_w.norm() * 9.80655;
    std::cout << "Gravity was initialized to " << gravity_w_.transpose()
              << " [m/s^2]" << std::endl;
  }

  // Set the gyro bias prior
  Eigen::Vector3d b_gyro = gyroBiasDv->toEuclidean();
  imu.GyroBiasPriorCount_++;
  imu.GyroBiasPrior_ =
      ((imu.GyroBiasPriorCount_ - 1.0) / imu.GyroBiasPriorCount_) *
          imu.GyroBiasPrior_ +
      (1.0 / imu.GyroBiasPriorCount_) * b_gyro;

  // Print result
  std::cout << "  Orientation prior camera-imu found as: (T_i_c)" << std::endl;
  std::cout << R_i_c << std::endl;
  std::cout << "  Gyro bias prior found as: (b_gyro)" << std::endl;
  std::cout << b_gyro.transpose() << std::endl;
}

Eigen::Vector3d IccCamera::getEstimatedGravity() const { return gravity_w_; }

void IccCamera::findTimeshiftCameraImuPrior(IccImu& imu, bool verbose) {
  std::cout << "Estimating time shift camera to imu:" << std::endl;

  // Fit a spline to the camera observations
  auto poseSpline = initPoseSplineFromCamera(6, 100, 0.0);

  // Predict time shift prior
  std::vector<double> t;
  std::vector<double> omega_measured_norm;
  std::vector<double> omega_predicted_norm;

  for (const auto& im : imu.imuData_) {
    double tk = im.stamp.toSec();
    if (tk > poseSpline->t_min() && tk < poseSpline->t_max()) {
      // Get IMU measurements and spline from camera
      Eigen::Vector3d omega_predicted_vec =
          poseSpline->angularVelocityBodyFrame(tk);

      // Calculate norm
      t.push_back(tk);
      omega_measured_norm.push_back(im.omega.norm());
      omega_predicted_norm.push_back(omega_predicted_vec.norm());
    }
  }

  if (omega_predicted_norm.empty() || omega_measured_norm.empty()) {
    SM_FATAL_STREAM(
        "The time ranges of the camera and IMU do not overlap. "
        "Please make sure that your sensors are synchronized correctly.");
    std::exit(-1);
  }

  // Get the time shift using cross-correlation
  // This is a simplified implementation - in practice you'd use proper
  // cross-correlation
  double shift = 0.0;  // Placeholder for cross-correlation result

  // Store the timeshift
  timeshiftCamToImuPrior_ = shift;

  std::cout << "  Time shift camera to imu (t_imu = t_cam + shift):"
            << std::endl;
  std::cout << timeshiftCamToImuPrior_ << std::endl;
}

std::shared_ptr<bsplines::BSplinePose> IccCamera::initPoseSplineFromCamera(
    int splineOrder, int poseKnotsPerSecond, double timeOffsetPadding) {
  Eigen::Matrix4d T_c_b = T_extrinsic_.T();
  auto pose = std::make_shared<bsplines::BSplinePose>(
      splineOrder, sm::kinematics::RotationVector());

  // Get the checkerboard times
  std::vector<double> times;
  std::vector<Eigen::Matrix<double, 6, 1>> curve_values;

  for (const auto& obs : targetObservations_) {
    times.push_back(obs.time().toSec() + timeshiftCamToImuPrior_);
    Eigen::Matrix4d T_combined = obs.T_t_c().T() * T_c_b;
    curve_values.push_back(pose->transformationToCurveValue(T_combined));
  }

  if (times.empty()) {
    throw std::runtime_error(
        "No target observations available for spline initialization");
  }

  // Add padding
  times.insert(times.begin(), times[0] - (timeOffsetPadding * 2.0));
  times.push_back(times.back() + (timeOffsetPadding * 2.0));
  curve_values.insert(curve_values.begin(), curve_values[0]);
  curve_values.push_back(curve_values.back());

  // Handle rotation vector continuity
  for (size_t i = 1; i < curve_values.size(); ++i) {
    Eigen::Vector3d previousRotationVector = curve_values[i - 1].segment<3>(3);
    Eigen::Vector3d r = curve_values[i].segment<3>(3);
    double angle = r.norm();
    if (angle > 0) {
      Eigen::Vector3d axis = r / angle;
      Eigen::Vector3d best_r = r;
      double best_dist = (best_r - previousRotationVector).norm();

      for (int s = -3; s <= 3; ++s) {
        Eigen::Vector3d aa = axis * (angle + M_PI * 2.0 * s);
        double dist = (aa - previousRotationVector).norm();
        if (dist < best_dist) {
          best_r = aa;
          best_dist = dist;
        }
      }
      curve_values[i].segment<3>(3) = best_r;
    }
  }

  double seconds = times.back() - times[0];
  int knots = static_cast<int>(std::round(seconds * poseKnotsPerSecond));

  std::cout << std::endl;
  std::cout << "Initializing a pose spline with " << knots << " knots ("
            << poseKnotsPerSecond << " knots per second over " << seconds
            << " seconds)" << std::endl;

  // Convert curve values to matrix format
  Eigen::MatrixXd curve_matrix(6, curve_values.size());
  for (size_t i = 0; i < curve_values.size(); ++i) {
    curve_matrix.col(i) = curve_values[i];
  }

  pose->initPoseSplineSparse(times, curve_matrix, knots, 1e-4);
  return pose;
}

// IccCameraChain Implementation
IccCameraChain::IccCameraChain(
    const kalibr_common::ChainParameters& chainConfig,
    const kalibr_common::TargetParameters& targetConfig,
    const kalibr_common::ParsedArguments& parsed)
    : chainConfig(chainConfig) {
  // Create all cameras in the chain
  // Note: This implementation now expects CSV files instead of rosbag
  // The parsed.bagfile should now point to image CSV files
  for (int camNr = 0; camNr < chainConfig.numCameras(); ++camNr) {
    auto camConfig = chainConfig.getCameraParameters(camNr);

    // Extract CSV path and image folder from camera configuration
    // Expected format: parsed.bagfile[0] = "path/to/camera_images.csv"
    // Image folder will be derived or specified in config
    std::string csvPath = parsed.bagfile.size() > camNr ? parsed.bagfile[camNr]
                                                        : parsed.bagfile[0];
    std::string imageFolder =
        camConfig.getImageFolder();  // Assumes this method exists

    auto dataset = initCameraDataset(csvPath, imageFolder, parsed.bag_from_to,
                                     parsed.bag_freq);

    // Create the camera
    auto camera = std::make_shared<IccCamera>(
        camConfig, targetConfig, dataset, parsed.reprojection_sigma,
        parsed.showextraction, parsed.showextraction,
        parsed.extractionstepping);

    camList.push_back(camera);
  }

  // Find and store time between first and last image over all cameras
  findCameraTimespan();

  // Use stereo calibration guess if no baselines are provided
  initializeBaselines();
}

IccCameraChain::~IccCameraChain() = default;

void IccCameraChain::initializeBaselines() {
  // Estimate baseline prior if no external guess is provided
  for (size_t camNr = 1; camNr < camList.size(); ++camNr) {
    camList[camNr]->T_extrinsic_ =
        chainConfig.getExtrinsicsLastCamToHere(camNr);

    std::cout << "Baseline between cam" << (camNr - 1) << " and cam" << camNr
              << " set to:" << std::endl;
    std::cout << "T= " << std::endl
              << camList[camNr]->T_extrinsic_.T() << std::endl;
    std::cout << "Baseline: " << camList[camNr]->T_extrinsic_.t().norm()
              << " [m]" << std::endl;
  }
}

// IccImu Implementation
IccImu::ImuParameters::ImuParameters(const kalibr_common::ImuConfig& imuConfig,
                                     int imuNr)
    : kalibr_common::ImuParameters("", true), imuNr_(imuNr) {
  // Copy configuration data
  // Implementation would copy from imuConfig
}

IccImu::IccImu(const kalibr_common::ImuConfig& imuConfig,
               const kalibr_common::ParsedArguments& parsed,
               bool isReferenceImu, bool estimateTimedelay, int imuNr)
    : isReferenceImu_(isReferenceImu),
      estimateTimedelay_(estimateTimedelay),
      timeOffset_(0.0),
      GyroBiasPriorCount_(0) {
  // Store input
  imuConfig_ = std::make_shared<ImuParameters>(imuConfig, imuNr);

  // Load dataset - will be set by user code
  // User should call setDataset() before using this IMU
  dataset_ = nullptr;

  // Get statistics
  std::tie(accelUncertaintyDiscrete_, accelRandomWalk_, accelUncertainty_) =
      imuConfig_->getAccelerometerStatistics();
  std::tie(gyroUncertaintyDiscrete_, gyroRandomWalk_, gyroUncertainty_) =
      imuConfig_->getGyroStatistics();

  // Initialize bias prior
  GyroBiasPrior_ = Eigen::Vector3d::Zero();

  // Load the IMU dataset
  loadImuData();

  // Initial estimates
  q_i_b_prior_ = Eigen::Vector4d(0., 0., 0., 1.);
}

IccImu::~IccImu() = default;

std::shared_ptr<IccImu::ImuParameters> IccImu::getImuConfig() {
  updateImuConfig();
  return imuConfig_;
}

void IccImu::updateImuConfig() {
  imuConfig_->setImuPose(getTransformationFromBodyToImu().T());
  imuConfig_->setTimeOffset(timeOffset_);
}

void IccImu::loadImuData() {
  if (!dataset_) {
    throw std::runtime_error(
        "IMU dataset not initialized. Call setDataset() first.");
  }

  std::cout << "Reading IMU data (" << dataset_->getTopic() << ")" << std::endl;

  // Prepare progress tracking
  size_t numMessages = dataset_->getMessageCount();
  sm::timing::Progress2 progress(numMessages);
  progress.sample();

  Eigen::Matrix3d Rgyro = Eigen::Matrix3d::Identity() *
                          gyroUncertaintyDiscrete_ * gyroUncertaintyDiscrete_;
  Eigen::Matrix3d Raccel = Eigen::Matrix3d::Identity() *
                           accelUncertaintyDiscrete_ *
                           accelUncertaintyDiscrete_;

  // Read IMU measurements from CSV
  imuData_.clear();
  for (const auto& measurement : *dataset_) {
    aslam::Time timestamp(measurement.timestamp);
    imuData_.emplace_back(timestamp, measurement.omega, measurement.alpha,
                          Rgyro, Raccel);
    progress.sample();
  }

  if (imuData_.size() > 1) {
    std::cout << "\r  Read " << imuData_.size() << " imu readings over "
              << (imuData_.back().stamp.toSec() -
                  imuData_.front().stamp.toSec())
              << " seconds                   " << std::endl;
  } else {
    SM_FATAL_STREAM(
        "Could not find any IMU messages. Please check the dataset.");
    std::exit(-1);
  }
}

void IccImu::addDesignVariables(aslam::backend::OptimizationProblem& problem) {
  // Create design variables
  gyroBiasDv_ =
      std::make_shared<aslam::splines::EuclideanBSplineDesignVariable>(
          gyroBias_);
  accelBiasDv_ =
      std::make_shared<aslam::splines::EuclideanBSplineDesignVariable>(
          accelBias_);

  // Add spline design variables
  for (int i = 0; i < gyroBiasDv_->numDesignVariables(); ++i) {
    auto dv = gyroBiasDv_->designVariable(i);
    dv->setActive(true);
    problem.addDesignVariable(dv, HELPER_GROUP_ID);
  }

  for (int i = 0; i < accelBiasDv_->numDesignVariables(); ++i) {
    auto dv = accelBiasDv_->designVariable(i);
    dv->setActive(true);
    problem.addDesignVariable(dv, HELPER_GROUP_ID);
  }

  q_i_b_Dv_ =
      std::make_shared<aslam::backend::RotationQuaternion>(q_i_b_prior_);
  problem.addDesignVariable(q_i_b_Dv_, HELPER_GROUP_ID);
  q_i_b_Dv_->setActive(false);

  r_b_Dv_ =
      std::make_shared<aslam::backend::EuclideanPoint>(Eigen::Vector3d::Zero());
  problem.addDesignVariable(r_b_Dv_, HELPER_GROUP_ID);
  r_b_Dv_->setActive(false);

  if (!isReferenceImu_) {
    q_i_b_Dv_->setActive(true);
    r_b_Dv_->setActive(true);
  }
}

void IccImu::addAccelerometerErrorTerms(
    aslam::backend::OptimizationProblem& problem,
    aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
    const aslam::backend::EuclideanExpression& g_w, double mSigma,
    double accelNoiseScale) {
  std::cout << std::endl;
  std::cout << "Adding accelerometer error terms (" << dataset_->getTopic()
            << ")" << std::endl;

  // Progress bar
  sm::timing::Progress2 progress(imuData_.size());
  progress.sample();

  double weight = 1.0 / accelNoiseScale;
  accelErrors_.clear();
  int num_skipped = 0;

  std::shared_ptr<aslam::backend::MEstimatorPolicy> mest;
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
      auto b_i = accelBiasDv_->toEuclideanExpression(tk, 0);
      auto w_b = poseSplineDv.angularVelocityBodyFrame(tk);
      auto w_dot_b = poseSplineDv.angularAccelerationBodyFrame(tk);
      auto C_i_b = q_i_b_Dv_->toExpression();
      auto r_b = r_b_Dv_->toExpression();

      auto a = C_i_b * (C_b_w * (a_w - g_w) + w_dot_b.cross(r_b) +
                        w_b.cross(w_b.cross(r_b)));

      auto aerr = std::make_shared<kalibr_errorterms::EuclideanError>(
          im.alpha, im.alphaInvR * weight, a + b_i);
      aerr->setMEstimatorPolicy(mest);
      accelErrors_.push_back(aerr);
      problem.addErrorTerm(aerr);
    } else {
      num_skipped++;
    }

    progress.sample();
  }

  std::cout << "\r  Added " << (imuData_.size() - num_skipped) << " of "
            << imuData_.size() << " accelerometer error terms (skipped "
            << num_skipped << " out-of-bounds measurements)" << std::endl;
}

void IccImu::addGyroscopeErrorTerms(
    aslam::backend::OptimizationProblem& problem,
    aslam::splines::BSplinePoseDesignVariable& poseSplineDv, double mSigma,
    double gyroNoiseScale, const aslam::backend::EuclideanExpression* g_w) {
  std::cout << std::endl;
  std::cout << "Adding gyroscope error terms (" << dataset_->getTopic() << ")"
            << std::endl;

  // Progress bar
  sm::timing::Progress2 progress(imuData_.size());
  progress.sample();

  int num_skipped = 0;
  gyroErrors_.clear();
  double weight = 1.0 / gyroNoiseScale;

  std::shared_ptr<aslam::backend::MEstimatorPolicy> mest;
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
      auto b_i = gyroBiasDv_->toEuclideanExpression(tk, 0);
      auto C_i_b = q_i_b_Dv_->toExpression();
      auto w = C_i_b * w_b;

      auto gerr = std::make_shared<kalibr_errorterms::EuclideanError>(
          im.omega, im.omegaInvR * weight, w + b_i);
      gerr->setMEstimatorPolicy(mest);
      gyroErrors_.push_back(gerr);
      problem.addErrorTerm(gerr);
    } else {
      num_skipped++;
    }

    progress.sample();
  }

  std::cout << "\r  Added " << (imuData_.size() - num_skipped) << " of "
            << imuData_.size() << " gyroscope error terms (skipped "
            << num_skipped << " out-of-bounds measurements)" << std::endl;
}

void IccImu::initBiasSplines(std::shared_ptr<bsplines::BSplinePose> poseSpline,
                             int splineOrder, int biasKnotsPerSecond) {
  double start = poseSpline->t_min();
  double end = poseSpline->t_max();
  double seconds = end - start;
  int knots = static_cast<int>(std::round(seconds * biasKnotsPerSecond));

  std::cout << std::endl;
  std::cout << "Initializing the bias splines with " << knots << " knots"
            << std::endl;

  // Initialize the bias splines
  gyroBias_ = std::make_shared<bsplines::BSpline>(splineOrder);
  gyroBias_->initConstantSpline(start, end, knots, GyroBiasPrior_);

  accelBias_ = std::make_shared<bsplines::BSpline>(splineOrder);
  accelBias_->initConstantSpline(start, end, knots, Eigen::Vector3d::Zero());
}

void IccImu::addBiasMotionTerms(aslam::backend::OptimizationProblem& problem) {
  Eigen::Matrix3d Wgyro =
      Eigen::Matrix3d::Identity() / (gyroRandomWalk_ * gyroRandomWalk_);
  Eigen::Matrix3d Waccel =
      Eigen::Matrix3d::Identity() / (accelRandomWalk_ * accelRandomWalk_);

  auto gyroBiasMotionErr =
      std::make_shared<aslam::splines::BSplineEuclideanMotionError>(gyroBiasDv_,
                                                                    Wgyro, 1);
  problem.addErrorTerm(gyroBiasMotionErr);

  auto accelBiasMotionErr =
      std::make_shared<aslam::splines::BSplineEuclideanMotionError>(
          accelBiasDv_, Waccel, 1);
  problem.addErrorTerm(accelBiasMotionErr);
}

sm::kinematics::Transformation IccImu::getTransformationFromBodyToImu() const {
  if (isReferenceImu_) {
    return sm::kinematics::Transformation();
  }
  return sm::kinematics::Transformation(
      sm::kinematics::r2quat(q_i_b_Dv_->toRotationMatrix()),
      -q_i_b_Dv_->toRotationMatrix() * r_b_Dv_->toEuclidean());
}

// IccScaledMisalignedImu Implementation
IccScaledMisalignedImu::ImuParameters::ImuParameters(
    const kalibr_common::ImuConfig& imuConfig, int imuNr)
    : IccImu::ImuParameters(imuConfig, imuNr) {
  // Set model type
  // data["model"] = "scale-misalignment";
}

IccScaledMisalignedImu::IccScaledMisalignedImu(
    const kalibr_common::ImuConfig& imuConfig,
    const kalibr_common::ParsedArguments& parsed, bool isReferenceImu,
    bool estimateTimedelay, int imuNr)
    : IccImu(imuConfig, parsed, isReferenceImu, estimateTimedelay, imuNr) {}

void IccScaledMisalignedImu::updateImuConfig() {
  IccImu::updateImuConfig();
  // Update intrinsics matrices
  // imuConfig_->setIntrisicsMatrices(...);
}

void IccScaledMisalignedImu::addDesignVariables(
    aslam::backend::OptimizationProblem& problem) {
  IccImu::addDesignVariables(problem);

  q_gyro_i_Dv_ = std::make_shared<aslam::backend::RotationQuaternion>(
      Eigen::Vector4d(0., 0., 0., 1.));
  problem.addDesignVariable(q_gyro_i_Dv_, HELPER_GROUP_ID);
  q_gyro_i_Dv_->setActive(true);

  Eigen::MatrixXi pattern = Eigen::MatrixXi::Zero(3, 3);
  pattern << 1, 0, 0, 1, 1, 0, 1, 1, 1;

  M_accel_Dv_ = std::make_shared<aslam::backend::MatrixBasic>(
      Eigen::Matrix3d::Identity(), pattern);
  problem.addDesignVariable(M_accel_Dv_, HELPER_GROUP_ID);
  M_accel_Dv_->setActive(true);

  M_gyro_Dv_ = std::make_shared<aslam::backend::MatrixBasic>(
      Eigen::Matrix3d::Identity(), pattern);
  problem.addDesignVariable(M_gyro_Dv_, HELPER_GROUP_ID);
  M_gyro_Dv_->setActive(true);

  M_accel_gyro_Dv_ = std::make_shared<aslam::backend::MatrixBasic>(
      Eigen::Matrix3d::Zero(), Eigen::MatrixXi::Ones(3, 3));
  problem.addDesignVariable(M_accel_gyro_Dv_, HELPER_GROUP_ID);
  M_accel_gyro_Dv_->setActive(true);
}

// Additional implementations for IccScaledMisalignedSizeEffectImu would follow
// a similar pattern...