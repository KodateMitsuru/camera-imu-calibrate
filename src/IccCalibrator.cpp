#include "IccCalibrator.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

// Utility function implementation
void addSplineDesignVariables(
    aslam_incremental_calibration::CalibrationOptimizationProblem* problem,
    aslam::splines::BSplinePoseDesignVariable* dvc, bool setActive,
    int group_id) {
  // Implementation would depend on actual aslam API
  // for (int i = 0; i < dvc->numDesignVariables(); ++i) {
  //     auto dv = dvc->designVariable(i);
  //     dv->setActive(setActive);
  //     problem->addDesignVariable(dv, group_id);
  // }
}

IccCalibrator::IccCalibrator() {
  // Initialize member variables
  ImuList.clear();
  CameraChain = nullptr;
  poseDv = nullptr;
  gravityDv = nullptr;
  gravityExpression = nullptr;
  problem = nullptr;
  optimizer = nullptr;
  noTimeCalibration = false;
}

IccCalibrator::~IccCalibrator() {
  // Clean up resources
  if (optimizer) {
    delete optimizer;
    optimizer = nullptr;
  }
  // Note: Other pointers might be managed by aslam libraries
}

void IccCalibrator::initDesignVariables(
    aslam_incremental_calibration::CalibrationOptimizationProblem* problem,
    aslam::splines::BSpline* poseSpline, bool noTimeCalibration,
    bool noChainExtrinsics, bool estimateGravityLength,
    const Eigen::Vector3d& initialGravityEstimate) {
  // Initialize the system pose spline (always attached to imu0)
  // poseDv = new aslam::splines::BSplinePoseDesignVariable(*poseSpline);
  // addSplineDesignVariables(problem, poseDv);

  // Add the calibration target orientation design variable (expressed as
  // gravity vector in target frame)
  if (estimateGravityLength) {
    // gravityDv = new aslam::backend::EuclideanPointDv(initialGravityEstimate);
  } else {
    // gravityDv = new
    // aslam::backend::EuclideanDirection(initialGravityEstimate);
  }

  // gravityExpression = gravityDv->toExpression();
  // gravityDv->setActive(true);
  // problem->addDesignVariable(gravityDv, HELPER_GROUP_ID);

  // Add all DVs for all IMUs
  for (auto imu : ImuList) {
    // imu->addDesignVariables(problem);
  }

  // Add all DVs for the camera chain
  if (CameraChain) {
    // CameraChain->addDesignVariables(problem, noTimeCalibration,
    // noChainExtrinsics);
  }
}

void IccCalibrator::addPoseMotionTerms(
    aslam_incremental_calibration::CalibrationOptimizationProblem* problem,
    double tv, double rv) {
  double wt = 1.0 / tv;
  double wr = 1.0 / rv;

  Eigen::Matrix<double, 6, 6> W = Eigen::Matrix<double, 6, 6>::Zero();
  W.diagonal() << wt, wt, wt, wr, wr, wr;

  // Implementation would use aslam splines API
  // aslam::splines::addMotionErrorTerms(problem, poseDv, W, errorOrder);
}

void IccCalibrator::registerCamChain(CameraChainType* sensor) {
  CameraChain = sensor;
}

void IccCalibrator::registerImu(ImuType* sensor) { ImuList.push_back(sensor); }

void IccCalibrator::buildProblem(
    int splineOrder, int poseKnotsPerSecond, int biasKnotsPerSecond,
    bool doPoseMotionError, double mrTranslationVariance,
    double mrRotationVariance, bool doBiasMotionError, int blakeZisserCam,
    double huberAccel, double huberGyro, bool noTimeCalibration,
    bool noChainExtrinsics, int maxIterations, double gyroNoiseScale,
    double accelNoiseScale, double timeOffsetPadding, bool verbose) {
  // Print configuration
  std::cout << "\tSpline order: " << splineOrder << std::endl;
  std::cout << "\tPose knots per second: " << poseKnotsPerSecond << std::endl;
  std::cout << "\tDo pose motion regularization: " << std::boolalpha
            << doPoseMotionError << std::endl;
  std::cout << "\t\txddot translation variance: " << mrTranslationVariance
            << std::endl;
  std::cout << "\t\txddot rotation variance: " << mrRotationVariance
            << std::endl;
  std::cout << "\tBias knots per second: " << biasKnotsPerSecond << std::endl;
  std::cout << "\tDo bias motion regularization: " << std::boolalpha
            << doBiasMotionError << std::endl;
  std::cout << "\tBlake-Zisserman on reprojection errors " << blakeZisserCam
            << std::endl;
  std::cout << "\tAcceleration Huber width (sigma): " << huberAccel
            << std::endl;
  std::cout << "\tGyroscope Huber width (sigma): " << huberGyro << std::endl;
  std::cout << "\tDo time calibration: " << std::boolalpha
            << (!noTimeCalibration) << std::endl;
  std::cout << "\tMax iterations: " << maxIterations << std::endl;
  std::cout << "\tTime offset padding: " << timeOffsetPadding << std::endl;

  ////////////////////////////////////////////
  // Initialize camera chain
  ////////////////////////////////////////////
  this->noTimeCalibration = noTimeCalibration;

  if (!noTimeCalibration && CameraChain) {
    // Estimate the timeshift for all cameras to the main imu
    // for (auto cam : CameraChain->camList) {
    //     cam->findTimeshiftCameraImuPrior(ImuList[0], verbose);
    // }
  }

  if (CameraChain && !ImuList.empty()) {
    // Obtain orientation prior between main imu and camera chain
    // CameraChain->findOrientationPriorCameraChainToImu(ImuList[0]);
    // Eigen::Vector3d estimatedGravity = CameraChain->getEstimatedGravity();
    Eigen::Vector3d estimatedGravity(0.0, 9.81, 0.0);  // Default fallback

    ////////////////////////////////////////////
    // Init optimization problem
    ////////////////////////////////////////////
    // Initialize a pose spline using the camera poses in the camera chain
    // auto poseSpline =
    // CameraChain->initializePoseSplineFromCameraChain(splineOrder,
    // poseKnotsPerSecond, timeOffsetPadding);

    // Initialize bias splines for all IMUs
    for (auto imu : ImuList) {
      // imu->initBiasSplines(poseSpline, splineOrder, biasKnotsPerSecond);
    }

    // Now build the problem
    // problem = new
    // aslam_incremental_calibration::CalibrationOptimizationProblem();

    // Initialize all design variables
    // initDesignVariables(problem, poseSpline, noTimeCalibration,
    // noChainExtrinsics, false, estimatedGravity);

    ////////////////////////////////////////////
    // Add error terms
    ////////////////////////////////////////////
    // Add calibration target reprojection error terms for all cameras in chain
    // CameraChain->addCameraChainErrorTerms(problem, poseDv, blakeZisserCam,
    // timeOffsetPadding);

    // Initialize IMU error terms
    for (auto imu : ImuList) {
      // imu->addAccelerometerErrorTerms(problem, poseDv, gravityExpression,
      // huberAccel, accelNoiseScale); imu->addGyroscopeErrorTerms(problem,
      // poseDv, huberGyro, gyroNoiseScale, gravityExpression);

      // Add the bias motion terms
      if (doBiasMotionError) {
        // imu->addBiasMotionTerms(problem);
      }
    }

    // Add the pose motion terms
    if (doPoseMotionError) {
      addPoseMotionTerms(problem, mrTranslationVariance, mrRotationVariance);
    }

    // Add a gravity prior
    // this->problem = problem;
  }
}

void IccCalibrator::optimize(aslam::backend::Optimizer2Options* options,
                             int maxIterations, bool recoverCov) {
  if (!options) {
    // options = new aslam::backend::Optimizer2Options();
    // options->verbose = true;
    // options->doLevenbergMarquardt = true;
    // options->levenbergMarquardtLambdaInit = 10.0;
    // options->nThreads = std::max(1,
    // static_cast<int>(std::thread::hardware_concurrency()) - 1);
    // options->convergenceDeltaX = 1e-5;
    // options->convergenceDeltaJ = 1e-2;
    // options->maxIterations = maxIterations;
    // options->trustRegionPolicy = new
    // aslam::backend::LevenbergMarquardtTrustRegionPolicy(options->levenbergMarquardtLambdaInit);
    // options->linearSolver = new
    // aslam::backend::BlockCholeskyLinearSystemSolver();
  }

  // Run the optimization
  // optimizer = new aslam::backend::Optimizer2(options);
  // optimizer->setProblem(problem);

  bool optimizationFailed = false;
  try {
    // auto retval = optimizer->optimize();
    // if (retval.linearSolverFailure) {
    //     optimizationFailed = true;
    // }
  } catch (const std::exception& e) {
    sm::logError(std::string(e.what()));
    optimizationFailed = true;
  }

  if (optimizationFailed) {
    sm::logError("Optimization failed!");
    throw std::runtime_error("Optimization failed!");
  }

  // Free some memory
  if (optimizer) {
    delete optimizer;
    optimizer = nullptr;
  }

  if (recoverCov) {
    recoverCovariance();
  }
}

void IccCalibrator::recoverCovariance() {
  // Covariance ordering (=dv ordering)
  // ORDERING:   N=num cams
  //            1. transformation imu-cam0 --> 6
  //            2. camera time2imu --> 1*numCams (only if enabled)

  std::cout << "Recovering covariance..." << std::endl;

  // aslam_incremental_calibration::IncrementalEstimator
  // estimator(CALIBRATION_GROUP_ID); auto rval = estimator.addBatch(problem,
  // true); Eigen::VectorXd est_stds =
  // estimator.getSigma2Theta().diagonal().array().sqrt();

  // Split and store the variance
  // std_trafo_ic = est_stds.segment(0, 6);
  // std_times = est_stds.segment(6, est_stds.size() - 6);
}

void IccCalibrator::saveImuSetParametersYaml(const std::string& resultFile) {
  // kalibr_common::ImuSetParameters imuSetConfig(resultFile, true);

  for (auto imu : ImuList) {
    // auto imuConfig = imu->getImuConfig();
    // imuSetConfig.addImuParameters(imuConfig);
  }

  // imuSetConfig.writeYaml(resultFile);
}

void IccCalibrator::saveCamChainParametersYaml(const std::string& resultFile) {
  if (!CameraChain) {
    throw std::runtime_error("CameraChain is null");
  }

  // auto chain = CameraChain->chainConfig;
  // int nCams = static_cast<int>(CameraChain->camList.size());

  // Calibration results
  // for (int camNr = 0; camNr < nCams; ++camNr) {
  //     // cam-cam baselines
  //     if (camNr > 0) {
  //         auto [T_cB_cA, baseline] = CameraChain->getResultBaseline(camNr -
  //         1, camNr); chain->setExtrinsicsLastCamToHere(camNr, T_cB_cA);
  //     }

  //     // imu-cam trafos
  //     auto T_ci = CameraChain->getResultTrafoImuToCam(camNr);
  //     chain->setExtrinsicsImuToCam(camNr, T_ci);

  //     if (!noTimeCalibration) {
  //         // imu to cam timeshift
  //         double timeshift = CameraChain->getResultTimeShift(camNr);
  //         chain->setTimeshiftCamImu(camNr, timeshift);
  //     }
  // }

  try {
    // chain->writeYaml(resultFile);
  } catch (...) {
    throw std::runtime_error("ERROR: Could not write parameters to file: " +
                             resultFile);
  }
}

// Placeholder implementation for sm::logError if not available
namespace sm {
void logError(const std::string& message) {
  std::cerr << "ERROR: " << message << std::endl;
}
}  // namespace sm
