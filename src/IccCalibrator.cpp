#include <Eigen/Core>
#include <IccCalibrator.hpp>
#include <IccSensors.hpp>
#include <aslam/backend/BSplineMotionErrorFactory.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/EuclideanDirection.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/OptimizerOptions.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <cstddef>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include "aslam/backend/BlockCholeskyLinearSystemSolver.hpp"
#include "aslam/backend/DesignVariable.hpp"
#include "aslam/backend/LevenbergMarquardtTrustRegionPolicy.hpp"
#include "aslam/calibration/core/IncrementalEstimator.h"
#include "bsplines/BSplinePose.hpp"

namespace kalibr {

IccCalibrator::IccCalibrator() = default;

IccCalibrator::~IccCalibrator() = default;

void IccCalibrator::initDesignVariables(
    aslam::calibration::OptimizationProblem& problem,
    const bsplines::BSplinePose& poseSpline, bool noTimeCalibration,
    bool noChainExtrinsics, bool estimateGravityLength,
    const Eigen::Vector3d& initialGravityEstimate) {
  // Initialize the system pose spline (always attached to imu0)
  poseDv_ =
      std::make_shared<aslam::splines::BSplinePoseDesignVariable>(poseSpline);
  addSplineDesignVariables(problem, *poseDv_);

  // Add the calibration target orientation design variable
  // (expressed as gravity vector in target frame)
  if (estimateGravityLength) {
    gravityDv_ = std::make_shared<aslam::backend::EuclideanPoint>(
        initialGravityEstimate);
    gravityExpression_ = std::make_shared<aslam::backend::EuclideanExpression>(
        ((aslam::backend::EuclideanPoint*)gravityDv_.get())->toExpression());
  } else {
    gravityDv_ = std::make_shared<aslam::backend::EuclideanDirection>(
        initialGravityEstimate);
    gravityExpression_ = std::make_shared<aslam::backend::EuclideanExpression>(
        ((aslam::backend::EuclideanDirection*)gravityDv_.get())
            ->toExpression());
  }

  // Provide appropriate arguments to toExpression, e.g., time = 0.0 and
  // derivative order = 0

  gravityDv_->setActive(true);
  problem.addDesignVariable(gravityDv_,
                            HELPER_GROUP_ID);

  // Add all DVs for all IMUs
  for (auto& imu : imuList_) {
    imu->addDesignVariables(problem);
  }

  // Add all DVs for the camera chain
  if (cameraChain_) {
    cameraChain_->addDesignVariables(problem, noTimeCalibration,
                                     noChainExtrinsics);
  }
}

void IccCalibrator::addPoseMotionTerms(
    aslam::calibration::OptimizationProblem& problem, double tv, double rv) {
  double wt = 1.0 / tv;
  double wr = 1.0 / rv;

  Eigen::Matrix<double, 6, 6> W;
  W.setZero();
  W.diagonal() << wt, wt, wt, wr, wr, wr;

  // Add motion error terms
  // Define the error order (e.g., 2 for acceleration, 1 for velocity)
  int errorOrder = 2;
  aslam::backend::addMotionErrorTerms<
      aslam::splines::BSplinePoseDesignVariable>(problem, *poseDv_, W,
                                                 errorOrder);
}

void IccCalibrator::registerCamChain(std::shared_ptr<IccCameraChain> sensor) {
  cameraChain_ = sensor;
}

void IccCalibrator::registerImu(std::shared_ptr<IccImu> sensor) {
  imuList_.push_back(sensor);
}

void IccCalibrator::buildProblem(
    int splineOrder, int poseKnotsPerSecond, int biasKnotsPerSecond,
    bool doPoseMotionError, double mrTranslationVariance,
    double mrRotationVariance, bool doBiasMotionError, int blakeZisserCam,
    double huberAccel, double huberGyro, bool noTimeCalibration,
    bool noChainExtrinsics, int maxIterations, double gyroNoiseScale,
    double accelNoiseScale, double timeOffsetPadding, bool verbose) {
  std::cout << "\tSpline order: " << splineOrder << std::endl;
  std::cout << "\tPose knots per second: " << poseKnotsPerSecond << std::endl;
  std::cout << "\tDo pose motion regularization: "
            << (doPoseMotionError ? "True" : "False") << std::endl;
  std::cout << "\t\txddot translation variance: " << mrTranslationVariance
            << std::endl;
  std::cout << "\t\txddot rotation variance: " << mrRotationVariance
            << std::endl;
  std::cout << "\tBias knots per second: " << biasKnotsPerSecond << std::endl;
  std::cout << "\tDo bias motion regularization: "
            << (doBiasMotionError ? "True" : "False") << std::endl;
  std::cout << "\tBlake-Zisserman on reprojection errors " << blakeZisserCam
            << std::endl;
  std::cout << "\tAcceleration Huber width (sigma): " << huberAccel
            << std::endl;
  std::cout << "\tGyroscope Huber width (sigma): " << huberGyro << std::endl;
  std::cout << "\tDo time calibration: " << !noTimeCalibration << std::endl;
  std::cout << "\tMax iterations: " << maxIterations << std::endl;
  std::cout << "\tTime offset padding: " << timeOffsetPadding << std::endl;

  noTimeCalibration_ = noTimeCalibration;
  if (!noTimeCalibration) {
    for (auto cam : cameraChain_->getCamList()) {
      cam->findTimeshiftCameraImuPrior(*imuList_[0], verbose);
    }
  }

  cameraChain_->findOrientationPriorCameraChainToImu(*imuList_[0]);
  Eigen::Vector3d estimatedGravity = cameraChain_->getEstimatedGravity();

  bsplines::BSplinePose poseSpline =
      cameraChain_->initializePoseSplineFromCameraChain(
          splineOrder, poseKnotsPerSecond, timeOffsetPadding);

  for (auto& imu : imuList_) {
    imu->initBiasSplines(poseSpline, splineOrder, biasKnotsPerSecond);
  }

  auto problem = std::make_shared<aslam::calibration::OptimizationProblem>();

  initDesignVariables(*problem, poseSpline, noTimeCalibration,
                      noChainExtrinsics, false, estimatedGravity);

  cameraChain_->addCameraChainerrorTerms(*problem, poseDv_, blakeZisserCam,
                                         timeOffsetPadding);

  for (auto& imu : imuList_) {
    imu->addAccelerometerErrorTerms(*problem, *poseDv_, *gravityExpression_,
                                    huberAccel, accelNoiseScale);
    imu->addGyroscopeErrorTerms(*problem, *poseDv_, huberGyro, gyroNoiseScale,
                                *gravityExpression_);

    if (doBiasMotionError) {
      imu->addBiasMotionTerms(*problem);
    }
  }

  if (doPoseMotionError) {
    addPoseMotionTerms(*problem, mrTranslationVariance, mrRotationVariance);
  }
  problem_ = std::move(problem);
}

void IccCalibrator::optimize(
    std::shared_ptr<aslam::backend::Optimizer2Options> options,
    int maxIterations, bool recoverCov) {
  if (!options) {
    options = std::make_shared<aslam::backend::Optimizer2Options>();
    options->verbose = true;
    options->nThreads = std::max(1u, std::thread::hardware_concurrency() - 1);
    options->convergenceDeltaX=1e-5;
    options->convergenceDeltaJ=1e-2;
    options->maxIterations=maxIterations;
    options->trustRegionPolicy = std::make_shared<aslam::backend::LevenbergMarquardtTrustRegionPolicy>(10.0);
    options->linearSystemSolver = std::make_shared<aslam::backend::BlockCholeskyLinearSystemSolver>();
  }

  optimizer_ = std::make_shared<aslam::backend::Optimizer2>(*options);
  optimizer_->setProblem(problem_);

  auto optimizationFailed = false;

  try {
    auto retval = optimizer_->optimize();
    if (retval.linearSolverFailure){
        optimizationFailed = true;
    }
  } catch (std::exception e) {
    std::cerr << e.what() <<std::endl;
    optimizationFailed = true;
  }

  if (optimizationFailed) {
    std::cerr << "Optimization failed!" << std::endl;
    throw std::runtime_error("Optimization failed!");
  }

  optimizer_.reset();
  if (recoverCov) {
    recoverCovariance();
  }
}


void IccCalibrator::recoverCovariance() {
  std::cout << "Recovering covariance..." << std::endl;
  auto estimator = aslam::calibration::IncrementalEstimator(CALIBRATION_GROUP_ID);
  auto rval = estimator.addBatch(problem_,true);
  Eigen::VectorXd est_stds = estimator.getSigma2Theta().diagonal().cwiseSqrt();

  std_trafo_ic_ = est_stds.head(6);
  std_times_ = est_stds.tail(est_stds.size() - 6);
}

void IccCalibrator::saveImuSetParametersYaml(const std::string& resultFile) {
  auto imuSetConfig =
      std::make_shared<kalibr::ImuSetParameters>(resultFile, true);
  for (auto imu : imuList_) {
    auto imuConfig = imu->getImuConfig();
    imuSetConfig->addImuConfig(imuConfig);
  }
  imuSetConfig->writeYaml(resultFile);
}

void IccCalibrator::saveCamChainParametersYaml(const std::string& resultFile)
{
  auto chain = cameraChain_->getChainConfig();
  auto nCams = cameraChain_->getCamList().size();

  for (size_t camNr=0;camNr < nCams;camNr++){
    if (camNr>0){
        auto [T_cB_cA,baseline] = cameraChain_->getResultBaseline(camNr-1, camNr);
        chain.setExtrinsicsLastCamToHere(camNr, T_cB_cA);
    }

    auto T_ci = cameraChain_->getResultTrafoImuToCam(camNr);
    chain.setExtrinsicsImuToCam(camNr, T_ci);

    if (!noTimeCalibration_){
        auto timeshift = cameraChain_->getResultTimeshift(camNr);
        chain.setTimeshiftCamImu(camNr, timeshift);
    }
  }

  try {
    chain.writeYaml(resultFile);
  } catch (...){
    throw std::runtime_error("ERROR: Could not write parameters to file: "+resultFile+"\n");
  }
}

}  // namespace kalibr
