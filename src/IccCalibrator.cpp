#include <Eigen/Core>
#include <IccCalibrator.hpp>
#include <IccSensors.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/EuclideanDirection.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/backend/BSplineMotionErrorFactory.hpp>
#include <iostream>
#include <memory>

#include "aslam/backend/DesignVariable.hpp"

namespace kalibr {

// IccCalibrator::IccCalibrator()
//     : problem_(std::make_shared<aslam::backend::OptimizationProblem>()) {}

// IccCalibrator::~IccCalibrator() = default;

// void IccCalibrator::initDesignVariables(
//     aslam::backend::OptimizationProblem& problem,
//     const bsplines::BSplinePose& poseSpline, bool noTimeCalibration,
//     bool noChainExtrinsics, bool estimateGravityLength,
//     const Eigen::Vector3d& initialGravityEstimate) {
//   // Initialize the system pose spline (always attached to imu0)
//   poseDv_ =
//       std::make_shared<aslam::splines::BSplinePoseDesignVariable>(poseSpline);
//   addSplineDesignVariables(problem, *poseDv_);

//   // Add the calibration target orientation design variable
//   // (expressed as gravity vector in target frame)
//   if (estimateGravityLength) {
//     gravityDv_ = std::make_shared<aslam::backend::EuclideanPoint>(
//         initialGravityEstimate);
//     gravityExpression_ =
//         std::make_shared<aslam::backend::EuclideanExpression>(
//             ((aslam::backend::EuclideanPoint*)gravityDv_.get())->toExpression());
//   } else {
//     gravityDv_ = std::make_shared<aslam::backend::EuclideanDirection>(initialGravityEstimate);
//     gravityExpression_ = std::make_shared<aslam::backend::EuclideanExpression>(
//         ((aslam::backend::EuclideanDirection*)gravityDv_.get())->toExpression());
//   }

//   // Provide appropriate arguments to toExpression, e.g., time = 0.0 and derivative order = 0
  
//   gravityDv_->setActive(true);
//   problem.addDesignVariable((aslam::backend::DesignVariable*)gravityDv_.get(), HELPER_GROUP_ID);

//   // Add all DVs for all IMUs
//   for (auto& imu : imuList_) {
//     imu->addDesignVariables(problem, poseSpline);
//   }

//   // Add all DVs for the camera chain
//   if (cameraChain_) {
//     cameraChain_->addDesignVariables(problem, noTimeCalibration,
//                                      noChainExtrinsics);
//   }
// }

// void IccCalibrator::addPoseMotionTerms(
//     aslam::backend::OptimizationProblem& problem, double tv, double rv) {
//   double wt = 1.0 / tv;
//   double wr = 1.0 / rv;

//   Eigen::Matrix<double, 6, 6> W;
//   W.setZero();
//   W.diagonal() << wt, wt, wt, wr, wr, wr;

//   // Add motion error terms
//   // Define the error order (e.g., 2 for acceleration, 1 for velocity)
//   int errorOrder = 2;
//   aslam::backend::addMotionErrorTerms<aslam::splines::BSplinePoseDesignVariable>(
//       problem, *poseDv_, W, errorOrder);
// }

// void IccCalibrator::registerCamChain(std::shared_ptr<IccCameraChain> sensor) {
//   cameraChain_ = sensor;
// }

// void IccCalibrator::registerImu(std::shared_ptr<IccImu> sensor) {
//   imuList_.push_back(sensor);
// }

// void IccCalibrator::buildProblem(
//     int splineOrder, int poseKnotsPerSecond, int biasKnotsPerSecond,
//     bool doPoseMotionError, double mrTranslationVariance,
//     double mrRotationVariance, bool doBiasMotionError, int blakeZisserCam,
//     double huberAccel, double huberGyro, bool noTimeCalibration,
//     bool noChainExtrinsics, int maxIterations, double gyroNoiseScale,
//     double accelNoiseScale, double timeOffsetPadding, bool verbose) {
//   std::cout << "\tSpline order: " << splineOrder << std::endl;
//   std::cout << "\tPose knots per second: " << poseKnotsPerSecond << std::endl;
//   std::cout << "\tDo pose motion regularization: "
//             << (doPoseMotionError ? "True" : "False") << std::endl;
//   std::cout << "\t\txddot translation variance: " << mrTranslationVariance
//             << std::endl;
//   std::cout << "\t\txddot rotation variance: " << mrRotationVariance
//             << std::endl;
//   std::cout << "\tBias knots per second: " << biasKnotsPerSecond << std::endl;
//   std::cout << "\tDo bias motion regularization: "
//             << (doBiasMotionError ? "True" : "False") << std::endl;
//   std::cout << "\tBlake-Zisserman on reprojection errors " << blakeZisserCam
//             << std::endl;
//   std::cout << "\tAcceleration Huber width (sigma): " << huberAccel
//             << std::endl;
//   std::cout << "\tGyroscope Huber width (sigma): " << huberGyro << std::endl;

//   // Implementation would continue here with:
//   // 1. Initialize pose spline from camera
//   // 2. Initialize bias splines for IMUs
//   // 3. Add error terms for cameras and IMUs
//   // 4. Add motion error terms if enabled
//   // This requires more detailed implementation based on the full Python code
// }

// void IccCalibrator::optimize(const aslam::backend::Optimizer2Options* options,
//                              int maxIterations, bool recoverCov) {
//   if (!problem_) {
//     throw std::runtime_error("Problem not initialized");
//   }

//   // Create optimizer
//   aslam::backend::Optimizer2Options defaultOptions;
//   if (!options) {
//     options = &defaultOptions;
//   }

//   optimizer_ = std::make_shared<aslam::backend::Optimizer2>(*options, problem_);

//   // Run optimization
//   optimizer_->optimize();

//   if (recoverCov) {
//     recoverCovariance();
//   }
// }

// void IccCalibrator::recoverCovariance() {
//   if (!optimizer_) {
//     throw std::runtime_error("Optimizer not initialized");
//   }

//   // Compute covariance
//   // optimizer_->computeCovariances();

//   std::cout << "Covariance recovery completed" << std::endl;
// }

// void IccCalibrator::saveImuSetParametersYaml(const std::string& resultFile) {
//   // Save IMU parameters to YAML file
//   std::cout << "Saving IMU parameters to: " << resultFile << std::endl;

//   // Implementation would write IMU calibration results to YAML
// }

// void IccCalibrator::saveCamChainParametersYaml(const std::string& resultFile) {
//   // Save camera chain parameters to YAML file
//   std::cout << "Saving camera chain parameters to: " << resultFile << std::endl;

//   // Implementation would write camera calibration results to YAML
// }

}  // namespace kalibr
