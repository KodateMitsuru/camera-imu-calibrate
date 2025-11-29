#ifndef ICC_CALIBRATOR_HPP
#define ICC_CALIBRATOR_HPP

#include <aslam/calibration/core/OptimizationProblem.h>

#include <Eigen/Core>
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/EuclideanDirection.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/splines/EuclideanBSplineDesignVariable.hpp>
#include <bsplines/BSplinePose.hpp>
#include <memory>
#include <string>
#include <vector>

#include "IccSensors.hpp"

namespace kalibr {

/**
 * @brief Main calibrator class for IMU-Camera calibration
 */
class IccCalibrator {
 public:
  IccCalibrator();
  ~IccCalibrator();

  /**
   * @brief Initialize design variables for optimization
   */
  void initDesignVariables(
      aslam::calibration::OptimizationProblem& problem,
      const bsplines::BSplinePose& poseSpline, bool noTimeCalibration,
      bool noChainExtrinsics = true, bool estimateGravityLength = false,
      const Eigen::Vector3d& initialGravityEstimate = Eigen::Vector3d(0.0, 9.81,
                                                                      0.0));

  /**
   * @brief Add pose motion error terms to the problem
   */
  void addPoseMotionTerms(aslam::calibration::OptimizationProblem& problem,
                          double tv, double rv);

  /**
   * @brief Register camera chain
   */
  void registerCamChain(std::shared_ptr<IccCameraChain> sensor);

  /**
   * @brief Register IMU
   */
  void registerImu(std::shared_ptr<IccImu> sensor);

  /**
   * @brief Build optimization problem
   */
  void buildProblem(int splineOrder = 6, int poseKnotsPerSecond = 70,
                    int biasKnotsPerSecond = 70, bool doPoseMotionError = false,
                    double mrTranslationVariance = 1e6,
                    double mrRotationVariance = 1e5,
                    bool doBiasMotionError = true, int blakeZisserCam = -1,
                    double huberAccel = -1.0, double huberGyro = -1.0,
                    bool noTimeCalibration = false,
                    bool noChainExtrinsics = true, int maxIterations = 20,
                    double gyroNoiseScale = 1.0, double accelNoiseScale = 1.0,
                    double timeOffsetPadding = 0.02, bool verbose = false);

  /**
   * @brief Optimize the calibration
   */
  void optimize(
      std::shared_ptr<aslam::backend::Optimizer2Options> options = nullptr,
      int maxIterations = 30, bool recoverCov = false);

  /**
   * @brief Recover covariance matrix
   */
  void recoverCovariance();

  /**
   * @brief Save IMU parameters to YAML
   */
  void saveImuSetParametersYaml(const std::string& resultFile);

  /**
   * @brief Save camera chain parameters to YAML
   */
  void saveCamChainParametersYaml(const std::string& resultFile);

  // Getters
  aslam::splines::BSplinePoseDesignVariable* getPoseDv() const {
    return poseDv_;
  }
  // 保存gravityDv_的实际类型信息
  std::size_t getGravityDvType() const { return gravityDvType_; }

  std::shared_ptr<aslam::backend::DesignVariable> getGravityDv() const {
    return gravityDv_;
  }

  std::shared_ptr<aslam::backend::EuclideanExpression> getGravityExpression()
      const {
    return gravityExpression_;
  }
  const std::vector<std::shared_ptr<IccImu>>& getImuList() const {
    return imuList_;
  }
  std::shared_ptr<IccCameraChain> getCameraChain() const {
    return cameraChain_;
  }

  const Eigen::VectorXd& getStdTrafoIc() const { return std_trafo_ic_; }

  const Eigen::VectorXd& getStdTimes() const { return std_times_; }

  bool noTimeCalibration() const { return noTimeCalibration_; }

 private:
  std::vector<std::shared_ptr<IccImu>> imuList_;
  std::shared_ptr<IccCameraChain> cameraChain_;
  aslam::splines::BSplinePoseDesignVariable* poseDv_;
  std::shared_ptr<aslam::backend::DesignVariable> gravityDv_;
  std::size_t gravityDvType_;
  std::shared_ptr<aslam::backend::EuclideanExpression> gravityExpression_;
  std::shared_ptr<aslam::calibration::OptimizationProblem> problem_;
  std::shared_ptr<aslam::backend::Optimizer2> optimizer_;
  Eigen::VectorXd std_trafo_ic_;
  Eigen::VectorXd std_times_;
  bool noTimeCalibration_;
};

}  // namespace kalibr

#endif  // ICC_CALIBRATOR_HPP
