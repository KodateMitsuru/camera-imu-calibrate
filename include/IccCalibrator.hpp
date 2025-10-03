#ifndef ICC_CALIBRATOR_HPP
#define ICC_CALIBRATOR_HPP

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

namespace kalibr {

// Forward declarations
class IccImu;
class IccCameraChain;

constexpr int CALIBRATION_GROUP_ID = 0;
constexpr int HELPER_GROUP_ID = 1;

/**
 * @brief Add spline design variables to the optimization problem
 */
void addSplineDesignVariables(aslam::backend::OptimizationProblem& problem,
                              aslam::splines::BSplinePoseDesignVariable& dvc,
                              bool setActive = true,
                              int group_id = HELPER_GROUP_ID);

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
      aslam::backend::OptimizationProblem& problem,
      const bsplines::BSplinePose& poseSpline, bool noTimeCalibration,
      bool noChainExtrinsics = true, bool estimateGravityLength = false,
      const Eigen::Vector3d& initialGravityEstimate = Eigen::Vector3d(0.0, 9.81,
                                                                      0.0));

  /**
   * @brief Add pose motion error terms to the problem
   */
  void addPoseMotionTerms(aslam::backend::OptimizationProblem& problem,
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
  void optimize(const aslam::backend::Optimizer2Options* options = nullptr,
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
  std::shared_ptr<aslam::splines::BSplinePoseDesignVariable> getPoseDv() const {
    return poseDv_;
  }
  std::shared_ptr<aslam::backend::DesignVariable> getGravityDv()
      const {
    return gravityDv_;
  }
  std::shared_ptr<aslam::backend::EuclideanExpression> getGravityExpression() const {
    return gravityExpression_;
  }
  const std::vector<std::shared_ptr<IccImu>>& getImuList() const {
    return imuList_;
  }
  std::shared_ptr<IccCameraChain> getCameraChain() const {
    return cameraChain_;
  }

 private:
  std::vector<std::shared_ptr<IccImu>> imuList_;
  std::shared_ptr<IccCameraChain> cameraChain_;
  std::shared_ptr<aslam::splines::BSplinePoseDesignVariable> poseDv_;
  std::shared_ptr<aslam::backend::DesignVariable> gravityDv_;
  std::shared_ptr<aslam::backend::EuclideanExpression> gravityExpression_;
  std::shared_ptr<aslam::backend::OptimizationProblem> problem_;
  std::shared_ptr<aslam::backend::Optimizer2> optimizer_;
};

}  // namespace kalibr

#endif  // ICC_CALIBRATOR_HPP
