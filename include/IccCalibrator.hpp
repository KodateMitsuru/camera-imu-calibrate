#ifndef ICCCALIBRATOR_HPP
#define ICCCALIBRATOR_HPP

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

// Include actual headers instead of forward declarations
#include <aslam/calibration/core/IncrementalEstimator.h>
#include <aslam/calibration/core/OptimizationProblem.h>

#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/EuclideanDirection.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/LinearSystemSolver.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/TrustRegionPolicy.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <sm/logging.hpp>

// Forward declarations for local types
class ImuType;
class CameraChainType;

namespace kalibr_common {
class ImuSetParameters;
}

// Forward declarations for local types
class ImuType;
class CameraChainType;

// Constants
constexpr int CALIBRATION_GROUP_ID = 0;
constexpr int HELPER_GROUP_ID = 1;

// Utility function
void addSplineDesignVariables(
    aslam::calibration::OptimizationProblem* problem,
    aslam::splines::BSplinePoseDesignVariable* dvc, bool setActive = true,
    int group_id = HELPER_GROUP_ID);

class IccCalibrator {
 public:
  IccCalibrator();
  ~IccCalibrator();

  // Core methods
  void initDesignVariables(
      aslam::calibration::OptimizationProblem* problem,
      bsplines::BSpline* poseSpline, bool noTimeCalibration,
      bool noChainExtrinsics = true, bool estimateGravityLength = false,
      const Eigen::Vector3d& initialGravityEstimate = Eigen::Vector3d(0.0, 9.81,
                                                                      0.0));

  void addPoseMotionTerms(
      aslam::calibration::OptimizationProblem* problem,
      double tv, double rv);

  void registerCamChain(CameraChainType* sensor);
  void registerImu(ImuType* sensor);

  void buildProblem(int splineOrder = 6, int poseKnotsPerSecond = 70,
                    int biasKnotsPerSecond = 70, bool doPoseMotionError = false,
                    double mrTranslationVariance = 1e6,
                    double mrRotationVariance = 1e5,
                    bool doBiasMotionError = true, int blakeZisserCam = -1,
                    double huberAccel = -1, double huberGyro = -1,
                    bool noTimeCalibration = false,
                    bool noChainExtrinsics = true, int maxIterations = 20,
                    double gyroNoiseScale = 1.0, double accelNoiseScale = 1.0,
                    double timeOffsetPadding = 0.02, bool verbose = false);

  void optimize(aslam::backend::Optimizer2Options* options = nullptr,
                int maxIterations = 30, bool recoverCov = false);

  void recoverCovariance();
  void saveImuSetParametersYaml(const std::string& resultFile);
  void saveCamChainParametersYaml(const std::string& resultFile);

 private:
  // Member variables
  std::vector<ImuType*> ImuList;
  CameraChainType* CameraChain = nullptr;
  aslam::splines::BSplinePoseDesignVariable* poseDv = nullptr;
  aslam::backend::DesignVariable* gravityDv = nullptr;
  aslam::backend::DesignVariable* gravityExpression = nullptr;
  aslam::calibration::OptimizationProblem* problem =
      nullptr;
  aslam::backend::Optimizer2* optimizer = nullptr;
  bool noTimeCalibration = false;

  // Covariance results
  Eigen::VectorXd std_trafo_ic;
  Eigen::VectorXd std_times;
};

#endif  // ICCCALIBRATOR_HPP