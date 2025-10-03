#ifndef ICC_SENSORS_HPP
#define ICC_SENSORS_HPP

#include <Eigen/Core>
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <aslam/splines/EuclideanBSplineDesignVariable.hpp>
#include <bsplines/BSplinePose.hpp>
#include <kalibr_backend/MatrixDesignVariable.hpp>
#include <kalibr_backend/ScalarDesignVariable.hpp>
#include <kalibr_backend/TransformationDesignVariable.hpp>
#include <kalibr_common/ConfigReader.hpp>
#include <kalibr_common/ImageDatasetReader.hpp>
#include <kalibr_common/ImuDatasetReader.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace kalibr {

/**
 * @brief Single camera for calibration
 */
class IccCamera {
 public:
  IccCamera(const CameraParameters& camConfig,
            const CalibrationTargetParameters& targetConfig,
            const ImageDatasetReader& dataset, double reprojectionSigma = 1.0,
            bool showCorners = true, bool showReproj = true,
            bool showOneStep = false);

  /**
   * @brief Setup calibration target
   */
  void setupCalibrationTarget(const CalibrationTargetParameters& targetConfig,
                              bool showExtraction = false,
                              bool showReproj = false,
                              bool imageStepping = false);

  /**
   * @brief Find orientation prior from camera to IMU
   */
  void findOrientationPriorCameraToImu(const class IccImu& imu);

  /**
   * @brief Get estimated gravity vector
   */
  Eigen::Vector3d getEstimatedGravity() const;

  /**
   * @brief Find time shift between camera and IMU using cross-correlation
   */
  double findTimeshiftCameraImuPrior(const class IccImu& imu,
                                     bool verbose = false);

  /**
   * @brief Initialize pose spline from camera poses
   */
  bsplines::BSplinePose initPoseSplineFromCamera(
      int splineOrder = 6, int poseKnotsPerSecond = 100,
      double timeOffsetPadding = 0.02);

  /**
   * @brief Add design variables to optimization problem
   */
  void addDesignVariables(aslam::backend::OptimizationProblem& problem,
                          bool noTimeCalibration = true,
                          bool estimateExtrinsics = false);

  /**
   * @brief Add camera reprojection error terms
   */
  void addCameraErrorTerms(
      aslam::backend::OptimizationProblem& problem,
      const std::shared_ptr<aslam::splines::BSplinePoseDesignVariable>& poseDv,
      bool useBlakeZissermanMest = false);

  // Getters
  const std::vector<aslam::cameras::GridCalibrationTargetObservation>&
  getObservations() const {
    return observations_;
  }
  const ImageDatasetReader& getDataset() const { return dataset_; }
  std::shared_ptr<AslamCamera> getCamera() const { return camera_; }
  std::shared_ptr<aslam::cameras::GridDetector> getDetector() const {
    return detector_;
  }
  sm::kinematics::Transformation getExtrinsic() const { return T_extrinsic_; }
  double getTimeshiftPrior() const { return timeshiftCamToImuPrior_; }

 private:
  ImageDatasetReader dataset_;
  std::shared_ptr<AslamCamera> camera_;
  std::shared_ptr<aslam::cameras::GridDetector> detector_;
  std::vector<aslam::cameras::GridCalibrationTargetObservation> observations_;

  // Configuration
  double reprojectionSigma_;
  double cornerUncertainty_;
  bool showCorners_;
  bool showReproj_;
  bool showOneStep_;

  // Calibration parameters
  sm::kinematics::Transformation T_extrinsic_;
  double timeshiftCamToImuPrior_;
  Eigen::Vector3d gravity_w_;

  // Design variables (set by addDesignVariables)
  std::shared_ptr<aslam::backend::TransformationDesignVariable> dv_T_c_b_;
  std::shared_ptr<aslam::backend::ScalarDesignVariable> dv_time_offset_;

  // Reprojection errors (set by addCameraErrorTerms)
  std::vector<std::vector<std::shared_ptr<aslam::backend::ErrorTerm>>>
      allReprojectionErrors_;
};

/**
 * @brief Camera chain (multiple cameras)
 */
class IccCameraChain {
 public:
  IccCameraChain(const std::vector<CameraParameters>& camConfigs,
                 const CalibrationTargetParameters& targetConfig,
                 const std::vector<ImageDatasetReader>& datasets);

  /**
   * @brief Add design variables to optimization problem
   */
  void addDesignVariables(aslam::backend::OptimizationProblem& problem,
                          bool noTimeCalibration, bool noChainExtrinsics);

  /**
   * @brief Initialize from camera chain
   */
  void initFromCameraChain();

  // Getters
  const std::vector<std::shared_ptr<IccCamera>>& getCamList() const {
    return camList_;
  }
  std::vector<std::shared_ptr<IccCamera>>& getCamList() { return camList_; }
  size_t numCameras() const { return camList_.size(); }

 private:
  std::vector<std::shared_ptr<IccCamera>> camList_;
  std::vector<std::shared_ptr<aslam::backend::TransformationDesignVariable>>
      transformations_;
  std::vector<std::shared_ptr<aslam::backend::ScalarDesignVariable>>
      timeOffsets_;
};

/**
 * @brief IMU sensor for calibration
 */
class IccImu {
 public:
  IccImu(const ImuParameters& imuConfig, const ImuDatasetReader& dataset);

  virtual ~IccImu() = default;

  /**
   * @brief Add design variables to optimization problem
   */
  virtual void addDesignVariables(aslam::backend::OptimizationProblem& problem,
                                  const bsplines::BSplinePose& poseSpline);

  /**
   * @brief Add error terms to optimization problem
   */
  virtual void addErrorTerms(
      aslam::backend::OptimizationProblem& problem,
      const aslam::splines::BSplinePoseDesignVariable& poseDv,
      const aslam::backend::EuclideanExpression& gravityExpression,
      double gyroNoiseScale = 1.0, double accelNoiseScale = 1.0,
      double huberGyro = -1.0, double huberAccel = -1.0);

  /**
   * @brief Initialize bias splines
   */
  void initBiasSplines(const bsplines::BSplinePose& poseSpline,
                       int biasKnotsPerSecond, int splineOrder);

  // Getters
  const ImuDatasetReader& getDataset() const { return dataset_; }
  ImuDatasetReader& getDataset() { return dataset_; }
  const std::vector<ImuMeasurement>& getImuData() const { return imuData_; }
  double getTimeOffset() const { return timeOffset_; }
  void setTimeOffset(double offset) { timeOffset_ = offset; }

  std::shared_ptr<aslam::backend::TransformationDesignVariable>
  getTransformationDv() const {
    return T_i_b_Dv_;
  }

 protected:
  ImuDatasetReader dataset_;
  std::vector<ImuMeasurement> imuData_;
  double timeOffset_;

  // Design variables
  std::shared_ptr<aslam::backend::TransformationDesignVariable> T_i_b_Dv_;
  std::shared_ptr<aslam::backend::ScalarDesignVariable> timeOffsetDv_;

  // Bias splines (3D for gyro and accel)
  std::shared_ptr<aslam::splines::BSplineDesignVariable<3>> gyroBiasSpline_;
  std::shared_ptr<aslam::splines::BSplineDesignVariable<3>> accelBiasSpline_;

  // Error terms
  std::vector<std::shared_ptr<aslam::backend::ErrorTerm>> gyroErrors_;
  std::vector<std::shared_ptr<aslam::backend::ErrorTerm>> accelErrors_;
};

/**
 * @brief IMU with scale and misalignment
 */
class IccScaledMisalignedImu : public IccImu {
 public:
  IccScaledMisalignedImu(const ImuParameters& imuConfig,
                         const ImuDatasetReader& dataset);

  void addDesignVariables(aslam::backend::OptimizationProblem& problem,
                          const bsplines::BSplinePose& poseSpline) override;

  void addErrorTerms(
      aslam::backend::OptimizationProblem& problem,
      const aslam::splines::BSplinePoseDesignVariable& poseDv,
      const aslam::backend::EuclideanExpression& gravityExpression,
      double gyroNoiseScale = 1.0, double accelNoiseScale = 1.0,
      double huberGyro = -1.0, double huberAccel = -1.0) override;

 protected:
  // Scale and misalignment matrices
  std::shared_ptr<aslam::backend::MatrixDesignVariable> M_gyro_;
  std::shared_ptr<aslam::backend::MatrixDesignVariable> M_accel_;
};

/**
 * @brief IMU with scale, misalignment, and size effect
 */
class IccScaledMisalignedSizeEffectImu : public IccScaledMisalignedImu {
 public:
  IccScaledMisalignedSizeEffectImu(const ImuParameters& imuConfig,
                                   const ImuDatasetReader& dataset);

  void addDesignVariables(aslam::backend::OptimizationProblem& problem,
                          const bsplines::BSplinePose& poseSpline) override;

  void addErrorTerms(
      aslam::backend::OptimizationProblem& problem,
      const aslam::splines::BSplinePoseDesignVariable& poseDv,
      const aslam::backend::EuclideanExpression& gravityExpression,
      double gyroNoiseScale = 1.0, double accelNoiseScale = 1.0,
      double huberGyro = -1.0, double huberAccel = -1.0) override;

 private:
  // Size effect parameters (eccentric mounting)
  std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> r_x_;
  std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> r_y_;
  std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> r_z_;
};

}  // namespace kalibr

#endif  // ICC_SENSORS_HPP
