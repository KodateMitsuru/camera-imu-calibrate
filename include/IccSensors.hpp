#ifndef ICC_SENSORS_HPP
#define ICC_SENSORS_HPP

#include <Eigen/Core>
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/MatrixBasic.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Scalar.hpp>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <aslam/splines/EuclideanBSplineDesignVariable.hpp>
#include <bsplines/BSplinePose.hpp>
#include <cstddef>
#include <fstream>
#include <kalibr_backend/TransformationDesignVariable.hpp>
#include <kalibr_common/ConfigReader.hpp>
#include <kalibr_common/ImageDatasetReader.hpp>
#include <kalibr_common/ImuDatasetReader.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <ranges>
#include <string>
#include <variant>
#include <vector>

#include "aslam/backend/EuclideanPoint.hpp"
#include "aslam/backend/RotationQuaternion.hpp"
#include "kalibr_errorterms/AccelerometerError.hpp"
#include "kalibr_errorterms/EuclideanError.hpp"
#include "kalibr_errorterms/GyroscopeError.hpp"
#include "sm/kinematics/Transformation.hpp"


namespace kalibr {

constexpr int CALIBRATION_GROUP_ID = 0;
constexpr int HELPER_GROUP_ID = 1;

/**
 * @brief Add spline design variables to the optimization problem
 */
template <typename SplineDesignVariableType>
concept HasSplineDesignVariableMethods =
    requires(SplineDesignVariableType dvc, size_t i) {
      { dvc.numDesignVariables() } -> std::convertible_to<size_t>;
      {
        dvc.designVariable(i)
      } -> std::convertible_to<aslam::backend::DesignVariable*>;
    };
template <typename OptimizationProblemType>
concept HasOptimizationProblemMethods =
    requires(OptimizationProblemType problem,
             aslam::backend::DesignVariable* dv, int group_id) {
      { problem.addDesignVariable(dv, group_id) };
    } ||
    requires(OptimizationProblemType problem,
             std::shared_ptr<aslam::backend::DesignVariable> dv_sp,
             int group_id) {
      { problem.addDesignVariable(dv_sp, group_id) };
    };

template <typename ProblemType, typename SplineDesignVariableType>
  requires HasOptimizationProblemMethods<ProblemType> &&
           HasSplineDesignVariableMethods<SplineDesignVariableType>
void addSplineDesignVariables(ProblemType& problem,
                              SplineDesignVariableType& dvc,
                              bool setActive = true,
                              int group_id = HELPER_GROUP_ID) {
  for (size_t i = 0; i < dvc.numDesignVariables(); ++i) {
    auto dv = dvc.designVariable(i);
    dv->setActive(setActive);
    if constexpr (requires { problem.addDesignVariable(dv, group_id); }) {
      problem.addDesignVariable(dv, group_id);
    } else if constexpr (requires {
                           problem.addDesignVariable(dv.get(), group_id);
                         }) {
      problem.addDesignVariable(dv.get(), group_id);
    } else if constexpr (requires {
                           problem.addDesignVariable(
                               std::shared_ptr<aslam::backend::DesignVariable>(
                                   dv),
                               group_id);
                         }) {
      problem.addDesignVariable(
          std::shared_ptr<aslam::backend::DesignVariable>(dv), group_id);
    } else {
      static_assert(
          []() { return false; }(),
          "No suitable addDesignVariable method found in ProblemType");
    }
  }
}

/**
 * @brief Single camera for calibration
 */
class IccCamera {
 public:
  IccCamera(const CameraParameters& camConfig,
            const CalibrationTargetParameters& targetConfig,
            const ImageDatasetReader& dataset, double reprojectionSigma = 1.0,
            bool showCorners = false, bool showReproj = false,
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
  void findOrientationPriorCameraToImu(class IccImu& imu);

  /**
   * @brief Get estimated gravity vector
   */
  Eigen::Vector3d getEstimatedGravity() const;

  /**
   * @brief Find time shift between camera and IMU using cross-correlation
   */
  double findTimeshiftCameraImuPrior(class IccImu& imu, bool verbose = false);

  /**
   * @brief Initialize pose spline from camera poses
   */
  bsplines::BSplinePose initPoseSplineFromCamera(
      int splineOrder = 6, int poseKnotsPerSecond = 100,
      double timeOffsetPadding = 0.02);

  /**
   * @brief Add design variables to optimization problem
   */
  template <typename ProblemType>
    requires HasOptimizationProblemMethods<ProblemType>
  void addDesignVariables(ProblemType& problem, bool noExtrinsics = true,
                          bool noTimeCalibration = true,
                          size_t baselinedv_group_id = HELPER_GROUP_ID) {
    // Add transformation design variables
    auto active = !noExtrinsics;
    this->T_c_b_Dv_ =
        std::make_shared<aslam::backend::TransformationDesignVariable>(
            this->T_extrinsic_, active, active);
    for (int i = 0; i < this->T_c_b_Dv_->numDesignVariables(); ++i) {
      if constexpr (requires {
                      problem.addDesignVariable(
                          this->T_c_b_Dv_->getDesignVariable(i),
                          baselinedv_group_id);
                    }) {
        problem.addDesignVariable(this->T_c_b_Dv_->getDesignVariable(i),
                                  baselinedv_group_id);
      } else {
        problem.addDesignVariable(this->T_c_b_Dv_->getDesignVariable(i).get(),
                                  baselinedv_group_id);
      }
    }

    // Add the time delay design variable.
    this->cameraTimeToImuTimeDv_ =
        std::make_shared<aslam::backend::Scalar>(0.0);
    this->cameraTimeToImuTimeDv_->setActive(!noTimeCalibration);
    if constexpr (requires {
                    problem.addDesignVariable(this->cameraTimeToImuTimeDv_,
                                              CALIBRATION_GROUP_ID);
                  }) {
      problem.addDesignVariable(this->cameraTimeToImuTimeDv_,
                                CALIBRATION_GROUP_ID);
    } else {
      problem.addDesignVariable(this->cameraTimeToImuTimeDv_.get(),
                                CALIBRATION_GROUP_ID);
    }
  }

  /**
   * @brief Add camera reprojection error terms
   * @param problem Optimization problem
   * @param poseSplineDv Pose spline design variable
   * @param T_cN_b Transformation from IMU to camera N
   * @param blakeZissermanDf Blake-Zisserman M-estimator degrees of freedom (0.0
   * = disabled)
   * @param timeOffsetPadding Time offset padding for spline evaluation
   */
  void addCameraErrorTerms(
      aslam::backend::OptimizationProblemBase& problem,
      const std::shared_ptr<aslam::splines::BSplinePoseDesignVariable>&
          poseSplineDv,
      const aslam::backend::TransformationExpression& T_cN_b,
      double blakeZissermanDf = 0.0, double timeOffsetPadding = 0.0);

  // Getters
  const std::vector<aslam::cameras::GridCalibrationTargetObservation>&
  getObservations() const {
    return targetObservations_;
  }
  const ImageDatasetReader& getDataset() const { return dataset_; }
  std::shared_ptr<AslamCamera> getCamera() const { return camera_; }
  std::shared_ptr<aslam::cameras::GridDetector> getDetector() const {
    return detector_;
  }
  sm::kinematics::Transformation& getExtrinsic() { return T_extrinsic_; }
  double& getTimeshiftPrior() { return timeshiftCamToImuPrior_; }
  std::shared_ptr<aslam::backend::TransformationDesignVariable>
  getExtrinsicDv() {
    return T_c_b_Dv_;
  }
  std::shared_ptr<aslam::backend::Scalar> getTimeOffsetDv() {
    return cameraTimeToImuTimeDv_;
  }

  std::vector<std::vector<std::shared_ptr<aslam::backend::ErrorTerm>>>&
  getAllReprojectionErrors() {
    return allReprojectionErrors_;
  }

  CameraParameters& getCamConfig() { return camConfig_; }
  CalibrationTargetParameters& getTargetConfig() { return targetConfig_; }
  double getCornerUncertainty() const { return cornerUncertainty_; }

 private:
  ImageDatasetReader dataset_;
  CameraParameters camConfig_;
  CalibrationTargetParameters targetConfig_;
  std::shared_ptr<AslamCamera> camera_;
  std::shared_ptr<aslam::cameras::GridDetector> detector_;
  std::vector<aslam::cameras::GridCalibrationTargetObservation>
      targetObservations_;

  // Configuration
  double cornerUncertainty_;

  // Calibration parameters
  sm::kinematics::Transformation T_extrinsic_;
  double timeshiftCamToImuPrior_;
  Eigen::Vector3d gravity_w_;

  // Design variables (set by addDesignVariables)
  std::shared_ptr<aslam::backend::TransformationDesignVariable> T_c_b_Dv_;
  std::shared_ptr<aslam::backend::Scalar> cameraTimeToImuTimeDv_;

  // Reprojection errors (set by addCameraErrorTerms)
  std::vector<std::vector<std::shared_ptr<aslam::backend::ErrorTerm>>>
      allReprojectionErrors_;
};

/**
 * @brief Camera chain (multiple cameras)
 */
class IccCameraChain {
 public:
  IccCameraChain(const CameraChainParameters& chainConfig,
                 const CalibrationTargetParameters& targetConfig,
                 const std::vector<ImageDatasetReader>& datasets);

  void initializeBaselines();

  bsplines::BSplinePose initializePoseSplineFromCameraChain(
      int splineOrder = 6, int poseKnotsPerSecond = 100,
      double timeOffsetPadding = 0.02);

  void findCameraTimespan();

  void findOrientationPriorCameraChainToImu(IccImu& imu);

  Eigen::Vector3d getEstimatedGravity();

  std::pair<sm::kinematics::Transformation, double> getResultBaseline(
      int fromCamANr, int toCamBNr) const;

  sm::kinematics::Transformation getResultTrafoImuToCam(int camNr) const;

  double getResultTimeshift(int camNr) const;

  /**
   * @brief Add design variables to optimization problem
   */
  template <typename ProblemType>
    requires HasOptimizationProblemMethods<ProblemType>
  void addDesignVariables(ProblemType& problem, bool noTimeCalibration,
                          bool noChainExtrinsics) {
    // add the design variables (T(R,t) & time)  for all induvidual cameras
    for (auto [camNr, cam] : std::ranges::views::enumerate(camList_)) {
      bool noExtrinsics;
      int baselinedv_group_id;
      if (camNr == 0) {
        noExtrinsics = false;
        baselinedv_group_id = CALIBRATION_GROUP_ID;
      } else {
        noExtrinsics = noChainExtrinsics;
        baselinedv_group_id = HELPER_GROUP_ID;
      }
      cam->addDesignVariables(problem, noExtrinsics, noTimeCalibration,
                              baselinedv_group_id);
    }
  }

  void addCameraChainerrorTerms(
      aslam::backend::OptimizationProblemBase& problem,
      const std::shared_ptr<aslam::splines::BSplinePoseDesignVariable>&
          poseSplineDv,
      double blakeZissermanDf = 0.0, double timeOffsetPadding = 0.0);

  // Getters
  const std::vector<std::shared_ptr<IccCamera>>& getCamList() const {
    return camList_;
  }
  std::vector<std::shared_ptr<IccCamera>>& getCamList() { return camList_; }
  size_t numCameras() const { return camList_.size(); }
  CameraChainParameters& getChainConfig() { return chainConfig_; }

 private:
  aslam::Time timeStart_;
  aslam::Time timeEnd_;
  CameraChainParameters chainConfig_;
  std::vector<std::shared_ptr<IccCamera>> camList_;
  std::vector<std::shared_ptr<aslam::backend::TransformationDesignVariable>>
      transformations_;
  std::vector<std::shared_ptr<aslam::backend::Scalar>> timeOffsets_;
};

/**
 * @brief IMU sensor for calibration
 */
class IccImu {
 public:
  class ImuParameters : public ::kalibr::ImuParameters {
   public:
    ImuParameters() = delete;
    explicit ImuParameters(const ::kalibr::ImuParameters& imuConfig, int imuNr);
    void setImuPose(const sm::kinematics::Transformation& T_i_b);
    void setTimeOffset(double offset);
    std::string formatIndented(const std::string& indent,
                               std::vector<double> array) const;
    void printDetails(std::ostream& ofs = std::cout) const;

   private:
    int imuNr_;
  };

  struct ImuMeasurement {
    aslam::Time stamp;
    Eigen::Vector3d omega;  // Angular velocity (rad/s)
    Eigen::Vector3d alpha;  // Linear acceleration (m/s^2)
    Eigen::Matrix3d omegaR;
    Eigen::Matrix3d omegaInvR;
    Eigen::Matrix3d alphaR;
    Eigen::Matrix3d alphaInvR;

    ImuMeasurement() = default;

    ImuMeasurement(const aslam::Time& stamp, const Eigen::Vector3d& omega,
                   const Eigen::Vector3d& alpha, const Eigen::Matrix3d& Rgyro,
                   const Eigen::Matrix3d& Raccel)
        : stamp(stamp),
          omega(omega),
          alpha(alpha),
          omegaR(Rgyro),
          omegaInvR(Rgyro.inverse()),
          alphaR(Raccel),
          alphaInvR(Raccel.inverse()) {}
  };
  virtual void updateImuConfig();

  const ImuParameters& getImuConfig();

  void loadImuData();

  IccImu(const ::kalibr::ImuParameters& imuConfig,
         const ImuDatasetReader& dataset, bool isReferenceImu = true,
         bool estimateTimeDelay = true, int imuNr = 0);

  virtual ~IccImu() = default;

  /**
   * @brief Add design variables to optimization problem
   */
  template <typename ProblemType>
    requires HasOptimizationProblemMethods<ProblemType>
  void addDesignVariables(ProblemType& problem) {
    gyroBiasDv_ =
        std::make_shared<aslam::splines::EuclideanBSplineDesignVariable>(
            *gyroBias_);
    accelBiasDv_ =
        std::make_shared<aslam::splines::EuclideanBSplineDesignVariable>(
            *accelBias_);

    addSplineDesignVariables(problem, *gyroBiasDv_, true, HELPER_GROUP_ID);
    addSplineDesignVariables(problem, *accelBiasDv_, true, HELPER_GROUP_ID);
    q_i_b_Dv_ =
        std::make_shared<aslam::backend::RotationQuaternion>(q_i_b_Prior_);
    if constexpr (requires {
                    problem.addDesignVariable(q_i_b_Dv_, HELPER_GROUP_ID);
                  }) {
      problem.addDesignVariable(q_i_b_Dv_, HELPER_GROUP_ID);
    } else {
      problem.addDesignVariable(q_i_b_Dv_.get(), HELPER_GROUP_ID);
    }
    q_i_b_Dv_->setActive(false);
    r_b_Dv_ = std::make_shared<aslam::backend::EuclideanPoint>(
        Eigen::Vector3d::Zero());
    if constexpr (requires {
                    problem.addDesignVariable(r_b_Dv_, HELPER_GROUP_ID);
                  }) {
      problem.addDesignVariable(r_b_Dv_, HELPER_GROUP_ID);
    } else {
      problem.addDesignVariable(r_b_Dv_.get(), HELPER_GROUP_ID);
    }
    r_b_Dv_->setActive(false);

    if (!isReferenceImu_) {
      q_i_b_Dv_->setActive(true);
      r_b_Dv_->setActive(true);
    }
  }

  /**
   * @brief Add error terms to optimization problem
   */
  virtual void addAccelerometerErrorTerms(
      aslam::backend::OptimizationProblemBase& problem,
      aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
      const aslam::backend::EuclideanExpression& g_w, double mSigma = 0.0,
      double accelNoiseScale = 1.0);
  virtual void addGyroscopeErrorTerms(
      aslam::backend::OptimizationProblemBase& problem,
      aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
      double mSigma = 0.0, double gyroNoiseScale = 1.0,
      const aslam::backend::EuclideanExpression& g_w =
          aslam::backend::EuclideanExpression(Eigen::Vector3d(0, 0, 0)));

  /**
   * @brief Initialize bias splines
   */
  void initBiasSplines(const bsplines::BSplinePose& poseSpline, int splineOrder,
                       int biasKnotsPerSecond);

  void addBiasMotionTerms(aslam::backend::OptimizationProblemBase& problem);

  sm::kinematics::Transformation getTransformationFromBodyToImu() const;

  void findOrientationPrior(const IccImu& referenceImu);

  // Getters
  const ImuDatasetReader& getDataset() const { return dataset_; }
  ImuDatasetReader& getDataset() { return dataset_; }
  const std::vector<ImuMeasurement>& getImuData() const { return imuData_; }
  double getTimeOffset() const { return timeOffset_; }
  void setTimeOffset(double offset) { timeOffset_ = offset; }
  std::vector<std::shared_ptr<kalibr_errorterms::EuclideanError>>
  getGyroErrors() const {
    return gyroErrors_;
  }
  std::vector<std::shared_ptr<kalibr_errorterms::EuclideanError>>
  getAccelErrors() const {
    return accelErrors_;
  }

  // Get bias design variables (for plotting)
  std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable>
  getAccelBiasDv() const {
    return accelBiasDv_;
  }

  std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable>
  getGyroBiasDv() const {
    return gyroBiasDv_;
  }
  int& getGyroBiasPriorCount() { return GyroBiasPriorCount_; }
  Eigen::Vector3d& getGyroBiasPrior() { return GyroBiasPrior_; }

 protected:
  ImuDatasetReader dataset_;
  ImuParameters imuConfig_;
  bool isReferenceImu_;
  bool estimateTimeDelay_;
  double accelUncertaintyDiscrete_, accelRandomWalk_, accelUncertainty_;
  double gyroUncertaintyDiscrete_, gyroRandomWalk_, gyroUncertainty_;
  Eigen::Vector3d GyroBiasPrior_;
  int GyroBiasPriorCount_;
  Eigen::Vector4d q_i_b_Prior_;
  double timeOffset_;
  std::vector<ImuMeasurement> imuData_;

  // Design variables
  std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> gyroBiasDv_;
  std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> accelBiasDv_;
  std::shared_ptr<aslam::backend::RotationQuaternion>
      q_i_b_Dv_;  // Rotation from IMU to body
  std::shared_ptr<aslam::backend::EuclideanPoint>
      r_b_Dv_;  // Position from IMU to body

  // Bias splines (3D for gyro and accel)
  std::shared_ptr<bsplines::BSpline> gyroBias_;
  std::shared_ptr<bsplines::BSpline> accelBias_;

  // Error terms
  std::vector<std::shared_ptr<kalibr_errorterms::EuclideanError>> accelErrors_;
  std::vector<std::shared_ptr<kalibr_errorterms::EuclideanError>> gyroErrors_;

 private:
  struct ShiftCost {
    explicit ShiftCost(const std::function<Eigen::VectorXd(double)>& ref,
                       const std::function<Eigen::VectorXd(double)>& abs)
        : ref_(ref), abs_(abs) {}

    bool operator()(const double* const shift, double* residual) const {
      Eigen::VectorXd ref_vals = ref_(shift[0]);
      Eigen::VectorXd abs_vals = abs_(shift[0]);
      if (ref_vals.size() != abs_vals.size()) {
        throw std::runtime_error("ShiftCost: size not match");
      }
      Eigen::VectorXd diff = ref_vals - abs_vals;
      double sum = diff.squaredNorm();

      residual[0] = sum;
      return true;
    }

   private:
    std::function<Eigen::VectorXd(double)> ref_;
    std::function<Eigen::VectorXd(double)> abs_;
  };
};

/**
 * @brief IMU with scale and misalignment
 */
class IccScaledMisalignedImu : public IccImu {
 public:
  class ImuParameters : public ::kalibr::IccImu::ImuParameters {
   public:
    ImuParameters() = delete;
    explicit ImuParameters(const ::kalibr::ImuParameters& imuConfig, int imuNr);
    void printDetails(std::ostream& ofs = std::cout) const;
    void setIntrisicsMatrices(const Eigen::Matrix3d& M_accel,
                              const Eigen::Matrix3d& C_gyro_i,
                              const Eigen::Matrix3d& M_gyro,
                              const Eigen::Matrix3d& Ma_gyro);
  };

  IccScaledMisalignedImu(const ::kalibr::ImuParameters& imuConfig,
                         const ImuDatasetReader& dataset,
                         bool isReferenceImu = true,
                         bool estimateTimeDelay = true, int imuNr = 0);

  void updateImuConfig() override;

  const ImuParameters& getImuConfig();

  template <typename ProblemType>
    requires HasOptimizationProblemMethods<ProblemType>
  void addDesignVariables(ProblemType& problem) {
    // Call base class
    IccImu::addDesignVariables(problem);

    q_gyro_i_Dv_ = std::make_shared<aslam::backend::RotationQuaternion>(
        Eigen::Vector4d(0, 0, 0, 1));
    problem.addDesignVariable(q_gyro_i_Dv_.get(), HELPER_GROUP_ID);
    q_gyro_i_Dv_->setActive(true);

    Eigen::Matrix3i updatePattern;
    updatePattern << 1, 0, 0, 1, 1, 0, 1, 1, 1;
    M_accel_Dv_ = std::make_shared<aslam::backend::MatrixBasic>(
        Eigen::Matrix3d::Identity(), updatePattern);
    problem.addDesignVariable(M_accel_Dv_.get(), HELPER_GROUP_ID);
    M_accel_Dv_->setActive(true);

    M_gyro_Dv_ = std::make_shared<aslam::backend::MatrixBasic>(
        Eigen::Matrix3d::Identity(), updatePattern);
    problem.addDesignVariable(M_gyro_Dv_.get(), HELPER_GROUP_ID);
    M_gyro_Dv_->setActive(true);

    M_accel_gyro_Dv_ = std::make_shared<aslam::backend::MatrixBasic>(
        Eigen::Matrix3d::Zero(), Eigen::Matrix3i::Ones());
    problem.addDesignVariable(M_accel_gyro_Dv_.get(), HELPER_GROUP_ID);
    M_accel_gyro_Dv_->setActive(true);
  }

  void addAccelerometerErrorTerms(
      aslam::backend::OptimizationProblemBase& problem,
      aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
      const aslam::backend::EuclideanExpression& g_w, double mSigma = 0.0,
      double accelNoiseScale = 1.0) override;

  void addGyroscopeErrorTerms(
      aslam::backend::OptimizationProblemBase& problem,
      aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
      double mSigma = 0.0, double gyroNoiseScale = 1.0,
      const aslam::backend::EuclideanExpression& g_w =
          aslam::backend::EuclideanExpression(Eigen::Vector3d(0, 0,
                                                              0))) override;

 protected:
  // Scale and misalignment matrices
  std::shared_ptr<aslam::backend::RotationQuaternion> q_gyro_i_Dv_;
  std::shared_ptr<aslam::backend::MatrixBasic> M_accel_Dv_;
  std::shared_ptr<aslam::backend::MatrixBasic> M_gyro_Dv_;
  std::shared_ptr<aslam::backend::MatrixBasic> M_accel_gyro_Dv_;
  ImuParameters imuConfig_;
};

/**
 * @brief IMU with scale, misalignment, and size effect
 */
class IccScaledMisalignedSizeEffectImu : public IccScaledMisalignedImu {
 public:
  class ImuParameters : public ::kalibr::IccScaledMisalignedImu::ImuParameters {
   public:
    ImuParameters() = delete;
    explicit ImuParameters(const ::kalibr::ImuParameters& imuConfig, int imuNr);
    void printDetails(std::ostream& ofs = std::cout) const;
    void setAccelerometerLeverArms(const Eigen::Vector3d& rx_i,
                                   const Eigen::Vector3d& ry_i,
                                   const Eigen::Vector3d& rz_i);
  };
  IccScaledMisalignedSizeEffectImu(const ::kalibr::ImuParameters& imuConfig,
                                   const ImuDatasetReader& dataset,
                                   bool isReferenceImu = true,
                                   bool estimateTimeDelay = true,
                                   int imuNr = 0);

  void updateImuConfig() override;

  const ImuParameters& getImuConfig();

  template <typename ProblemType>
    requires HasOptimizationProblemMethods<ProblemType>
  void addDesignVariables(ProblemType& problem) {
    // Call base class
    IccScaledMisalignedImu::addDesignVariables(problem);

    rx_i_Dv_ = std::make_shared<aslam::backend::EuclideanPoint>(
        Eigen::Vector3d::Zero());
    problem.addDesignVariable(rx_i_Dv_.get(), HELPER_GROUP_ID);
    rx_i_Dv_->setActive(true);

    ry_i_Dv_ = std::make_shared<aslam::backend::EuclideanPoint>(
        Eigen::Vector3d::Zero());
    problem.addDesignVariable(ry_i_Dv_.get(), HELPER_GROUP_ID);
    ry_i_Dv_->setActive(true);

    rz_i_Dv_ = std::make_shared<aslam::backend::EuclideanPoint>(
        Eigen::Vector3d::Zero());
    problem.addDesignVariable(rz_i_Dv_.get(), HELPER_GROUP_ID);
    rz_i_Dv_->setActive(true);

    Eigen::Matrix3d A;
    A.diagonal() << 1.0, 0.0, 0.0;
    Ix_Dv_ = std::make_shared<aslam::backend::MatrixBasic>(
        A, Eigen::Matrix3i::Zero());
    problem.addDesignVariable(Ix_Dv_.get(), HELPER_GROUP_ID);
    Ix_Dv_->setActive(false);

    A.diagonal() << 0.0, 1.0, 0.0;
    Iy_Dv_ = std::make_shared<aslam::backend::MatrixBasic>(
        A, Eigen::Matrix3i::Zero());
    problem.addDesignVariable(Iy_Dv_.get(), HELPER_GROUP_ID);
    Iy_Dv_->setActive(false);

    A.diagonal() << 0.0, 0.0, 1.0;
    Iz_Dv_ = std::make_shared<aslam::backend::MatrixBasic>(
        A, Eigen::Matrix3i::Zero());
    problem.addDesignVariable(Iz_Dv_.get(), HELPER_GROUP_ID);
    Iz_Dv_->setActive(false);
  }

  void addAccelerometerErrorTerms(
      aslam::backend::OptimizationProblemBase& problem,
      aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
      const aslam::backend::EuclideanExpression& g_w, double mSigma = 0.0,
      double accelNoiseScale = 1.0) override;

 private:
  // Size effect parameters (eccentric mounting)
  std::shared_ptr<aslam::backend::EuclideanPoint> rx_i_Dv_;
  std::shared_ptr<aslam::backend::EuclideanPoint> ry_i_Dv_;
  std::shared_ptr<aslam::backend::EuclideanPoint> rz_i_Dv_;
  std::shared_ptr<aslam::backend::MatrixBasic> Ix_Dv_;
  std::shared_ptr<aslam::backend::MatrixBasic> Iy_Dv_;
  std::shared_ptr<aslam::backend::MatrixBasic> Iz_Dv_;
  ImuParameters imuConfig_;
};

}  // namespace kalibr

#endif  // ICC_SENSORS_HPP
