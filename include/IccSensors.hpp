#ifndef ICC_SENSORS_HPP
#define ICC_SENSORS_HPP

#include <Eigen/Core>
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Scalar.hpp>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <aslam/splines/EuclideanBSplineDesignVariable.hpp>
#include <bsplines/BSplinePose.hpp>
#include <cstddef>
#include <fstream>
#include <kalibr_backend/MatrixDesignVariable.hpp>
#include <kalibr_backend/TransformationDesignVariable.hpp>
#include <kalibr_common/ConfigReader.hpp>
#include <kalibr_common/ImageDatasetReader.hpp>
#include <kalibr_common/ImuDatasetReader.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <variant>
#include <vector>

#include "IccCalibrator.hpp"
#include "aslam/backend/EuclideanPoint.hpp"
#include "sm/kinematics/Transformation.hpp"

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
                          bool noExtrinsics = true,
                          bool noTimeCalibration = true,
                          size_t baselinedv_group_id = HELPER_GROUP_ID);

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
      aslam::backend::OptimizationProblem& problem,
      const std::shared_ptr<aslam::splines::BSplinePoseDesignVariable>&
          poseSplineDv,
      const aslam::backend::TransformationExpression& T_cN_b,
      double blakeZissermanDf = 0.0, double timeOffsetPadding = 0.0);

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
  CameraParameters camConfig_;
  CalibrationTargetParameters targetConfig_;
  std::shared_ptr<AslamCamera> camera_;
  std::shared_ptr<aslam::cameras::GridDetector> detector_;
  std::vector<aslam::cameras::GridCalibrationTargetObservation> observations_;

  // Configuration
  double cornerUncertainty_;


  // Calibration parameters
  sm::kinematics::Transformation T_extrinsic_;
  double timeshiftCamToImuPrior_;
  Eigen::Vector3d gravity_w_;

  // Design variables (set by addDesignVariables)
  std::shared_ptr<aslam::backend::TransformationDesignVariable> dv_T_c_b_;
  std::shared_ptr<aslam::backend::Scalar> dv_time_offset_;

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
    std::string formatIndented(const std::string& indent,std::vector<double> array) const;
    void printDetails(std::ostream& ofs=std::cout) const;
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

  void loadImuData();

  IccImu(const ::kalibr::ImuParameters& imuConfig, const ImuDatasetReader& dataset,bool isReferenceImu=true, bool estimateTimeDelay=true,int imuNr=0);

  virtual ~IccImu() = default;

  /**
   * @brief Add design variables to optimization problem
   */
  virtual void addDesignVariables(aslam::backend::OptimizationProblem& problem);

  /**
   * @brief Add error terms to optimization problem
   */
  virtual void addAccelerometerErrorTerms(
      aslam::backend::OptimizationProblem& problem,
      aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
      const aslam::backend::EuclideanExpression& g_w,
    double mSigma=0.0,double accelNoiseScale=1.0);
  virtual void addGyroscopeErrorTerms(
      aslam::backend::OptimizationProblem& problem,
      aslam::splines::BSplinePoseDesignVariable& poseSplineDv, double mSigma=0.0,
      double gyroNoiseScale=1.0,const aslam::backend::EuclideanExpression& g_w=aslam::backend::EuclideanExpression(Eigen::Vector3d(0,0,0)));

  /**
   * @brief Initialize bias splines
   */
  void initBiasSplines(const bsplines::BSplinePose& poseSpline, int splineOrder,
                       int biasKnotsPerSecond);
  
  void addBiasMotionTerms(aslam::backend::OptimizationProblem& problem);

  sm::kinematics::Transformation getTransformationFromBodyToImu() const;

  void findOrientationPrior(const ImuDatasetReader& referenceImu);

  // Getters
  const ImuDatasetReader& getDataset() const { return dataset_; }
  ImuDatasetReader& getDataset() { return dataset_; }
  const std::vector<ImuMeasurement>& getImuData() const { return imuData_; }
  double getTimeOffset() const { return timeOffset_; }
  void setTimeOffset(double offset) { timeOffset_ = offset; }

 protected:
  ImuDatasetReader dataset_;
  ImuParameters imuConfig_;
  bool isReferenceImu_;
  bool estimateTimeDelay_;
  double accelUncertaintyDiscrete_, accelRandomWalk_, accelUncertainty_;
  double gyroUncertaintyDiscrete_, gyroRandomWalk_, gyroUncertainty_;
  Eigen::Vector3d GyroBiasPrior_ = Eigen::Vector3d::Zero();
  int GyroBiasPriorCount_ = 0;
  Eigen::Vector4d q_i_b_Prior_ = Eigen::Vector4d::Identity();
  double timeOffset_ = 0.0;
  std::vector<ImuMeasurement> imuData_;

  // Design variables
  std::variant<std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable>,
               std::shared_ptr<aslam::backend::EuclideanPoint>>
      gyroBiasDv_;
  std::variant<std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable>,
               std::shared_ptr<aslam::backend::EuclideanPoint>>
      accelBiasDv_;
  std::shared_ptr<aslam::backend::RotationQuaternion> q_i_b_Dv_; // Rotation from IMU to body
  std::shared_ptr<aslam::backend::EuclideanPoint> r_b_Dv_; // Position from IMU to body

  // Bias splines (3D for gyro and accel)
  std::shared_ptr<bsplines::BSpline> gyroBias_;
  std::shared_ptr<bsplines::BSpline> accelBias_;

  // Error terms
  std::vector<std::shared_ptr<aslam::backend::ErrorTerm>> accelErrors_;
  std::vector<std::shared_ptr<aslam::backend::ErrorTerm>> gyroErrors_;
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

// /**
//  * @brief IMU with scale and misalignment
//  */
// class IccScaledMisalignedImu : public IccImu {
//  public:
//   IccScaledMisalignedImu(const ImuParameters& imuConfig,
//                          const ImuDatasetReader& dataset);

//   void addDesignVariables(aslam::backend::OptimizationProblem& problem,
//                           const bsplines::BSplinePose& poseSpline) override;

//   void addErrorTerms(
//       aslam::backend::OptimizationProblem& problem,
//       const aslam::splines::BSplinePoseDesignVariable& poseDv,
//       const aslam::backend::EuclideanExpression& gravityExpression,
//       double gyroNoiseScale = 1.0, double accelNoiseScale = 1.0,
//       double huberGyro = -1.0, double huberAccel = -1.0) override;

//  protected:
//   // Scale and misalignment matrices
//   std::shared_ptr<aslam::backend::MatrixDesignVariable> M_gyro_;
//   std::shared_ptr<aslam::backend::MatrixDesignVariable> M_accel_;
// };

// /**
//  * @brief IMU with scale, misalignment, and size effect
//  */
// class IccScaledMisalignedSizeEffectImu : public IccScaledMisalignedImu {
//  public:
//   IccScaledMisalignedSizeEffectImu(const ImuParameters& imuConfig,
//                                    const ImuDatasetReader& dataset);

//   void addDesignVariables(aslam::backend::OptimizationProblem& problem,
//                           const bsplines::BSplinePose& poseSpline) override;

//   void addErrorTerms(
//       aslam::backend::OptimizationProblem& problem,
//       const aslam::splines::BSplinePoseDesignVariable& poseDv,
//       const aslam::backend::EuclideanExpression& gravityExpression,
//       double gyroNoiseScale = 1.0, double accelNoiseScale = 1.0,
//       double huberGyro = -1.0, double huberAccel = -1.0) override;

//  private:
//   // Size effect parameters (eccentric mounting)
//   std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> r_x_;
//   std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> r_y_;
//   std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> r_z_;
// };

}  // namespace kalibr

#endif  // ICC_SENSORS_HPP
