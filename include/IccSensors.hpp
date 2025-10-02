#ifndef ICCSENSORS_HPP
#define ICCSENSORS_HPP

#include <Eigen/Dense>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// ASLAM includes
#include <aslam/backend/DesignVariable.hpp>
#include <aslam/backend/EuclideanDirection.hpp>
#include <aslam/backend/EuclideanPoint.hpp>
#include <aslam/backend/MEstimatorPolicies.hpp>
#include <aslam/backend/MatrixBasic.hpp>
#include <aslam/backend/OptimizationProblem.hpp>
#include <aslam/backend/Optimizer2.hpp>
#include <aslam/backend/Optimizer2Options.hpp>
#include <aslam/backend/RotationQuaternion.hpp>
#include <aslam/backend/Scalar.hpp>
#include <aslam/cameras/CameraGeometry.hpp>
#include <aslam/cameras/Frame.hpp>
#include <aslam/cameras/Keypoint.hpp>
#include <aslam/cv/GridCalibrationTarget.hpp>
#include <aslam/cv/GridDetector.hpp>
#include <aslam/splines/BSplinePoseDesignVariable.hpp>
#include <aslam/splines/EuclideanBSplineDesignVariable.hpp>
#include <bsplines/BSpline.hpp>
#include <bsplines/BSplinePose.hpp>
#include <sm/kinematics/Transformation.hpp>
#include <sm/logging.hpp>
#include <sm/timing/Timer.hpp>

#include "IccDatasetReaders.hpp"

// Forward declarations
class IccCalibrator;

// Constants
constexpr int CALIBRATION_GROUP_ID = 0;
constexpr int HELPER_GROUP_ID = 1;

// Dataset readers initialization functions
std::shared_ptr<CsvImageDatasetReader> initCameraDataset(
    const std::string& csvPath, const std::string& imageFolder,
    const std::pair<double, double>& from_to, double freq);

std::shared_ptr<CsvImuDatasetReader> initImuDataset(
    const std::string& csvPath,
    const std::pair<double, double>& from_to = {0.0, 0.0});

// Mono Camera Class
class IccCamera {
 public:
  IccCamera(const kalibr_common::CameraParameters& camConfig,
            const kalibr_common::TargetParameters& targetConfig,
            std::shared_ptr<CsvImageDatasetReader> dataset,
            double reprojectionSigma = 1.0, bool showCorners = true,
            bool showReproj = true, bool showOneStep = false);

  ~IccCamera();

  // Setup calibration target
  void setupCalibrationTarget(
      const kalibr_common::TargetParameters& targetConfig,
      bool showExtraction = false, bool showReproj = false,
      bool imageStepping = false);

  // Find orientation prior between camera and IMU
  void findOrientationPriorCameraToImu(class IccImu& imu);

  // Get estimated gravity vector
  Eigen::Vector3d getEstimatedGravity() const;

  // Find timeshift between camera and IMU
  void findTimeshiftCameraImuPrior(class IccImu& imu, bool verbose = false);

  // Initialize pose spline from camera observations
  std::shared_ptr<bsplines::BSplinePose> initPoseSplineFromCamera(
      int splineOrder = 6, int poseKnotsPerSecond = 100,
      double timeOffsetPadding = 0.02);

  // Add design variables to optimization problem
  void addDesignVariables(aslam::backend::OptimizationProblem& problem,
                          bool noExtrinsics = true,
                          bool noTimeCalibration = true,
                          int baselinedv_group_id = HELPER_GROUP_ID);

  // Add camera error terms
  void addCameraErrorTerms(
      aslam::backend::OptimizationProblem& problem,
      aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
      const aslam::backend::TransformationExpression& T_cN_b,
      double blakeZissermanDf = 0.0, double timeOffsetPadding = 0.0);

 private:
  // Dataset and configuration
  std::shared_ptr<CsvImageDatasetReader> dataset_;
  kalibr_common::CameraParameters camConfig_;
  kalibr_common::TargetParameters targetConfig_;

  // Camera setup
  double cornerUncertainty_;
  sm::kinematics::Transformation T_extrinsic_;
  double timeshiftCamToImuPrior_;
  std::shared_ptr<kalibr_common::AslamCamera> camera_;

  // Calibration target
  std::shared_ptr<aslam::cv::GridDetector> detector_;
  std::vector<aslam::cv::TargetObservation> targetObservations_;

  // Gravity estimate
  Eigen::Vector3d gravity_w_;

  // Design variables
  std::shared_ptr<aslam::backend::TransformationDv> T_c_b_Dv_;
  std::shared_ptr<aslam::backend::Scalar> cameraTimeToImuTimeDv_;

  // Error terms
  std::vector<std::vector<std::shared_ptr<aslam::backend::ErrorTerm>>>
      allReprojectionErrors_;
};

// Camera Chain Class
class IccCameraChain {
 public:
  IccCameraChain(const kalibr_common::ChainParameters& chainConfig,
                 const kalibr_common::TargetParameters& targetConfig,
                 const kalibr_common::ParsedArguments& parsed);

  ~IccCameraChain();

  // Initialize baselines between cameras
  void initializeBaselines();

  // Initialize pose spline from camera chain
  std::shared_ptr<bsplines::BSplinePose> initializePoseSplineFromCameraChain(
      int splineOrder = 6, int poseKnotsPerSecond = 100,
      double timeOffsetPadding = 0.02);

  // Find camera timespan
  void findCameraTimespan();

  // Find orientation prior between camera chain and IMU
  void findOrientationPriorCameraChainToImu(class IccImu& imu);

  // Get estimated gravity
  Eigen::Vector3d getEstimatedGravity() const;

  // Get baseline transformation
  std::pair<sm::kinematics::Transformation, double> getResultBaseline(
      int fromCamANr, int toCamBNr);

  // Get transformation from IMU to camera
  sm::kinematics::Transformation getResultTrafoImuToCam(int camNr);

  // Get time shift for camera
  double getResultTimeShift(int camNr);

  // Add design variables
  void addDesignVariables(aslam::backend::OptimizationProblem& problem,
                          bool noTimeCalibration = true,
                          bool noChainExtrinsics = true);

  // Add camera chain error terms
  void addCameraChainErrorTerms(
      aslam::backend::OptimizationProblem& problem,
      aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
      double blakeZissermanDf = -1, double timeOffsetPadding = 0.0);

 public:
  std::vector<std::shared_ptr<IccCamera>> camList;
  kalibr_common::ChainParameters chainConfig;

 private:
  aslam::cv::Time timeStart_;
  aslam::cv::Time timeEnd_;
};

// IMU Measurement Class
class ImuMeasurement {
 public:
  ImuMeasurement(const aslam::cv::Time& stamp, const Eigen::Vector3d& omega,
                 const Eigen::Vector3d& alpha, const Eigen::Matrix3d& Rgyro,
                 const Eigen::Matrix3d& Raccel);

  Eigen::Vector3d omega;
  Eigen::Vector3d alpha;
  Eigen::Matrix3d omegaR;
  Eigen::Matrix3d omegaInvR;
  Eigen::Matrix3d alphaR;
  Eigen::Matrix3d alphaInvR;
  aslam::cv::Time stamp;
};

// Base IMU Class
class IccImu {
 public:
  // Nested ImuParameters class
  class ImuParameters : public kalibr_common::ImuParameters {
   public:
    ImuParameters(const kalibr_common::ImuConfig& imuConfig, int imuNr);

    void setImuPose(const Eigen::Matrix4d& T_i_b);
    void setTimeOffset(double time_offset);
    void printDetails(std::ostream& dest = std::cout) const;

   private:
    std::string formatIndented(const std::string& indent,
                               const Eigen::MatrixXd& array) const;
    int imuNr_;
  };

  IccImu(const kalibr_common::ImuConfig& imuConfig,
         const kalibr_common::ParsedArguments& parsed,
         bool isReferenceImu = true, bool estimateTimedelay = true,
         int imuNr = 0);

  virtual ~IccImu();

  // Get IMU configuration
  std::shared_ptr<ImuParameters> getImuConfig();

  // Update IMU configuration
  virtual void updateImuConfig();

  // Load IMU data from dataset
  void loadImuData();

  // Set the dataset (must be called before loadImuData)
  void setDataset(std::shared_ptr<CsvImuDatasetReader> dataset) {
    dataset_ = dataset;
  }

  // Add design variables
  virtual void addDesignVariables(aslam::backend::OptimizationProblem& problem);

  // Add accelerometer error terms
  virtual void addAccelerometerErrorTerms(
      aslam::backend::OptimizationProblem& problem,
      aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
      const aslam::backend::EuclideanExpression& g_w, double mSigma = 0.0,
      double accelNoiseScale = 1.0);

  // Add gyroscope error terms
  virtual void addGyroscopeErrorTerms(
      aslam::backend::OptimizationProblem& problem,
      aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
      double mSigma = 0.0, double gyroNoiseScale = 1.0,
      const aslam::backend::EuclideanExpression* g_w = nullptr);

  // Initialize bias splines
  void initBiasSplines(std::shared_ptr<bsplines::BSplinePose> poseSpline,
                       int splineOrder, int biasKnotsPerSecond);

  // Add bias motion terms
  void addBiasMotionTerms(aslam::backend::OptimizationProblem& problem);

  // Get transformation from body to IMU
  sm::kinematics::Transformation getTransformationFromBodyToImu() const;

  // Find orientation prior relative to reference IMU
  void findOrientationPrior(const IccImu& referenceImu);

 protected:
  // Dataset and configuration
  std::shared_ptr<CsvImuDatasetReader> dataset_;
  std::shared_ptr<ImuParameters> imuConfig_;

  // IMU properties
  bool isReferenceImu_;
  bool estimateTimedelay_;
  double timeOffset_;

  // Statistics
  double accelUncertaintyDiscrete_, accelRandomWalk_, accelUncertainty_;
  double gyroUncertaintyDiscrete_, gyroRandomWalk_, gyroUncertainty_;

  // Bias priors
  Eigen::Vector3d GyroBiasPrior_;
  int GyroBiasPriorCount_;

  // IMU data
  std::vector<ImuMeasurement> imuData_;

  // Initial estimates
  Eigen::Vector4d q_i_b_prior_;

  // Bias splines
  std::shared_ptr<bsplines::BSpline> gyroBias_;
  std::shared_ptr<bsplines::BSpline> accelBias_;

  // Design variables
  std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> gyroBiasDv_;
  std::shared_ptr<aslam::splines::EuclideanBSplineDesignVariable> accelBiasDv_;
  std::shared_ptr<aslam::backend::RotationQuaternionDv> q_i_b_Dv_;
  std::shared_ptr<aslam::backend::EuclideanPointDv> r_b_Dv_;

  // Error terms
  std::vector<std::shared_ptr<aslam::backend::ErrorTerm>> accelErrors_;
  std::vector<std::shared_ptr<aslam::backend::ErrorTerm>> gyroErrors_;
};

// Scaled Misaligned IMU Class
class IccScaledMisalignedImu : public IccImu {
 public:
  // Nested ImuParameters class
  class ImuParameters : public IccImu::ImuParameters {
   public:
    ImuParameters(const kalibr_common::ImuConfig& imuConfig, int imuNr);

    void printDetails(std::ostream& dest = std::cout) const override;
    void setIntrisicsMatrices(const Eigen::Matrix3d& M_accel,
                              const Eigen::Matrix3d& C_gyro_i,
                              const Eigen::Matrix3d& M_gyro,
                              const Eigen::Matrix3d& Ma_gyro);
  };

  IccScaledMisalignedImu(const kalibr_common::ImuConfig& imuConfig,
                         const kalibr_common::ParsedArguments& parsed,
                         bool isReferenceImu = true,
                         bool estimateTimedelay = true, int imuNr = 0);

  void updateImuConfig() override;
  void addDesignVariables(
      aslam::backend::OptimizationProblem& problem) override;

  void addAccelerometerErrorTerms(
      aslam::backend::OptimizationProblem& problem,
      aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
      const aslam::backend::EuclideanExpression& g_w, double mSigma = 0.0,
      double accelNoiseScale = 1.0) override;

  void addGyroscopeErrorTerms(
      aslam::backend::OptimizationProblem& problem,
      aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
      double mSigma = 0.0, double gyroNoiseScale = 1.0,
      const aslam::backend::EuclideanExpression* g_w = nullptr) override;

 protected:
  // Additional design variables for scaled misaligned model
  std::shared_ptr<aslam::backend::RotationQuaternionDv> q_gyro_i_Dv_;
  std::shared_ptr<aslam::backend::MatrixBasicDv> M_accel_Dv_;
  std::shared_ptr<aslam::backend::MatrixBasicDv> M_gyro_Dv_;
  std::shared_ptr<aslam::backend::MatrixBasicDv> M_accel_gyro_Dv_;
};

// Scaled Misaligned Size Effect IMU Class
class IccScaledMisalignedSizeEffectImu : public IccScaledMisalignedImu {
 public:
  // Nested ImuParameters class
  class ImuParameters : public IccScaledMisalignedImu::ImuParameters {
   public:
    ImuParameters(const kalibr_common::ImuConfig& imuConfig, int imuNr);

    void printDetails(std::ostream& dest = std::cout) const override;
    void setAccelerometerLeverArms(const Eigen::Vector3d& rx_i,
                                   const Eigen::Vector3d& ry_i,
                                   const Eigen::Vector3d& rz_i);
  };

  IccScaledMisalignedSizeEffectImu(const kalibr_common::ImuConfig& imuConfig,
                                   const kalibr_common::ParsedArguments& parsed,
                                   bool isReferenceImu = true,
                                   bool estimateTimedelay = true,
                                   int imuNr = 0);

  void updateImuConfig() override;
  void addDesignVariables(
      aslam::backend::OptimizationProblem& problem) override;

  void addAccelerometerErrorTerms(
      aslam::backend::OptimizationProblem& problem,
      aslam::splines::BSplinePoseDesignVariable& poseSplineDv,
      const aslam::backend::EuclideanExpression& g_w, double mSigma = 0.0,
      double accelNoiseScale = 1.0) override;

 protected:
  // Additional design variables for size effect model
  std::shared_ptr<aslam::backend::EuclideanPointDv> rx_i_Dv_;
  std::shared_ptr<aslam::backend::EuclideanPointDv> ry_i_Dv_;
  std::shared_ptr<aslam::backend::EuclideanPointDv> rz_i_Dv_;
  std::shared_ptr<aslam::backend::MatrixBasicDv> Ix_Dv_;
  std::shared_ptr<aslam::backend::MatrixBasicDv> Iy_Dv_;
  std::shared_ptr<aslam::backend::MatrixBasicDv> Iz_Dv_;
};

#endif  // ICCSENSORS_HPP
