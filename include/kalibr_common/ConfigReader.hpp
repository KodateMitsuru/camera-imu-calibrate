#ifndef KALIBR_COMMON_CONFIG_READER_HPP
#define KALIBR_COMMON_CONFIG_READER_HPP

#include <Eigen/Dense>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>
#include <functional>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <unordered_map>
#include <vector>
#include <any>

// Forward declarations
// namespace aslam {
// namespace backend {
// class ErrorTerm;
// class HomogeneousExpression;
// }  // namespace backend
// }  // namespace aslam

namespace kalibr {

/**
 * @brief Camera model types
 */
enum class CameraModel {
  Pinhole,
  Omni,
  EUCM,  // Extended Unified Camera Model
  DS     // Double Sphere
};

/**
 * @brief Distortion model types
 */
enum class DistortionModel {
  RadTan,       // Radial-Tangential (Brown-Conrady)
  Equidistant,  // Kannala-Brandt
  FOV,          // Field of View
  None          // No distortion
};

/**
 * @brief Calibration target types
 */
enum class TargetType { Aprilgrid, Checkerboard, Circlegrid };

/**
 * @brief Base class for parameter readers
 */
class ParametersBase {
 public:
  using DictType = std::unordered_map<std::string, std::any>;
  ParametersBase(const std::string& yamlFile, const std::string& name, bool createYaml = false);
  virtual ~ParametersBase() = default;

  /**
   * @brief Read YAML file
   */
  DictType readYaml();

  /**
   * @brief Write to YAML file
   */
  void writeYaml(const std::string& filename = "");

  /**
   * @brief Get YAML file path
   */
  DictType getYamlDict() const;

  void setYamlDict(const DictType& dict);

 protected:
  /**
   * @brief Raise error with formatted message
   */
  void raiseError(const std::string& message) const;

  std::string yamlFile_;
  std::string name_;
  DictType data_;
};

/**
 * @brief Camera parameters reader
 */
class CameraParameters : public ParametersBase {
 public:
  explicit CameraParameters(const std::string& yamlFile,bool createYaml = false);

  // Accessors
  std::string getImageFolder() const;
  void setImageFolder(const std::string& folder);

  std::pair<CameraModel, std::vector<double>> getIntrinsics() const;
  void setIntrinsics(CameraModel model, const std::vector<double>& intrinsics);

  std::pair<DistortionModel, std::vector<double>> getDistortion() const;
  void setDistortion(DistortionModel model, const std::vector<double>& coeffs);

  Eigen::Vector2i getResolution() const;
  void setResolution(const Eigen::Vector2i& resolution);

  double getLineDelay() const;
  void setLineDelay(double lineDelay);

  double getReprojectionSigma() const;
  void setReprojectionSigma(double sigma);

  // Helpers
  void printDetails(std::ostream& os = std::cout) const;

 private:
  void checkIntrinsics(CameraModel model,
                       const std::vector<double>& intrinsics) const;
  void checkDistortion(DistortionModel model,
                       const std::vector<double>& coeffs) const;
  void checkResolution(const Eigen::Vector2i& resolution) const;
};

/**
 * @brief IMU parameters reader
 */
class ImuParameters : public ParametersBase {
 public:
  explicit ImuParameters(const std::string& yamlFile, bool createYaml = false);

  // Accessors
  std::string getCsvFile() const;
  void setCsvFile(const std::string& file);

  double getUpdateRate() const;
  void setUpdateRate(double updateRate);

  // Returns: (discrete_noise, random_walk, continuous_noise)
  std::tuple<double, double, double> getAccelerometerStatistics() const;
  void setAccelerometerStatistics(double noiseDensity, double randomWalk);

  // Returns: (discrete_noise, random_walk, continuous_noise)
  std::tuple<double, double, double> getGyroscopeStatistics() const;
  void setGyroscopeStatistics(double noiseDensity, double randomWalk);

  // Helpers
  void printDetails(std::ostream& os = std::cout) const;

 private:
  void checkUpdateRate(double updateRate) const;
  void checkAccelerometerStatistics(double noiseDensity,
                                    double randomWalk) const;
  void checkGyroscopeStatistics(double noiseDensity, double randomWalk) const;
};

class ImuSetParameters : public ParametersBase {
 public:
  explicit ImuSetParameters(const std::string& yamlFile, bool createYaml = false);

  size_t numImus() const {
    return imuCount_;
  }

  void addImuConfig(const ImuParameters& imuConfig,std::string name = "");
 private:
  size_t imuCount_ = 0;
};

/**
 * @brief Calibration target parameters reader
 */
class CalibrationTargetParameters : public ParametersBase {
 public:
  explicit CalibrationTargetParameters(const std::string& yamlFile,
                                       bool createYaml = false);

  // Accessors
  TargetType getTargetType() const;

  // Checkerboard parameters
  struct CheckerboardParams {
    int rows;
    int cols;
    double rowSpacing;  // meters
    double colSpacing;  // meters
  };
  CheckerboardParams getCheckerboardParams() const;

  // Circlegrid parameters
  struct CirclegridParams {
    int rows;
    int cols;
    double spacing;  // meters
    bool asymmetric;
  };
  CirclegridParams getCirclegridParams() const;

  // Aprilgrid parameters
  struct AprilgridParams {
    int tagRows;
    int tagCols;
    double tagSize;     // meters
    double tagSpacing;  // relative spacing
  };
  AprilgridParams getAprilgridParams() const;

  // Helpers
  void printDetails(std::ostream& os = std::cout) const;

 private:
  void checkTargetType(TargetType type) const;
};

/**
 * @brief Camera chain parameters reader
 *
 * Manages multiple cameras with extrinsics
 */
class CameraChainParameters : public ParametersBase {
 public:
  explicit CameraChainParameters(const std::string& yamlFile, bool createYaml = false);

  // Accessors
  size_t numCameras() const;

  std::shared_ptr<CameraParameters> getCameraParameters(size_t camIdx) const;

  // Extrinsics from previous camera to this camera (cam_n to cam_{n-1})
  // Not valid for cam0 (base camera)
  sm::kinematics::Transformation getExtrinsicsLastCamToHere(
      size_t camIdx) const;
  void setExtrinsicsLastCamToHere(size_t camIdx, const sm::kinematics::Transformation& T);

  // Extrinsics from IMU to camera
  sm::kinematics::Transformation getExtrinsicsImuToCam(size_t camIdx) const;
  void setExtrinsicsImuToCam(size_t camIdx, const sm::kinematics::Transformation& T);

  // Time shift between camera and IMU (t_imu = t_cam + shift)
  double getTimeshiftCamImu(size_t camIdx) const;
  void setTimeshiftCamImu(size_t camIdx, double timeshift);

  // Camera overlap information
  std::vector<int> getCamOverlaps(size_t camIdx) const;
  void setCamOverlaps(size_t camIdx, const std::vector<int>& overlaps);

  // Helpers
  void printDetails(std::ostream& os = std::cout) const;

 private:
  void checkCameraIndex(size_t camIdx) const;
};

/**
 * @brief ASLAM camera wrapper
 *
 * This class wraps ASLAM camera geometry and provides factory methods
 * to create cameras from configuration parameters
 */
class AslamCamera {
 public:
  /**
   * @brief Create ASLAM camera from parameters
   */
  static std::shared_ptr<AslamCamera> fromParameters(
      CameraModel cameraModel, const std::vector<double>& intrinsics,
      DistortionModel distModel, const std::vector<double>& distCoeffs,
      const Eigen::Vector2i& resolution);

  static std::shared_ptr<AslamCamera> fromParameters(
      const CameraParameters& params);

  /**
   * @brief Get the camera geometry
   */
  std::shared_ptr<aslam::cameras::CameraGeometryBase> getGeometry() const {
    return geometry_;
  }

  /**
   * @brief Create reprojection error term with correct concrete type
   *
   * This method uses the stored concrete geometry type to create
   * a properly typed SimpleReprojectionError without needing reflection.
   *
   * @param measurement The measured 2D image point
   * @param invR The inverse measurement covariance
   * @param point The 3D point expression in camera coordinates
   * @return Shared pointer to the error term
   */
  std::shared_ptr<aslam::backend::ErrorTerm> createReprojectionError(
      const Eigen::Vector2d& measurement, const Eigen::Matrix2d& invR,
      const aslam::backend::HomogeneousExpression& point) const;

 private:
  AslamCamera() = default;

  std::shared_ptr<aslam::cameras::CameraGeometryBase> geometry_;

  // Store the concrete camera type for dispatch
  // This enables zero-overhead type dispatch without reflection
  std::function<std::shared_ptr<aslam::backend::ErrorTerm>(
      const Eigen::Vector2d&, const Eigen::Matrix2d&,
      const aslam::backend::HomogeneousExpression&)>
      errorFactory_;
};

// Helper functions
std::string cameraModelToString(CameraModel model);
CameraModel stringToCameraModel(const std::string& str);
std::string distortionModelToString(DistortionModel model);
DistortionModel stringToDistortionModel(const std::string& str);
std::string targetTypeToString(TargetType type);
TargetType stringToTargetType(const std::string& str);

}  // namespace kalibr

#endif  // KALIBR_COMMON_CONFIG_READER_HPP
