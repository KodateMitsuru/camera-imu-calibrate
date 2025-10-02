#ifndef KALIBR_COMMON_HPP
#define KALIBR_COMMON_HPP

#include <Eigen/Dense>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <map>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace kalibr_common {

// ============================================================================
// Camera Parameters
// ============================================================================

class CameraParameters {
 public:
  CameraParameters() = default;

  // Load from YAML node
  static CameraParameters fromYaml(const cv::FileNode& node);

  // Getters
  std::string getCameraModel() const { return camera_model_; }
  std::string getDistortionModel() const { return distortion_model_; }
  std::vector<double> getIntrinsics() const { return intrinsics_; }
  std::vector<double> getDistortionCoeffs() const { return distortion_coeffs_; }
  std::vector<int> getResolution() const { return resolution_; }
  std::string getCsvFile() const { return csv_file_; }
  std::string getImageFolder() const { return image_folder_; }
  std::string getRosTopic() const { return ros_topic_; }  // For compatibility
  Eigen::Matrix4d getT_cam_imu() const { return T_cam_imu_; }

  // Setters
  void setCameraModel(const std::string& model) { camera_model_ = model; }
  void setDistortionModel(const std::string& model) {
    distortion_model_ = model;
  }
  void setIntrinsics(const std::vector<double>& intrinsics) {
    intrinsics_ = intrinsics;
  }
  void setDistortionCoeffs(const std::vector<double>& coeffs) {
    distortion_coeffs_ = coeffs;
  }
  void setResolution(const std::vector<int>& res) { resolution_ = res; }
  void setCsvFile(const std::string& file) { csv_file_ = file; }
  void setImageFolder(const std::string& folder) { image_folder_ = folder; }

 private:
  std::string camera_model_;
  std::string distortion_model_;
  std::vector<double> intrinsics_;
  std::vector<double> distortion_coeffs_;
  std::vector<int> resolution_;
  std::string csv_file_;
  std::string image_folder_;
  std::string ros_topic_;  // For compatibility
  Eigen::Matrix4d T_cam_imu_ = Eigen::Matrix4d::Identity();
};

// ============================================================================
// Target Parameters
// ============================================================================

class TargetParameters {
 public:
  TargetParameters() = default;

  // Load from YAML node
  static TargetParameters fromYaml(const cv::FileNode& node);

  // Getters
  std::string getTargetType() const { return target_type_; }
  std::map<std::string, double> getTargetParams() const {
    return target_params_;
  }

  // Setters
  void setTargetType(const std::string& type) { target_type_ = type; }
  void setTargetParams(const std::map<std::string, double>& params) {
    target_params_ = params;
  }

 private:
  std::string target_type_;
  std::map<std::string, double> target_params_;
};

// ============================================================================
// IMU Parameters
// ============================================================================

class ImuParameters {
 public:
  ImuParameters() = default;
  ImuParameters(const std::string& name, bool isReferenceImu)
      : name_(name), is_reference_imu_(isReferenceImu) {}

  virtual ~ImuParameters() = default;

  // Getters
  std::string getName() const { return name_; }
  bool isReferenceImu() const { return is_reference_imu_; }
  Eigen::Matrix4d getT_imu_body() const { return T_imu_body_; }
  double getTimeOffset() const { return time_offset_; }

  // Get noise statistics
  std::tuple<double, double, double> getAccelerometerStatistics() const {
    return {accel_noise_density_, accel_random_walk_, accel_noise_density_};
  }

  std::tuple<double, double, double> getGyroStatistics() const {
    return {gyro_noise_density_, gyro_random_walk_, gyro_noise_density_};
  }

  // Setters
  void setImuPose(const Eigen::Matrix4d& T) { T_imu_body_ = T; }
  void setTimeOffset(double offset) { time_offset_ = offset; }

  virtual void printDetails(std::ostream& out = std::cout) const {
    out << "IMU: " << name_ << std::endl;
    out << "  Reference IMU: " << (is_reference_imu_ ? "yes" : "no")
        << std::endl;
    out << "  Time offset: " << time_offset_ << " s" << std::endl;
  }

 protected:
  std::string name_;
  bool is_reference_imu_ = true;
  Eigen::Matrix4d T_imu_body_ = Eigen::Matrix4d::Identity();
  double time_offset_ = 0.0;

  // Noise parameters
  double accel_noise_density_ = 0.01;
  double accel_random_walk_ = 0.0002;
  double gyro_noise_density_ = 0.005;
  double gyro_random_walk_ = 0.0001;

  friend class ImuConfig;
};

// ============================================================================
// IMU Configuration
// ============================================================================

class ImuConfig {
 public:
  ImuConfig() = default;

  // Load from YAML node
  static ImuConfig fromYaml(const cv::FileNode& node);

  // Getters
  std::string getName() const { return name_; }
  std::string getCsvFile() const { return csv_file_; }
  std::string getRosTopic() const { return ros_topic_; }  // For compatibility
  double getAccelNoiseDensity() const { return accel_noise_density_; }
  double getAccelRandomWalk() const { return accel_random_walk_; }
  double getGyroNoiseDensity() const { return gyro_noise_density_; }
  double getGyroRandomWalk() const { return gyro_random_walk_; }
  double getUpdateRate() const { return update_rate_; }
  Eigen::Matrix4d getT_cam_imu() const { return T_cam_imu_; }

  // Setters
  void setName(const std::string& name) { name_ = name; }
  void setCsvFile(const std::string& file) { csv_file_ = file; }

 private:
  std::string name_;
  std::string csv_file_;
  std::string ros_topic_;  // For compatibility
  double accel_noise_density_ = 0.01;
  double accel_random_walk_ = 0.0002;
  double gyro_noise_density_ = 0.005;
  double gyro_random_walk_ = 0.0001;
  double update_rate_ = 200.0;
  Eigen::Matrix4d T_cam_imu_ = Eigen::Matrix4d::Identity();
};

// ============================================================================
// Chain Parameters (for multi-camera)
// ============================================================================

class ChainParameters {
 public:
  ChainParameters() = default;

  // Load from YAML file
  static ChainParameters fromYamlFile(const std::string& filename);

  // Getters
  int numCameras() const { return cameras_.size(); }
  CameraParameters getCameraParameters(int idx) const;
  Eigen::Matrix4d getExtrinsicsLastCamToHere(int idx) const;

  // Add camera
  void addCamera(const CameraParameters& cam) { cameras_.push_back(cam); }

 private:
  std::vector<CameraParameters> cameras_;
  std::vector<Eigen::Matrix4d> extrinsics_;
};

// ============================================================================
// Parsed Arguments (command line / config)
// ============================================================================

struct ParsedArguments {
  std::vector<std::string> bagfile;  // Now CSV files
  std::pair<double, double> bag_from_to = {0.0, 0.0};
  double bag_freq = 0.0;
  bool perform_synchronization = false;
  double reprojection_sigma = 1.0;
  bool showextraction = false;
  bool extractionstepping = false;

  // Additional parameters
  std::string config_file;
  std::string output_file;
  int max_iterations = 50;
  double convergence_threshold = 1e-6;
};

// ============================================================================
// Camera Geometry Wrapper (ASLAM compatibility)
// ============================================================================

class AslamCamera {
 public:
  std::shared_ptr<aslam::cameras::CameraGeometryBase> geometry;

  // Factory method
  static std::shared_ptr<AslamCamera> fromParameters(
      const CameraParameters& params);

 private:
  AslamCamera() = default;
};

// ============================================================================
// Helper Functions
// ============================================================================

// Read matrix from YAML
Eigen::MatrixXd readMatrixFromYaml(const cv::FileNode& node);

// Read vector from YAML
template <typename T>
std::vector<T> readVectorFromYaml(const cv::FileNode& node);

// Parse configuration file
ParsedArguments parseConfigFile(const std::string& config_file);

}  // namespace kalibr_common

#endif  // KALIBR_COMMON_HPP
