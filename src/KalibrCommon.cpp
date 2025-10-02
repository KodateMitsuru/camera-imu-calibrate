#include "KalibrCommon.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>

namespace kalibr_common {

// ============================================================================
// Helper Functions Implementation
// ============================================================================

Eigen::MatrixXd readMatrixFromYaml(const cv::FileNode& node) {
  if (node.empty() || !node.isSeq()) {
    throw std::runtime_error("Invalid matrix format in YAML");
  }

  std::vector<std::vector<double>> data;
  for (const auto& row_node : node) {
    std::vector<double> row;
    for (const auto& val : row_node) {
      row.push_back(static_cast<double>(val));
    }
    data.push_back(row);
  }

  if (data.empty()) {
    return Eigen::MatrixXd();
  }

  size_t rows = data.size();
  size_t cols = data[0].size();
  Eigen::MatrixXd mat(rows, cols);

  for (size_t i = 0; i < rows; ++i) {
    if (data[i].size() != cols) {
      throw std::runtime_error("Inconsistent matrix row sizes in YAML");
    }
    for (size_t j = 0; j < cols; ++j) {
      mat(i, j) = data[i][j];
    }
  }

  return mat;
}

template <typename T>
std::vector<T> readVectorFromYaml(const cv::FileNode& node) {
  std::vector<T> result;
  if (node.empty() || !node.isSeq()) {
    return result;
  }

  for (const auto& val : node) {
    result.push_back(static_cast<T>(val));
  }

  return result;
}

// Explicit template instantiations
template std::vector<double> readVectorFromYaml<double>(const cv::FileNode&);
template std::vector<int> readVectorFromYaml<int>(const cv::FileNode&);
template std::vector<std::string> readVectorFromYaml<std::string>(
    const cv::FileNode&);

// ============================================================================
// CameraParameters Implementation
// ============================================================================

CameraParameters CameraParameters::fromYaml(const cv::FileNode& node) {
  CameraParameters params;

  if (!node.empty()) {
    if (!node["camera_model"].empty())
      params.camera_model_ = static_cast<std::string>(node["camera_model"]);

    if (!node["distortion_model"].empty())
      params.distortion_model_ =
          static_cast<std::string>(node["distortion_model"]);

    if (!node["intrinsics"].empty())
      params.intrinsics_ = readVectorFromYaml<double>(node["intrinsics"]);

    if (!node["distortion_coeffs"].empty())
      params.distortion_coeffs_ =
          readVectorFromYaml<double>(node["distortion_coeffs"]);

    if (!node["resolution"].empty())
      params.resolution_ = readVectorFromYaml<int>(node["resolution"]);

    if (!node["csv_file"].empty())
      params.csv_file_ = static_cast<std::string>(node["csv_file"]);

    if (!node["image_folder"].empty())
      params.image_folder_ = static_cast<std::string>(node["image_folder"]);

    if (!node["rostopic"].empty())
      params.ros_topic_ = static_cast<std::string>(node["rostopic"]);

    if (!node["T_cam_imu"].empty()) {
      Eigen::MatrixXd T = readMatrixFromYaml(node["T_cam_imu"]);
      if (T.rows() == 4 && T.cols() == 4) {
        params.T_cam_imu_ = T;
      }
    }
  }

  return params;
}

// ============================================================================
// TargetParameters Implementation
// ============================================================================

TargetParameters TargetParameters::fromYaml(const cv::FileNode& node) {
  TargetParameters params;

  if (!node.empty()) {
    if (!node["type"].empty())
      params.target_type_ = static_cast<std::string>(node["type"]);

    // AprilGrid parameters
    if (!node["tagRows"].empty())
      params.target_params_["tagRows"] = static_cast<double>(node["tagRows"]);
    if (!node["tagCols"].empty())
      params.target_params_["tagCols"] = static_cast<double>(node["tagCols"]);
    if (!node["tagSize"].empty())
      params.target_params_["tagSize"] = static_cast<double>(node["tagSize"]);
    if (!node["tagSpacing"].empty())
      params.target_params_["tagSpacing"] =
          static_cast<double>(node["tagSpacing"]);

    // Checkerboard parameters
    if (!node["targetRows"].empty())
      params.target_params_["targetRows"] =
          static_cast<double>(node["targetRows"]);
    if (!node["targetCols"].empty())
      params.target_params_["targetCols"] =
          static_cast<double>(node["targetCols"]);
    if (!node["rowSpacingMeters"].empty())
      params.target_params_["rowSpacingMeters"] =
          static_cast<double>(node["rowSpacingMeters"]);
    if (!node["colSpacingMeters"].empty())
      params.target_params_["colSpacingMeters"] =
          static_cast<double>(node["colSpacingMeters"]);

    // CircleGrid parameters
    if (!node["spacingMeters"].empty())
      params.target_params_["spacingMeters"] =
          static_cast<double>(node["spacingMeters"]);
    if (!node["asymmetricGrid"].empty())
      params.target_params_["asymmetricGrid"] =
          static_cast<double>(node["asymmetricGrid"]);
  }

  return params;
}

// ============================================================================
// ImuConfig Implementation
// ============================================================================

ImuConfig ImuConfig::fromYaml(const cv::FileNode& node) {
  ImuConfig config;

  if (!node.empty()) {
    if (!node["csv_file"].empty())
      config.csv_file_ = static_cast<std::string>(node["csv_file"]);

    if (!node["rostopic"].empty())
      config.ros_topic_ = static_cast<std::string>(node["rostopic"]);

    if (!node["accelerometer_noise_density"].empty())
      config.accel_noise_density_ =
          static_cast<double>(node["accelerometer_noise_density"]);

    if (!node["accelerometer_random_walk"].empty())
      config.accel_random_walk_ =
          static_cast<double>(node["accelerometer_random_walk"]);

    if (!node["gyroscope_noise_density"].empty())
      config.gyro_noise_density_ =
          static_cast<double>(node["gyroscope_noise_density"]);

    if (!node["gyroscope_random_walk"].empty())
      config.gyro_random_walk_ =
          static_cast<double>(node["gyroscope_random_walk"]);

    if (!node["update_rate"].empty())
      config.update_rate_ = static_cast<double>(node["update_rate"]);

    if (!node["T_cam_imu"].empty()) {
      Eigen::MatrixXd T = readMatrixFromYaml(node["T_cam_imu"]);
      if (T.rows() == 4 && T.cols() == 4) {
        config.T_cam_imu_ = T;
      }
    }
  }

  return config;
}

// ============================================================================
// ChainParameters Implementation
// ============================================================================

ChainParameters ChainParameters::fromYamlFile(const std::string& filename) {
  ChainParameters params;

  cv::FileStorage fs(filename, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    throw std::runtime_error("Failed to open config file: " + filename);
  }

  // Read cameras
  cv::FileNode cameras_node = fs["cameras"];
  if (!cameras_node.empty() && cameras_node.isMap()) {
    for (auto it = cameras_node.begin(); it != cameras_node.end(); ++it) {
      std::string cam_name = (*it).name();
      cv::FileNode cam_node = *it;

      CameraParameters cam_params = CameraParameters::fromYaml(cam_node);
      params.cameras_.push_back(cam_params);

      // Read extrinsics if available
      if (!cam_node["T_cn_cnm1"].empty()) {
        Eigen::MatrixXd T = readMatrixFromYaml(cam_node["T_cn_cnm1"]);
        if (T.rows() == 4 && T.cols() == 4) {
          params.extrinsics_.push_back(T);
        }
      } else {
        params.extrinsics_.push_back(Eigen::Matrix4d::Identity());
      }
    }
  }

  fs.release();
  return params;
}

CameraParameters ChainParameters::getCameraParameters(int idx) const {
  if (idx < 0 || idx >= static_cast<int>(cameras_.size())) {
    throw std::out_of_range("Camera index out of range");
  }
  return cameras_[idx];
}

Eigen::Matrix4d ChainParameters::getExtrinsicsLastCamToHere(int idx) const {
  if (idx < 0 || idx >= static_cast<int>(extrinsics_.size())) {
    return Eigen::Matrix4d::Identity();
  }
  return extrinsics_[idx];
}

// ============================================================================
// AslamCamera Implementation
// ============================================================================

std::shared_ptr<AslamCamera> AslamCamera::fromParameters(
    const CameraParameters& params) {
  auto camera = std::make_shared<AslamCamera>();

  // Note: This is a placeholder. In the real implementation,
  // you would create the appropriate ASLAM camera geometry here
  // based on the camera model and parameters.

  // For now, we'll leave the geometry as nullptr and implement
  // the actual camera creation in the ASLAM integration layer

  std::cout << "Creating camera with model: " << params.getCameraModel()
            << std::endl;
  std::cout << "  Resolution: " << params.getResolution()[0] << "x"
            << params.getResolution()[1] << std::endl;

  return camera;
}

// ============================================================================
// Configuration File Parser
// ============================================================================

ParsedArguments parseConfigFile(const std::string& config_file) {
  ParsedArguments args;
  args.config_file = config_file;

  cv::FileStorage fs(config_file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    throw std::runtime_error("Failed to open config file: " + config_file);
  }

  // Read calibration options
  cv::FileNode calib_node = fs["calibration"];
  if (!calib_node.empty()) {
    if (!calib_node["show_extraction"].empty())
      args.showextraction =
          static_cast<int>(calib_node["show_extraction"]) != 0;

    if (!calib_node["reprojection_sigma"].empty())
      args.reprojection_sigma =
          static_cast<double>(calib_node["reprojection_sigma"]);

    if (!calib_node["max_iterations"].empty())
      args.max_iterations = static_cast<int>(calib_node["max_iterations"]);

    if (!calib_node["convergence_delta_x"].empty())
      args.convergence_threshold =
          static_cast<double>(calib_node["convergence_delta_x"]);
  }

  // Read output options
  cv::FileNode output_node = fs["output"];
  if (!output_node.empty()) {
    if (!output_node["results_file"].empty())
      args.output_file = static_cast<std::string>(output_node["results_file"]);
  }

  // Read camera CSV files
  cv::FileNode cameras_node = fs["cameras"];
  if (!cameras_node.empty() && cameras_node.isMap()) {
    for (auto it = cameras_node.begin(); it != cameras_node.end(); ++it) {
      cv::FileNode cam_node = *it;
      if (!cam_node["csv_file"].empty()) {
        std::string csv_file = static_cast<std::string>(cam_node["csv_file"]);
        args.bagfile.push_back(csv_file);
      }
    }
  }

  fs.release();
  return args;
}

}  // namespace kalibr_common
