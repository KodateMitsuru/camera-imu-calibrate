#include <algorithm>
#include <cmath>
#include <iostream>
#include <kalibr_common/ConfigReader.hpp>
#include <opencv2/core/persistence.hpp>
#include <sstream>
#include <stdexcept>

// ASLAM camera includes
#include <aslam/cameras/CameraGeometry.hpp>
#include <aslam/cameras/DoubleSphereProjection.hpp>
#include <aslam/cameras/EquidistantDistortion.hpp>
#include <aslam/cameras/ExtendedUnifiedProjection.hpp>
#include <aslam/cameras/FovDistortion.hpp>
#include <aslam/cameras/GlobalShutter.hpp>
#include <aslam/cameras/NoDistortion.hpp>
#include <aslam/cameras/NoMask.hpp>
#include <aslam/cameras/OmniProjection.hpp>
#include <aslam/cameras/PinholeProjection.hpp>
#include <aslam/cameras/RadialTangentialDistortion.hpp>

namespace kalibr {

// ============================================================================
// Helper Functions - String conversions
// ============================================================================

std::string cameraModelToString(CameraModel model) {
  switch (model) {
    case CameraModel::Pinhole:
      return "pinhole";
    case CameraModel::Omni:
      return "omni";
    case CameraModel::EUCM:
      return "eucm";
    case CameraModel::DS:
      return "ds";
    default:
      throw std::runtime_error("Unknown camera model");
  }
}

CameraModel stringToCameraModel(const std::string& str) {
  std::string lower = str;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

  if (lower == "pinhole") return CameraModel::Pinhole;
  if (lower == "omni") return CameraModel::Omni;
  if (lower == "eucm") return CameraModel::EUCM;
  if (lower == "ds") return CameraModel::DS;

  throw std::runtime_error("Unknown camera model: " + str);
}

std::string distortionModelToString(DistortionModel model) {
  switch (model) {
    case DistortionModel::RadTan:
      return "radtan";
    case DistortionModel::Equidistant:
      return "equidistant";
    case DistortionModel::FOV:
      return "fov";
    case DistortionModel::None:
      return "none";
    default:
      throw std::runtime_error("Unknown distortion model");
  }
}

DistortionModel stringToDistortionModel(const std::string& str) {
  std::string lower = str;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

  if (lower == "radtan") return DistortionModel::RadTan;
  if (lower == "equidistant") return DistortionModel::Equidistant;
  if (lower == "fov") return DistortionModel::FOV;
  if (lower == "none") return DistortionModel::None;

  throw std::runtime_error("Unknown distortion model: " + str);
}

std::string targetTypeToString(TargetType type) {
  switch (type) {
    case TargetType::Aprilgrid:
      return "aprilgrid";
    case TargetType::Checkerboard:
      return "checkerboard";
    case TargetType::Circlegrid:
      return "circlegrid";
    default:
      throw std::runtime_error("Unknown target type");
  }
}

TargetType stringToTargetType(const std::string& str) {
  std::string lower = str;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

  if (lower == "aprilgrid") return TargetType::Aprilgrid;
  if (lower == "checkerboard") return TargetType::Checkerboard;
  if (lower == "circlegrid") return TargetType::Circlegrid;

  throw std::runtime_error("Unknown target type: " + str);
}

// ============================================================================
// ParametersBase Implementation
// ============================================================================

ParametersBase::ParametersBase(const std::string& yamlFile,
                               const std::string& name)
    : yamlFile_(yamlFile), name_(name) {
  readYaml();
}

void ParametersBase::readYaml() {
  fs_ = cv::FileStorage(yamlFile_, cv::FileStorage::READ);
  if (!fs_.isOpened()) {
    raiseError("Could not read configuration from " + yamlFile_);
  }
}

void ParametersBase::writeYaml(const std::string& filename) {
  std::string outFile = filename.empty() ? yamlFile_ : filename;

  cv::FileStorage fsOut(outFile, cv::FileStorage::WRITE);
  if (!fsOut.isOpened()) {
    raiseError("Could not write configuration to " + outFile);
  }

  // Note: This is a simplified version. Full implementation would need to
  // copy all data from fs_ to fsOut
  fsOut.release();
}

void ParametersBase::raiseError(const std::string& message) const {
  throw std::runtime_error("[" + name_ + " Reader]: " + message);
}

// ============================================================================
// CameraParameters Implementation
// ============================================================================

CameraParameters::CameraParameters(const std::string& yamlFile)
    : ParametersBase(yamlFile, "CameraConfig") {}

std::string CameraParameters::getImageFolder() const {
  std::string folder;
  fs_["image_folder"] >> folder;
  if (folder.empty()) {
    raiseError("image_folder field is missing or empty");
  }
  return folder;
}

void CameraParameters::setImageFolder(const std::string& folder) {
  if (folder.empty()) {
    raiseError("image_folder cannot be empty");
  }
  // Note: Setting values in cv::FileStorage requires rewriting
}

std::pair<CameraModel, std::vector<double>> CameraParameters::getIntrinsics()
    const {
  std::string modelStr;
  fs_["camera_model"] >> modelStr;
  if (modelStr.empty()) {
    raiseError("camera_model field is missing");
  }

  CameraModel model = stringToCameraModel(modelStr);

  std::vector<double> intrinsics;
  cv::FileNode intrinsicsNode = fs_["intrinsics"];
  if (intrinsicsNode.empty()) {
    raiseError("intrinsics field is missing");
  }

  for (const auto& value : intrinsicsNode) {
    intrinsics.push_back(static_cast<double>(value));
  }

  checkIntrinsics(model, intrinsics);

  return {model, intrinsics};
}

void CameraParameters::setIntrinsics(CameraModel model,
                                     const std::vector<double>& intrinsics) {
  checkIntrinsics(model, intrinsics);
  // Note: Setting values requires rewriting the file
}

std::pair<DistortionModel, std::vector<double>>
CameraParameters::getDistortion() const {
  std::string modelStr;
  fs_["distortion_model"] >> modelStr;
  if (modelStr.empty()) {
    raiseError("distortion_model field is missing");
  }

  DistortionModel model = stringToDistortionModel(modelStr);

  std::vector<double> coeffs;
  cv::FileNode coeffsNode = fs_["distortion_coeffs"];
  if (!coeffsNode.empty()) {
    for (const auto& value : coeffsNode) {
      coeffs.push_back(static_cast<double>(value));
    }
  }

  checkDistortion(model, coeffs);

  return {model, coeffs};
}

void CameraParameters::setDistortion(DistortionModel model,
                                     const std::vector<double>& coeffs) {
  checkDistortion(model, coeffs);
  // Note: Setting values requires rewriting the file
}

Eigen::Vector2i CameraParameters::getResolution() const {
  std::vector<int> res;
  cv::FileNode resNode = fs_["resolution"];
  if (resNode.empty()) {
    raiseError("resolution field is missing");
  }

  for (const auto& value : resNode) {
    res.push_back(static_cast<int>(value));
  }

  if (res.size() != 2) {
    raiseError("resolution must have exactly 2 values");
  }

  Eigen::Vector2i resolution(res[0], res[1]);
  checkResolution(resolution);

  return resolution;
}

void CameraParameters::setResolution(const Eigen::Vector2i& resolution) {
  checkResolution(resolution);
  // Note: Setting values requires rewriting the file
}

double CameraParameters::getLineDelay() const {
  double lineDelay = 0.0;
  fs_["line_delay"] >> lineDelay;
  return lineDelay;
}

void CameraParameters::setLineDelay(double lineDelay) {
  // Note: Setting values requires rewriting the file
}

void CameraParameters::checkIntrinsics(
    CameraModel model, const std::vector<double>& intrinsics) const {
  switch (model) {
    case CameraModel::Pinhole:
      if (intrinsics.size() != 4) {
        raiseError(
            "Pinhole camera model requires 4 intrinsics [fu, fv, pu, pv], "
            "got " +
            std::to_string(intrinsics.size()));
      }
      if (intrinsics[0] <= 0.0 || intrinsics[1] <= 0.0) {
        raiseError("Invalid focal lengths");
      }
      if (intrinsics[2] < 0.0 || intrinsics[3] < 0.0) {
        raiseError("Invalid principal point");
      }
      break;

    case CameraModel::Omni:
      if (intrinsics.size() != 5) {
        raiseError(
            "Omni camera model requires 5 intrinsics [xi, fu, fv, pu, pv], "
            "got " +
            std::to_string(intrinsics.size()));
      }
      if (intrinsics[0] < 0.0) {
        raiseError("Invalid xi parameter (must be >= 0)");
      }
      if (intrinsics[1] <= 0.0 || intrinsics[2] <= 0.0) {
        raiseError("Invalid focal lengths");
      }
      if (intrinsics[3] < 0.0 || intrinsics[4] < 0.0) {
        raiseError("Invalid principal point");
      }
      break;

    case CameraModel::EUCM:
    case CameraModel::DS:
      if (intrinsics.size() != 6) {
        std::string modelName = (model == CameraModel::EUCM) ? "EUCM" : "DS";
        raiseError(modelName +
                   " camera model requires 6 intrinsics [alpha, beta, fu, fv, "
                   "pu, pv], got " +
                   std::to_string(intrinsics.size()));
      }
      if (intrinsics[0] < 0.0 || intrinsics[0] >= 1.0) {
        raiseError("Invalid alpha parameter (must be in [0, 1))");
      }
      if (intrinsics[1] < 0.0) {
        raiseError("Invalid beta parameter (must be >= 0)");
      }
      if (intrinsics[2] <= 0.0 || intrinsics[3] <= 0.0) {
        raiseError("Invalid focal lengths");
      }
      if (intrinsics[4] < 0.0 || intrinsics[5] < 0.0) {
        raiseError("Invalid principal point");
      }
      break;
  }
}

void CameraParameters::checkDistortion(
    DistortionModel model, const std::vector<double>& coeffs) const {
  size_t expectedSize = 0;
  switch (model) {
    case DistortionModel::RadTan:
    case DistortionModel::Equidistant:
      expectedSize = 4;
      break;
    case DistortionModel::FOV:
      expectedSize = 1;
      break;
    case DistortionModel::None:
      expectedSize = 0;
      break;
  }

  if (coeffs.size() != expectedSize) {
    raiseError("Distortion model " + distortionModelToString(model) +
               " requires " + std::to_string(expectedSize) +
               " coefficients, got " + std::to_string(coeffs.size()));
  }
}

void CameraParameters::checkResolution(
    const Eigen::Vector2i& resolution) const {
  if (resolution.x() <= 0 || resolution.y() <= 0) {
    raiseError("Invalid resolution");
  }
}

void CameraParameters::printDetails(std::ostream& os) const {
  auto [model, intrinsics] = getIntrinsics();
  auto [distModel, distCoeffs] = getDistortion();
  auto resolution = getResolution();

  os << "  Camera model: " << cameraModelToString(model) << std::endl;

  if (model == CameraModel::Pinhole) {
    os << "  Focal length: [" << intrinsics[0] << ", " << intrinsics[1] << "]"
       << std::endl;
    os << "  Principal point: [" << intrinsics[2] << ", " << intrinsics[3]
       << "]" << std::endl;
  } else if (model == CameraModel::Omni) {
    os << "  Omni xi: " << intrinsics[0] << std::endl;
    os << "  Focal length: [" << intrinsics[1] << ", " << intrinsics[2] << "]"
       << std::endl;
    os << "  Principal point: [" << intrinsics[3] << ", " << intrinsics[4]
       << "]" << std::endl;
  } else if (model == CameraModel::EUCM) {
    os << "  EUCM alpha: " << intrinsics[0] << std::endl;
    os << "  EUCM beta: " << intrinsics[1] << std::endl;
    os << "  Focal length: [" << intrinsics[2] << ", " << intrinsics[3] << "]"
       << std::endl;
    os << "  Principal point: [" << intrinsics[4] << ", " << intrinsics[5]
       << "]" << std::endl;
  } else if (model == CameraModel::DS) {
    os << "  DS xi: " << intrinsics[0] << std::endl;
    os << "  DS alpha: " << intrinsics[1] << std::endl;
    os << "  Focal length: [" << intrinsics[2] << ", " << intrinsics[3] << "]"
       << std::endl;
    os << "  Principal point: [" << intrinsics[4] << ", " << intrinsics[5]
       << "]" << std::endl;
  }

  os << "  Distortion model: " << distortionModelToString(distModel)
     << std::endl;
  os << "  Distortion coefficients: [";
  for (size_t i = 0; i < distCoeffs.size(); ++i) {
    if (i > 0) os << ", ";
    os << distCoeffs[i];
  }
  os << "]" << std::endl;
  os << "  Resolution: [" << resolution.x() << ", " << resolution.y() << "]"
     << std::endl;
}

// ============================================================================
// ImuParameters Implementation
// ============================================================================

ImuParameters::ImuParameters(const std::string& yamlFile)
    : ParametersBase(yamlFile, "ImuConfig") {}

std::string ImuParameters::getCsvFile() const {
  std::string csvFile;
  fs_["csv_file"] >> csvFile;
  if (csvFile.empty()) {
    raiseError("csv_file field is missing or empty");
  }
  return csvFile;
}

void ImuParameters::setCsvFile(const std::string& file) {
  if (file.empty()) {
    raiseError("csv_file cannot be empty");
  }
  // Note: Setting values requires rewriting the file
}

double ImuParameters::getUpdateRate() const {
  double updateRate = 0.0;
  fs_["update_rate"] >> updateRate;
  if (updateRate <= 0.0) {
    raiseError("update_rate field is missing or invalid");
  }
  checkUpdateRate(updateRate);
  return updateRate;
}

void ImuParameters::setUpdateRate(double updateRate) {
  checkUpdateRate(updateRate);
  // Note: Setting values requires rewriting the file
}

std::tuple<double, double, double> ImuParameters::getAccelerometerStatistics()
    const {
  double noiseDensity = 0.0;
  double randomWalk = 0.0;

  fs_["accelerometer_noise_density"] >> noiseDensity;
  fs_["accelerometer_random_walk"] >> randomWalk;

  checkAccelerometerStatistics(noiseDensity, randomWalk);

  double updateRate = getUpdateRate();
  double discreteNoise = noiseDensity / std::sqrt(1.0 / updateRate);

  return {discreteNoise, randomWalk, noiseDensity};
}

void ImuParameters::setAccelerometerStatistics(double noiseDensity,
                                               double randomWalk) {
  checkAccelerometerStatistics(noiseDensity, randomWalk);
  // Note: Setting values requires rewriting the file
}

std::tuple<double, double, double> ImuParameters::getGyroscopeStatistics()
    const {
  double noiseDensity = 0.0;
  double randomWalk = 0.0;

  fs_["gyroscope_noise_density"] >> noiseDensity;
  fs_["gyroscope_random_walk"] >> randomWalk;

  checkGyroscopeStatistics(noiseDensity, randomWalk);

  double updateRate = getUpdateRate();
  double discreteNoise = noiseDensity / std::sqrt(1.0 / updateRate);

  return {discreteNoise, randomWalk, noiseDensity};
}

void ImuParameters::setGyroscopeStatistics(double noiseDensity,
                                           double randomWalk) {
  checkGyroscopeStatistics(noiseDensity, randomWalk);
  // Note: Setting values requires rewriting the file
}

void ImuParameters::checkUpdateRate(double updateRate) const {
  if (updateRate <= 0.0) {
    raiseError("Invalid update_rate (must be > 0)");
  }
}

void ImuParameters::checkAccelerometerStatistics(double noiseDensity,
                                                 double randomWalk) const {
  if (noiseDensity <= 0.0) {
    raiseError("Invalid accelerometer_noise_density (must be > 0)");
  }
  if (randomWalk <= 0.0) {
    raiseError("Invalid accelerometer_random_walk (must be > 0)");
  }
}

void ImuParameters::checkGyroscopeStatistics(double noiseDensity,
                                             double randomWalk) const {
  if (noiseDensity <= 0.0) {
    raiseError("Invalid gyroscope_noise_density (must be > 0)");
  }
  if (randomWalk <= 0.0) {
    raiseError("Invalid gyroscope_random_walk (must be > 0)");
  }
}

void ImuParameters::printDetails(std::ostream& os) const {
  double updateRate = getUpdateRate();
  auto [accelDiscrete, accelRandomWalk, accelContinuous] =
      getAccelerometerStatistics();
  auto [gyroDiscrete, gyroRandomWalk, gyroContinuous] =
      getGyroscopeStatistics();

  os << "  Update rate: " << updateRate << " Hz" << std::endl;
  os << "  Accelerometer:" << std::endl;
  os << "    Noise density: " << accelContinuous << std::endl;
  os << "    Noise density (discrete): " << accelDiscrete << std::endl;
  os << "    Random walk: " << accelRandomWalk << std::endl;
  os << "  Gyroscope:" << std::endl;
  os << "    Noise density: " << gyroContinuous << std::endl;
  os << "    Noise density (discrete): " << gyroDiscrete << std::endl;
  os << "    Random walk: " << gyroRandomWalk << std::endl;
}

// ============================================================================
// CalibrationTargetParameters Implementation
// ============================================================================

CalibrationTargetParameters::CalibrationTargetParameters(
    const std::string& yamlFile)
    : ParametersBase(yamlFile, "CalibrationTargetConfig") {}

TargetType CalibrationTargetParameters::getTargetType() const {
  std::string typeStr;
  fs_["target_type"] >> typeStr;
  if (typeStr.empty()) {
    raiseError("target_type field is missing");
  }

  TargetType type = stringToTargetType(typeStr);
  checkTargetType(type);

  return type;
}

CalibrationTargetParameters::CheckerboardParams
CalibrationTargetParameters::getCheckerboardParams() const {
  CheckerboardParams params;

  fs_["targetRows"] >> params.rows;
  fs_["targetCols"] >> params.cols;
  fs_["rowSpacingMeters"] >> params.rowSpacing;
  fs_["colSpacingMeters"] >> params.colSpacing;

  if (params.rows < 3 || params.cols < 3) {
    raiseError("Invalid checkerboard dimensions (must be >= 3x3)");
  }
  if (params.rowSpacing <= 0.0 || params.colSpacing <= 0.0) {
    raiseError("Invalid checkerboard spacing (must be > 0)");
  }

  return params;
}

CalibrationTargetParameters::CirclegridParams
CalibrationTargetParameters::getCirclegridParams() const {
  CirclegridParams params;

  fs_["targetRows"] >> params.rows;
  fs_["targetCols"] >> params.cols;
  fs_["spacingMeters"] >> params.spacing;

  int asymmetric = 0;
  fs_["asymmetricGrid"] >> asymmetric;
  params.asymmetric = (asymmetric != 0);

  if (params.rows < 3 || params.cols < 3) {
    raiseError("Invalid circlegrid dimensions (must be >= 3x3)");
  }
  if (params.spacing <= 0.0) {
    raiseError("Invalid circlegrid spacing (must be > 0)");
  }

  return params;
}

CalibrationTargetParameters::AprilgridParams
CalibrationTargetParameters::getAprilgridParams() const {
  AprilgridParams params;

  fs_["tagRows"] >> params.tagRows;
  fs_["tagCols"] >> params.tagCols;
  fs_["tagSize"] >> params.tagSize;
  fs_["tagSpacing"] >> params.tagSpacing;

  if (params.tagRows < 1 || params.tagCols < 1) {
    raiseError("Invalid aprilgrid dimensions (must be >= 1x1)");
  }
  if (params.tagSize <= 0.0) {
    raiseError("Invalid tag size (must be > 0)");
  }
  if (params.tagSpacing <= 0.0) {
    raiseError("Invalid tag spacing (must be > 0)");
  }

  return params;
}

void CalibrationTargetParameters::checkTargetType(TargetType type) const {
  // All enum values are valid
}

void CalibrationTargetParameters::printDetails(std::ostream& os) const {
  TargetType type = getTargetType();
  os << "  Type: " << targetTypeToString(type) << std::endl;

  if (type == TargetType::Checkerboard) {
    auto params = getCheckerboardParams();
    os << "  Rows:" << std::endl;
    os << "    Count: " << params.rows << std::endl;
    os << "    Distance: " << params.rowSpacing << " m" << std::endl;
    os << "  Cols:" << std::endl;
    os << "    Count: " << params.cols << std::endl;
    os << "    Distance: " << params.colSpacing << " m" << std::endl;
  } else if (type == TargetType::Aprilgrid) {
    auto params = getAprilgridParams();
    os << "  Tags:" << std::endl;
    os << "    Rows: " << params.tagRows << std::endl;
    os << "    Cols: " << params.tagCols << std::endl;
    os << "    Size: " << params.tagSize << " m" << std::endl;
    os << "    Spacing: " << (params.tagSize * params.tagSpacing) << " m"
       << std::endl;
  } else if (type == TargetType::Circlegrid) {
    auto params = getCirclegridParams();
    os << "  Circles:" << std::endl;
    os << "    Rows: " << params.rows << std::endl;
    os << "    Cols: " << params.cols << std::endl;
    os << "    Spacing: " << params.spacing << " m" << std::endl;
    os << "    Asymmetric: " << (params.asymmetric ? "yes" : "no") << std::endl;
  }
}

// ============================================================================
// CameraChainParameters Implementation
// ============================================================================

CameraChainParameters::CameraChainParameters(const std::string& yamlFile)
    : ParametersBase(yamlFile, "CameraChainParameters") {
  // Count number of cameras
  size_t camCount = 0;
  while (true) {
    std::string camKey = "cam" + std::to_string(camCount);
    cv::FileNode node = fs_[camKey];
    if (node.empty()) {
      break;
    }
    camCount++;
  }

  cameraCache_.resize(camCount);
}

size_t CameraChainParameters::numCameras() const { return cameraCache_.size(); }

std::shared_ptr<CameraParameters> CameraChainParameters::getCameraParameters(
    size_t camIdx) const {
  checkCameraIndex(camIdx);

  if (!cameraCache_[camIdx]) {
    // Create a temporary camera parameters object
    // Note: This requires extracting the cam{N} subtree from the main file
    // For now, we'll return nullptr and this needs to be implemented properly
    raiseError(
        "getCameraParameters not fully implemented - requires YAML subtree "
        "extraction");
  }

  return cameraCache_[camIdx];
}

Eigen::Matrix4d CameraChainParameters::getExtrinsicsLastCamToHere(
    size_t camIdx) const {
  checkCameraIndex(camIdx);

  if (camIdx == 0) {
    raiseError("Cannot get extrinsics for cam0 (base camera)");
  }

  std::string camKey = "cam" + std::to_string(camIdx);
  cv::FileNode camNode = fs_[camKey];
  cv::FileNode tNode = camNode["T_cn_cnm1"];

  if (tNode.empty()) {
    raiseError("T_cn_cnm1 field is missing for " + camKey);
  }

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  int row = 0;
  for (const auto& rowNode : tNode) {
    int col = 0;
    for (const auto& value : rowNode) {
      T(row, col++) = static_cast<double>(value);
    }
    row++;
  }

  return T;
}

void CameraChainParameters::setExtrinsicsLastCamToHere(
    size_t camIdx, const Eigen::Matrix4d& T) {
  checkCameraIndex(camIdx);
  if (camIdx == 0) {
    raiseError("Cannot set extrinsics for cam0 (base camera)");
  }
  // Note: Setting values requires rewriting the file
}

Eigen::Matrix4d CameraChainParameters::getExtrinsicsImuToCam(
    size_t camIdx) const {
  checkCameraIndex(camIdx);

  std::string camKey = "cam" + std::to_string(camIdx);
  cv::FileNode camNode = fs_[camKey];
  cv::FileNode tNode = camNode["T_cam_imu"];

  if (tNode.empty()) {
    raiseError("T_cam_imu field is missing for " + camKey);
  }

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  int row = 0;
  for (const auto& rowNode : tNode) {
    int col = 0;
    for (const auto& value : rowNode) {
      T(row, col++) = static_cast<double>(value);
    }
    row++;
  }

  return T;
}

void CameraChainParameters::setExtrinsicsImuToCam(size_t camIdx,
                                                  const Eigen::Matrix4d& T) {
  checkCameraIndex(camIdx);
  // Note: Setting values requires rewriting the file
}

double CameraChainParameters::getTimeshiftCamImu(size_t camIdx) const {
  checkCameraIndex(camIdx);

  std::string camKey = "cam" + std::to_string(camIdx);
  cv::FileNode camNode = fs_[camKey];

  double timeshift = 0.0;
  camNode["timeshift_cam_imu"] >> timeshift;

  return timeshift;
}

void CameraChainParameters::setTimeshiftCamImu(size_t camIdx,
                                               double timeshift) {
  checkCameraIndex(camIdx);
  // Note: Setting values requires rewriting the file
}

std::vector<int> CameraChainParameters::getCamOverlaps(size_t camIdx) const {
  checkCameraIndex(camIdx);

  std::string camKey = "cam" + std::to_string(camIdx);
  cv::FileNode camNode = fs_[camKey];
  cv::FileNode overlapsNode = camNode["cam_overlaps"];

  std::vector<int> overlaps;
  if (!overlapsNode.empty()) {
    for (const auto& value : overlapsNode) {
      overlaps.push_back(static_cast<int>(value));
    }
  }

  return overlaps;
}

void CameraChainParameters::setCamOverlaps(size_t camIdx,
                                           const std::vector<int>& overlaps) {
  checkCameraIndex(camIdx);
  // Note: Setting values requires rewriting the file
}

void CameraChainParameters::checkCameraIndex(size_t camIdx) const {
  if (camIdx >= numCameras()) {
    raiseError("Camera index " + std::to_string(camIdx) +
               " out of range (max: " + std::to_string(numCameras() - 1) + ")");
  }
}

void CameraChainParameters::printDetails(std::ostream& os) const {
  for (size_t i = 0; i < numCameras(); ++i) {
    os << "Camera chain - cam" << i << ":" << std::endl;

    // Print extrinsics if available
    if (i > 0) {
      try {
        Eigen::Matrix4d T = getExtrinsicsLastCamToHere(i);
        os << "  Baseline (T_cn_cnm1):" << std::endl;
        os << T << std::endl;
      } catch (...) {
        os << "  Baseline: no data available" << std::endl;
      }
    }

    try {
      Eigen::Matrix4d T = getExtrinsicsImuToCam(i);
      os << "  T_cam_imu:" << std::endl;
      os << T << std::endl;
    } catch (...) {
      os << "  T_cam_imu: no data available" << std::endl;
    }

    double timeshift = getTimeshiftCamImu(i);
    os << "  Timeshift cam-imu: " << timeshift << " s" << std::endl;
  }
}

// ============================================================================
// AslamCamera Implementation
// ============================================================================

std::shared_ptr<AslamCamera> AslamCamera::fromParameters(
    CameraModel cameraModel, const std::vector<double>& intrinsics,
    DistortionModel distModel, const std::vector<double>& distCoeffs,
    const Eigen::Vector2i& resolution) {
  auto camera = std::make_shared<AslamCamera>();

  // Pinhole camera model
  if (cameraModel == CameraModel::Pinhole) {
    double fu = intrinsics[0];
    double fv = intrinsics[1];
    double pu = intrinsics[2];
    double pv = intrinsics[3];

    if (distModel == DistortionModel::RadTan) {
      // RadTan distortion
      aslam::cameras::RadialTangentialDistortion dist(
          distCoeffs[0], distCoeffs[1], distCoeffs[2], distCoeffs[3]);
      aslam::cameras::PinholeProjection<
          aslam::cameras::RadialTangentialDistortion>
          proj(fu, fv, pu, pv, resolution.x(), resolution.y(), dist);
      camera->geometry_ = std::make_shared<aslam::cameras::CameraGeometry<
          aslam::cameras::PinholeProjection<
              aslam::cameras::RadialTangentialDistortion>,
          aslam::cameras::GlobalShutter, aslam::cameras::NoMask>>(proj);

    } else if (distModel == DistortionModel::Equidistant) {
      // Equidistant distortion
      aslam::cameras::EquidistantDistortion dist(distCoeffs[0], distCoeffs[1],
                                                 distCoeffs[2], distCoeffs[3]);
      aslam::cameras::PinholeProjection<aslam::cameras::EquidistantDistortion>
          proj(fu, fv, pu, pv, resolution.x(), resolution.y(), dist);
      camera->geometry_ = std::make_shared<aslam::cameras::CameraGeometry<
          aslam::cameras::PinholeProjection<
              aslam::cameras::EquidistantDistortion>,
          aslam::cameras::GlobalShutter, aslam::cameras::NoMask>>(proj);

    } else if (distModel == DistortionModel::FOV) {
      // FOV distortion
      aslam::cameras::FovDistortion dist(distCoeffs[0]);
      aslam::cameras::PinholeProjection<aslam::cameras::FovDistortion> proj(
          fu, fv, pu, pv, resolution.x(), resolution.y(), dist);
      camera->geometry_ = std::make_shared<aslam::cameras::CameraGeometry<
          aslam::cameras::PinholeProjection<aslam::cameras::FovDistortion>,
          aslam::cameras::GlobalShutter, aslam::cameras::NoMask>>(proj);

    } else if (distModel == DistortionModel::None) {
      // No distortion
      aslam::cameras::PinholeProjection<aslam::cameras::NoDistortion> proj(
          fu, fv, pu, pv, resolution.x(), resolution.y());
      camera->geometry_ = std::make_shared<aslam::cameras::CameraGeometry<
          aslam::cameras::PinholeProjection<aslam::cameras::NoDistortion>,
          aslam::cameras::GlobalShutter, aslam::cameras::NoMask>>(proj);

    } else {
      throw std::runtime_error(
          "Pinhole camera model does not support distortion model: " +
          distortionModelToString(distModel));
    }
  }
  // Omni camera model
  else if (cameraModel == CameraModel::Omni) {
    double xi = intrinsics[0];
    double fu = intrinsics[1];
    double fv = intrinsics[2];
    double pu = intrinsics[3];
    double pv = intrinsics[4];

    if (distModel == DistortionModel::RadTan) {
      // RadTan distortion
      aslam::cameras::RadialTangentialDistortion dist(
          distCoeffs[0], distCoeffs[1], distCoeffs[2], distCoeffs[3]);
      aslam::cameras::OmniProjection<aslam::cameras::RadialTangentialDistortion>
          proj(xi, fu, fv, pu, pv, resolution.x(), resolution.y(), dist);
      camera->geometry_ = std::make_shared<aslam::cameras::CameraGeometry<
          aslam::cameras::OmniProjection<
              aslam::cameras::RadialTangentialDistortion>,
          aslam::cameras::GlobalShutter, aslam::cameras::NoMask>>(proj);

    } else if (distModel == DistortionModel::Equidistant) {
      throw std::runtime_error(
          "Omni with equidistant distortion model not yet supported!");

    } else if (distModel == DistortionModel::None) {
      // No distortion
      aslam::cameras::OmniProjection<aslam::cameras::NoDistortion> proj(
          xi, fu, fv, pu, pv, resolution.x(), resolution.y());
      camera->geometry_ = std::make_shared<aslam::cameras::CameraGeometry<
          aslam::cameras::OmniProjection<aslam::cameras::NoDistortion>,
          aslam::cameras::GlobalShutter, aslam::cameras::NoMask>>(proj);

    } else {
      throw std::runtime_error(
          "Omni camera model does not support distortion model: " +
          distortionModelToString(distModel));
    }
  }
  // Extended Unified Camera Model (EUCM)
  else if (cameraModel == CameraModel::EUCM) {
    double alpha = intrinsics[0];
    double beta = intrinsics[1];
    double fu = intrinsics[2];
    double fv = intrinsics[3];
    double pu = intrinsics[4];
    double pv = intrinsics[5];

    if (distModel == DistortionModel::None) {
      // No distortion
      aslam::cameras::ExtendedUnifiedProjection<aslam::cameras::NoDistortion>
          proj(alpha, beta, fu, fv, pu, pv, resolution.x(), resolution.y());
      camera->geometry_ = std::make_shared<aslam::cameras::CameraGeometry<
          aslam::cameras::ExtendedUnifiedProjection<
              aslam::cameras::NoDistortion>,
          aslam::cameras::GlobalShutter, aslam::cameras::NoMask>>(proj);

    } else {
      throw std::runtime_error(
          "EUCM camera model does not support distortion model: " +
          distortionModelToString(distModel));
    }
  }
  // Double Sphere (DS) camera model
  else if (cameraModel == CameraModel::DS) {
    double xi = intrinsics[0];
    double alpha = intrinsics[1];
    double fu = intrinsics[2];
    double fv = intrinsics[3];
    double pu = intrinsics[4];
    double pv = intrinsics[5];

    if (distModel == DistortionModel::None) {
      // No distortion
      aslam::cameras::DoubleSphereProjection<aslam::cameras::NoDistortion> proj(
          xi, alpha, fu, fv, pu, pv, resolution.x(), resolution.y());
      camera->geometry_ = std::make_shared<aslam::cameras::CameraGeometry<
          aslam::cameras::DoubleSphereProjection<aslam::cameras::NoDistortion>,
          aslam::cameras::GlobalShutter, aslam::cameras::NoMask>>(proj);

    } else {
      throw std::runtime_error(
          "DS camera model does not support distortion model: " +
          distortionModelToString(distModel));
    }
  } else {
    throw std::runtime_error("Unknown camera model: " +
                             cameraModelToString(cameraModel));
  }

  return camera;
}

std::shared_ptr<AslamCamera> AslamCamera::fromParameters(
    const CameraParameters& params) {
  auto [cameraModel, intrinsics] = params.getIntrinsics();
  auto [distModel, distCoeffs] = params.getDistortion();
  auto resolution = params.getResolution();

  return fromParameters(cameraModel, intrinsics, distModel, distCoeffs,
                        resolution);
}

}  // namespace kalibr
