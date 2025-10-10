#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <any>
#include <cmath>
#include <iostream>
#include <kalibr_common/ConfigReader.hpp>
#include <print>
#include <stdexcept>

// ASLAM camera includes
#include <aslam/cameras.hpp>
#include <vector>
// ASLAM backend includes
#include <aslam/Frame.hpp>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>
#include <aslam/backend/SimpleReprojectionError.hpp>
#include <sm/kinematics/Transformation.hpp>
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
                               const std::string& name, bool createYaml)
    : yamlFile_(yamlFile), name_(name) {
  if (createYaml) {
    data_.clear();
  } else {
    data_ = this->readYaml();
  }
}

// Helper: check whether a scalar string represents an integer (only digits
// with optional sign)
static bool isIntegerString(const std::string& s) {
  if (s.empty()) return false;
  size_t i = 0;
  if (s[0] == '+' || s[0] == '-') {
    if (s.size() == 1) return false;
    i = 1;
  }
  for (; i < s.size(); ++i) {
    if (!std::isdigit(static_cast<unsigned char>(s[i]))) return false;
  }
  return true;
}

// Helper: check whether a scalar string looks like a floating point number.
// This is a lightweight check (presence of '.' or 'e'/'E') and is not a full
// validator; used to choose parsing strategy.
static bool looksLikeFloat(const std::string& s) {
  for (char c : s) {
    if (c == '.' || c == 'e' || c == 'E') return true;
  }
  return false;
}

// Parse a scalar node into std::any (int, double, bool or string). This
// function avoids relying on YAML::Node::as<T>() to control flow and instead
// uses deterministic string inspection plus minimal std::stod/stoll guarded by
// try/catch for safe conversion.
static std::any parseScalarNode(const YAML::Node& node) {
  const std::string s = node.Scalar();
  // boolean
  std::string lower = s;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
  if (lower == "true") return true;
  if (lower == "false") return false;

  // integer
  if (isIntegerString(s)) {
    try {
      long long v = std::stoll(s);
      return static_cast<int>(v);
    } catch (...) {
      // fall through to try double/string
    }
  }

  // float-ish
  if (looksLikeFloat(s)) {
    try {
      double d = std::stod(s);
      return d;
    } catch (...) {
      // fall through to string
    }
  }

  // fallback to string
  return s;
}

// Parse a sequence node into either vector<int>, vector<double> or
// vector<string> depending on element contents.
static std::any parseSequenceNode(const YAML::Node& node) {
  std::vector<int> vi;
  std::vector<double> vd;
  std::vector<std::string> vs;

  bool allInt = true;
  bool allDouble = true;

  for (const auto& el : node) {
    if (!el.IsScalar()) {
      // Non-scalar element: fall back to vector<string> of dumped nodes
      vs.push_back(YAML::Dump(el));
      allInt = false;
      allDouble = false;
      continue;
    }
    std::string s = el.Scalar();
    if (isIntegerString(s)) {
      try {
        vi.push_back(static_cast<int>(std::stoll(s)));
      } catch (...) {
        allInt = false;
      }
      try {
        vd.push_back(std::stod(s));
      } catch (...) {
        allDouble = false;
      }
      vs.push_back(s);
    } else if (looksLikeFloat(s)) {
      // treat as double candidate
      try {
        double d = std::stod(s);
        vd.push_back(d);
      } catch (...) {
        allDouble = false;
      }
      allInt = false;
      vs.push_back(s);
    } else {
      // treat as string
      vs.push_back(s);
      allInt = false;
      allDouble = false;
    }
  }

  if (allInt && !vi.empty()) return vi;
  if (allDouble && !vd.empty()) return vd;
  return vs;
}

// Recursively parse any YAML node into std::any using the above helpers.
static std::any parseNodeToAny(const YAML::Node& node) {
  if (!node) return std::any();
  if (node.IsScalar()) return parseScalarNode(node);
  if (node.IsSequence()) return parseSequenceNode(node);
  if (node.IsMap()) {
    ParametersBase::DictType nested;
    for (auto it = node.begin(); it != node.end(); ++it) {
      std::string key = it->first.as<std::string>();
      nested[key] = parseNodeToAny(it->second);
    }
    return nested;
  }
  // fallback: return string dump
  return YAML::Dump(node);
}

ParametersBase::DictType ParametersBase::readYaml() {
  ParametersBase::DictType data;
  try {
    YAML::Node root = YAML::LoadFile(yamlFile_);
    if (!root) {
      raiseError("Could not open configuration file: " + yamlFile_);
    }

    for (auto it = root.begin(); it != root.end(); ++it) {
      const std::string key = it->first.as<std::string>();
      const YAML::Node& value = it->second;
      data[key] = parseNodeToAny(value);
    }
  } catch (const std::exception& e) {
    raiseError("Could not read configuration from " + yamlFile_ + ": " +
               e.what());
  } catch (...) {
    raiseError("Could not read configuration from " + yamlFile_ +
               ": unknown error");
  }
  return data;
}

void ParametersBase::writeYaml(const std::string& filename) {
  std::string filename_ = filename;
  if (filename_.empty()) {
    filename_ = yamlFile_;
  }
  try {
    YAML::Node root;
    for (const auto& [key, value] : data_) {
      if (value.type() == typeid(int)) {
        root[key] = std::any_cast<int>(value);
      } else if (value.type() == typeid(double)) {
        root[key] = std::any_cast<double>(value);
      } else if (value.type() == typeid(bool)) {
        root[key] = std::any_cast<bool>(value);
      } else if (value.type() == typeid(std::string)) {
        root[key] = std::any_cast<std::string>(value);
      } else if (value.type() == typeid(std::vector<int>)) {
        root[key] = std::any_cast<std::vector<int>>(value);
      } else if (value.type() == typeid(std::vector<double>)) {
        root[key] = std::any_cast<std::vector<double>>(value);
      } else if (value.type() == typeid(std::vector<std::string>)) {
        root[key] = std::any_cast<std::vector<std::string>>(value);
      } else if (value.type() == typeid(DictType)) {
        YAML::Node nested;
        DictType nestedDict = std::any_cast<DictType>(value);
        for (const auto& [nestedKey, nestedValue] : nestedDict) {
          if (nestedValue.type() == typeid(int)) {
            nested[nestedKey] = std::any_cast<int>(nestedValue);
          } else if (nestedValue.type() == typeid(double)) {
            nested[nestedKey] = std::any_cast<double>(nestedValue);
          } else if (nestedValue.type() == typeid(bool)) {
            nested[nestedKey] = std::any_cast<bool>(nestedValue);
          } else if (nestedValue.type() == typeid(std::string)) {
            nested[nestedKey] = std::any_cast<std::string>(nestedValue);
          } else if (nestedValue.type() == typeid(std::vector<int>)) {
            nested[nestedKey] = std::any_cast<std::vector<int>>(nestedValue);
          } else if (nestedValue.type() == typeid(std::vector<double>)) {
            nested[nestedKey] = std::any_cast<std::vector<double>>(nestedValue);
          } else if (nestedValue.type() == typeid(std::vector<std::string>)) {
            nested[nestedKey] =
                std::any_cast<std::vector<std::string>>(nestedValue);
          } else {
            std::println("Unsupported data type for key:  {}", nestedKey);
            std::println("Type: {}", nestedValue.type().name());
            raiseError("Unsupported data type for key: " + nestedKey);
          }
        }
        root[key] = nested;
      } else {
        std::println("Unsupported data type for key:  {}", key);
        std::println("Type: {}", value.type().name());
        raiseError("Unsupported data type for key: " + key);
      }
    }

    // Emit YAML to file
    std::ofstream fout(filename_);
    if (!fout.is_open()) {
      raiseError("Could not write configuration to " + filename_);
    }
    fout << YAML::Dump(root);
    fout.close();
  } catch (const std::exception& e) {
    raiseError(std::string("Could not write configuration to ") + filename_ +
               ": " + e.what());
  } catch (...) {
    raiseError("Could not write configuration to " + filename_);
  }
}

ParametersBase::DictType ParametersBase::getYamlDict() const { return data_; }

void ParametersBase::setYamlDict(const DictType& dict) { data_ = dict; }

void ParametersBase::raiseError(const std::string& message) const {
  throw std::runtime_error("[" + name_ + " Reader]: " + message);
}

// ============================================================================
// CameraParameters Implementation
// ============================================================================

CameraParameters::CameraParameters(const std::string& yamlFile, bool createYaml)
    : ParametersBase(yamlFile, "CameraConfig", createYaml) {}

std::string CameraParameters::getImageFolder() const {
  std::string folder;
  folder = std::any_cast<std::string>(data_.at("image_folder"));
  return folder;
}

void CameraParameters::setImageFolder(const std::string& folder) {
  if (folder.empty()) {
    raiseError("image_folder cannot be empty");
  }
  data_["image_folder"] = folder;
}

std::pair<CameraModel, std::vector<double>> CameraParameters::getIntrinsics()
    const {
  std::string modelStr;
  modelStr = std::any_cast<std::string>(data_.at("camera_model"));
  if (modelStr.empty()) {
    raiseError("camera_model field is missing");
  }

  CameraModel model = stringToCameraModel(modelStr);

  std::vector<double> intrinsics;
  intrinsics = std::any_cast<std::vector<double>>(data_.at("intrinsics"));
  if (intrinsics.empty()) {
    raiseError("intrinsics field is missing");
  }

  checkIntrinsics(model, intrinsics);

  return {model, intrinsics};
}

void CameraParameters::setIntrinsics(CameraModel model,
                                     const std::vector<double>& intrinsics) {
  checkIntrinsics(model, intrinsics);
  data_["camera_model"] = cameraModelToString(model);
  data_["intrinsics"] = intrinsics;
}

std::pair<DistortionModel, std::vector<double>>
CameraParameters::getDistortion() const {
  std::string modelStr;
  modelStr = std::any_cast<std::string>(data_.at("distortion_model"));
  if (modelStr.empty()) {
    raiseError("distortion_model field is missing");
  }

  DistortionModel model = stringToDistortionModel(modelStr);

  std::vector<double> coeffs;
  coeffs = std::any_cast<std::vector<double>>(data_.at("distortion_coeffs"));
  if (coeffs.empty()) {
    raiseError("distortion_coeffs field is missing");
  }

  checkDistortion(model, coeffs);

  return {model, coeffs};
}

void CameraParameters::setDistortion(DistortionModel model,
                                     const std::vector<double>& coeffs) {
  checkDistortion(model, coeffs);
  data_["distortion_model"] = distortionModelToString(model);
  data_["distortion_coeffs"] = coeffs;
}

Eigen::Vector2i CameraParameters::getResolution() const {
  std::vector<int> res;
  res = std::any_cast<std::vector<int>>(data_.at("resolution"));
  if (res.empty()) {
    raiseError("resolution field is missing");
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
  data_["resolution"] = std::vector<int>{resolution(0), resolution(1)};
}

double CameraParameters::getLineDelay() const {
  double lineDelay = 0.0;
  if (data_.find("line_delay") != data_.end()) {
    lineDelay = std::any_cast<double>(data_.at("line_delay"));
  }
  return lineDelay;
}

void CameraParameters::setLineDelay(double lineDelay) {
  data_["line_delay"] = lineDelay;
}

double CameraParameters::getReprojectionSigma() const {
  double sigma = 1.0;
  if (data_.find("reprojection_sigma") != data_.end()) {
    sigma = std::any_cast<double>(data_.at("reprojection_sigma"));
  }
  return sigma;
}

void CameraParameters::setReprojectionSigma(double sigma) {
  data_["reprojection_sigma"] = sigma;
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

  std::println(os, "  Camera model: {}", cameraModelToString(model));

  if (model == CameraModel::Pinhole) {
    std::println(os, "  Focal length: [{}, {}]", intrinsics[0], intrinsics[1]);
    std::println(os, "  Principal point: [{}, {}]", intrinsics[2],
                 intrinsics[3]);
  } else if (model == CameraModel::Omni) {
    std::println(os, "  Omni xi: {}", intrinsics[0]);
    std::println(os, "  Focal length: [{}, {}]", intrinsics[1], intrinsics[2]);
    std::println(os, "  Principal point: [{}, {}]", intrinsics[3],
                 intrinsics[4]);
  } else if (model == CameraModel::EUCM) {
    std::println(os, "  EUCM alpha: {}", intrinsics[0]);
    std::println(os, "  EUCM beta: {}", intrinsics[1]);
    std::println(os, "  Focal length: [{}, {}]", intrinsics[2], intrinsics[3]);
    std::println(os, "  Principal point: [{}, {}]", intrinsics[4],
                 intrinsics[5]);
  } else if (model == CameraModel::DS) {
    std::println(os, "  DS xi: {}", intrinsics[0]);
    std::println(os, "  DS alpha: {}", intrinsics[1]);
    std::println(os, "  Focal length: [{}, {}]", intrinsics[2], intrinsics[3]);
    std::println(os, "  Principal point: [{}, {}]", intrinsics[4],
                 intrinsics[5]);
  }

  std::println(os, "  Distortion model: {}",
               distortionModelToString(distModel));

  std::string coeffStr = "[";
  for (size_t i = 0; i < distCoeffs.size(); ++i) {
    if (i > 0) coeffStr += ", ";
    coeffStr += std::format("{}", distCoeffs[i]);
  }
  coeffStr += "]";
  std::println(os, "  Distortion coefficients: {}", coeffStr);

  std::println(os, "  Resolution: [{}, {}]", resolution.x(), resolution.y());
}

// ============================================================================
// ImuParameters Implementation
// ============================================================================

ImuParameters::ImuParameters(const std::string& yamlFile, bool createYaml)
    : ParametersBase(yamlFile, "ImuConfig", createYaml) {}

std::string ImuParameters::getCsvFile() const {
  std::string csvFile;
  csvFile = std::any_cast<std::string>(data_.at("csv_file"));
  if (csvFile.empty()) {
    raiseError("csv_file field is missing or empty");
  }
  return csvFile;
}

void ImuParameters::setCsvFile(const std::string& file) {
  if (file.empty()) {
    raiseError("csv_file cannot be empty");
  }
  data_["csv_file"] = file;
}

double ImuParameters::getUpdateRate() const {
  double updateRate = 0.0;
  updateRate = std::any_cast<double>(data_.at("update_rate"));
  checkUpdateRate(updateRate);
  return updateRate;
}

void ImuParameters::setUpdateRate(double updateRate) {
  checkUpdateRate(updateRate);
  data_["update_rate"] = updateRate;
}

std::tuple<double, double, double> ImuParameters::getAccelerometerStatistics()
    const {
  double noiseDensity = 0.0;
  double randomWalk = 0.0;

  noiseDensity = std::any_cast<double>(data_.at("accelerometer_noise_density"));
  randomWalk = std::any_cast<double>(data_.at("accelerometer_random_walk"));

  checkAccelerometerStatistics(noiseDensity, randomWalk);

  double updateRate = getUpdateRate();
  double discreteNoise = noiseDensity / std::sqrt(1.0 / updateRate);

  return {discreteNoise, randomWalk, noiseDensity};
}

void ImuParameters::setAccelerometerStatistics(double noiseDensity,
                                               double randomWalk) {
  checkAccelerometerStatistics(noiseDensity, randomWalk);
  data_["accelerometer_noise_density"] = noiseDensity;
  data_["accelerometer_random_walk"] = randomWalk;
}

std::tuple<double, double, double> ImuParameters::getGyroscopeStatistics()
    const {
  double noiseDensity = 0.0;
  double randomWalk = 0.0;

  noiseDensity = std::any_cast<double>(data_.at("gyroscope_noise_density"));
  randomWalk = std::any_cast<double>(data_.at("gyroscope_random_walk"));

  checkGyroscopeStatistics(noiseDensity, randomWalk);

  double updateRate = getUpdateRate();
  double discreteNoise = noiseDensity / std::sqrt(1.0 / updateRate);

  return {discreteNoise, randomWalk, noiseDensity};
}

void ImuParameters::setGyroscopeStatistics(double noiseDensity,
                                           double randomWalk) {
  checkGyroscopeStatistics(noiseDensity, randomWalk);
  data_["gyroscope_noise_density"] = noiseDensity;
  data_["gyroscope_random_walk"] = randomWalk;
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

  std::println(os, "  Update rate: {} Hz", updateRate);
  std::println(os, "  Accelerometer:");
  std::println(os, "    Noise density: {}", accelContinuous);
  std::println(os, "    Noise density (discrete): {}", accelDiscrete);
  std::println(os, "    Random walk: {}", accelRandomWalk);
  std::println(os, "  Gyroscope:");
  std::println(os, "    Noise density: {}", gyroContinuous);
  std::println(os, "    Noise density (discrete): {}", gyroDiscrete);
  std::println(os, "    Random walk: {}", gyroRandomWalk);
}

// ============================================================================
// ImuSetParameters Implementation
// ============================================================================

ImuSetParameters::ImuSetParameters(const std::string& yamlFile, bool createYaml)
    : ParametersBase(yamlFile, "ImuSetConfig", createYaml) {
  imuCount_ = 0;
}

void ImuSetParameters::addImuConfig(const ImuParameters& imuConfig,
                                    std::string name) {
  if (name.empty()) {
    name = "imu_" + std::to_string(imuCount_);
  }
  data_[name] = imuConfig.getYamlDict();
  imuCount_++;
}

// ============================================================================
// CalibrationTargetParameters Implementation
// ============================================================================

CalibrationTargetParameters::CalibrationTargetParameters(
    const std::string& yamlFile, bool createYaml)
    : ParametersBase(yamlFile, "CalibrationTargetConfig", createYaml) {}

TargetType CalibrationTargetParameters::getTargetType() const {
  std::string typeStr;
  typeStr = std::any_cast<std::string>(data_.at("target_type"));
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

  params.rows = std::any_cast<int>(data_.at("targetRows"));
  params.cols = std::any_cast<int>(data_.at("targetCols"));
  params.rowSpacing = std::any_cast<double>(data_.at("rowSpacingMeters"));
  params.colSpacing = std::any_cast<double>(data_.at("colSpacingMeters"));

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

  params.rows = std::any_cast<int>(data_.at("targetRows"));
  params.cols = std::any_cast<int>(data_.at("targetCols"));
  params.spacing = std::any_cast<double>(data_.at("spacingMeters"));

  int asymmetric = 0;
  asymmetric = std::any_cast<int>(data_.at("asymmetricGrid"));
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

  params.tagRows = std::any_cast<int>(data_.at("tagRows"));
  params.tagCols = std::any_cast<int>(data_.at("tagCols"));
  params.tagSize = std::any_cast<double>(data_.at("tagSize"));
  params.tagSpacing = std::any_cast<double>(data_.at("tagSpacing"));

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

void CalibrationTargetParameters::checkTargetType(TargetType) const {
  // All enum values are valid
}

void CalibrationTargetParameters::printDetails(std::ostream& os) const {
  TargetType type = getTargetType();
  std::println(os, "  Type: {}", targetTypeToString(type));

  if (type == TargetType::Checkerboard) {
    auto params = getCheckerboardParams();
    std::println(os, "  Rows:");
    std::println(os, "    Count: {}", params.rows);
    std::println(os, "    Distance: {} m", params.rowSpacing);
    std::println(os, "  Cols:");
    std::println(os, "    Count: {}", params.cols);
    std::println(os, "    Distance: {} m", params.colSpacing);
  } else if (type == TargetType::Aprilgrid) {
    auto params = getAprilgridParams();
    std::println(os, "  Tags:");
    std::println(os, "    Rows: {}", params.tagRows);
    std::println(os, "    Cols: {}", params.tagCols);
    std::println(os, "    Size: {} m", params.tagSize);
    std::println(os, "    Spacing: {} m", params.tagSize * params.tagSpacing);
  } else if (type == TargetType::Circlegrid) {
    auto params = getCirclegridParams();
    std::println(os, "  Circles:");
    std::println(os, "    Rows: {}", params.rows);
    std::println(os, "    Cols: {}", params.cols);
    std::println(os, "    Spacing: {} m", params.spacing);
    std::println(os, "    Asymmetric: {}", params.asymmetric ? "yes" : "no");
  }
}

// ============================================================================
// CameraChainParameters Implementation
// ============================================================================

CameraChainParameters::CameraChainParameters(const std::string& yamlFile,
                                             bool createYaml)
    : ParametersBase(yamlFile, "CameraChainParameters", createYaml) {}

size_t CameraChainParameters::numCameras() const { return data_.size(); }

std::shared_ptr<CameraParameters> CameraChainParameters::getCameraParameters(
    size_t camIdx) const {
  checkCameraIndex(camIdx);
  auto param = std::make_shared<CameraParameters>("TEMP_CONFIG", true);
  param->setYamlDict(std::any_cast<ParametersBase::DictType>(
      data_.at("cam" + std::to_string(camIdx))));
  return param;
}

sm::kinematics::Transformation
CameraChainParameters::getExtrinsicsLastCamToHere(size_t camIdx) const {
  checkCameraIndex(camIdx);

  if (camIdx == 0) {
    raiseError(
        "setExtrinsicsLastCamToHere(): can't get extrinsics for first camera "
        "in chain (cam0=base)");
  }
  sm::kinematics::Transformation trafo;
  try {
    auto t_vec = std::any_cast<std::vector<double>>(
        std::any_cast<ParametersBase::DictType>(
            data_.at("cam" + std::to_string(camIdx)))
            .at("T_cn_cnm1"));
    auto t_mat = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(
        t_vec.data());
    trafo = sm::kinematics::Transformation(t_mat);
  } catch (...) {
    raiseError("invalid camera baseline (cam" + std::to_string(camIdx) +
               " in " + yamlFile_ + ")");
  }
  return trafo;
}

void CameraChainParameters::setExtrinsicsLastCamToHere(
    size_t camIdx, const sm::kinematics::Transformation& T) {
  checkCameraIndex(camIdx);
  if (camIdx == 0) {
    raiseError(
        "setExtrinsicsLastCamToHere(): can't set extrinsics for first cam in "
        "chain (cam0=base)");
  }
  auto t_vec = std::vector<double>(T.T().data(), T.T().data() + T.T().size());
  std::any_cast<std::unordered_map<std::string, std::any>&>(
      data_["cam" + std::to_string(camIdx)])["T_cn_cnm1"] = t_vec;
}

sm::kinematics::Transformation CameraChainParameters::getExtrinsicsImuToCam(
    size_t camIdx) const {
  checkCameraIndex(camIdx);
  sm::kinematics::Transformation T;
  try {
    auto t_vec = std::any_cast<std::vector<double>>(
        std::any_cast<ParametersBase::DictType>(
            data_.at("cam" + std::to_string(camIdx)))
            .at("T_cam_imu"));
    T = sm::kinematics::Transformation(
        Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(
            t_vec.data()));
  } catch (...) {
    raiseError("invalid T_cam_imu (cam" + std::to_string(camIdx) + " in " +
               yamlFile_ + ")");
  }
  return T;
}

void CameraChainParameters::setExtrinsicsImuToCam(
    size_t camIdx, const sm::kinematics::Transformation& T) {
  checkCameraIndex(camIdx);
  auto t_vec = std::vector<double>(T.T().data(), T.T().data() + T.T().size());
  std::any_cast<std::unordered_map<std::string, std::any>&>(
      data_["cam" + std::to_string(camIdx)])["T_cam_imu"] = t_vec;
}

double CameraChainParameters::getTimeshiftCamImu(size_t camIdx) const {
  checkCameraIndex(camIdx);
  double timeshift = 0.0;
  timeshift =
      std::any_cast<double>(std::any_cast<ParametersBase::DictType>(
                                data_.at("cam" + std::to_string(camIdx)))
                                .at("timeshift_cam_imu"));
  return timeshift;
}

void CameraChainParameters::setTimeshiftCamImu(size_t camIdx,
                                               double timeshift) {
  checkCameraIndex(camIdx);
  std::any_cast<std::unordered_map<std::string, std::any>&>(
      data_["cam" + std::to_string(camIdx)])["timeshift_cam_imu"] = timeshift;
}

std::vector<int> CameraChainParameters::getCamOverlaps(size_t camIdx) const {
  checkCameraIndex(camIdx);
  std::vector<int> overlaps;
  try {
    overlaps = std::any_cast<std::vector<int>>(
        std::any_cast<ParametersBase::DictType>(
            data_.at("cam" + std::to_string(camIdx)))
            .at("overlaps"));
  } catch (...) {
    raiseError("invalid overlaps (cam" + std::to_string(camIdx) + " in " +
               yamlFile_ + ")");
  }
  return overlaps;
}

void CameraChainParameters::setCamOverlaps(size_t camIdx,
                                           const std::vector<int>& overlaps) {
  checkCameraIndex(camIdx);
  std::any_cast<std::unordered_map<std::string, std::any>&>(
      data_["cam" + std::to_string(camIdx)])["overlaps"] = overlaps;
}

void CameraChainParameters::checkCameraIndex(size_t camIdx) const {
  if (camIdx >= numCameras()) {
    raiseError("Camera index " + std::to_string(camIdx) +
               " out of range (max: " + std::to_string(numCameras() - 1) + ")");
  }
}

void CameraChainParameters::printDetails(std::ostream& os) const {
  for (size_t i = 0; i < numCameras(); ++i) {
    std::println(os, "Camera chain - cam{}:", i);

    // Print extrinsics if available
    if (i > 0) {
      try {
        sm::kinematics::Transformation T = getExtrinsicsLastCamToHere(i);
        std::println(os, "  Baseline (T_cn_cnm1):");
        os << T.T()
           << std::endl;  // Eigen matrix output requires stream operator
      } catch (...) {
        std::println(os, "  Baseline: no data available");
      }
    }

    try {
      sm::kinematics::Transformation T = getExtrinsicsImuToCam(i);
      std::println(os, "  T_cam_imu:");
      os << T.T() << std::endl;  // Eigen matrix output requires stream operator
    } catch (...) {
      std::println(os, "  T_cam_imu: no data available");
    }

    double timeshift = getTimeshiftCamImu(i);
    std::println(os, "  Timeshift cam-imu: {} s", timeshift);
  }
}

// ============================================================================
// AslamCamera Implementation
// ============================================================================

std::shared_ptr<AslamCamera> AslamCamera::fromParameters(
    CameraModel cameraModel, const std::vector<double>& intrinsics,
    DistortionModel distModel, const std::vector<double>& distCoeffs,
    const Eigen::Vector2i& resolution) {
  auto cameraPtr = new AslamCamera();
  std::shared_ptr<AslamCamera> camera(cameraPtr);

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

      using GeometryType = aslam::cameras::DistortedPinholeCameraGeometry;
      using FrameType = aslam::Frame<GeometryType>;

      auto geom = std::make_shared<GeometryType>(proj);
      camera->geometry_ = geom;
      camera->frameType_ = typeid(FrameType).hash_code();
      camera->reprojectionErrorType_ =
          typeid(aslam::backend::SimpleReprojectionError<FrameType>)
              .hash_code();

    } else if (distModel == DistortionModel::Equidistant) {
      // Equidistant distortion
      aslam::cameras::EquidistantDistortion dist(distCoeffs[0], distCoeffs[1],
                                                 distCoeffs[2], distCoeffs[3]);
      aslam::cameras::PinholeProjection<aslam::cameras::EquidistantDistortion>
          proj(fu, fv, pu, pv, resolution.x(), resolution.y(), dist);

      using GeometryType =
          aslam::cameras::EquidistantDistortedPinholeCameraGeometry;
      using FrameType = aslam::Frame<GeometryType>;

      auto geom = std::make_shared<GeometryType>(proj);
      camera->geometry_ = geom;

      camera->frameType_ = typeid(FrameType).hash_code();
      camera->reprojectionErrorType_ =
          typeid(aslam::backend::SimpleReprojectionError<FrameType>)
              .hash_code();

    } else if (distModel == DistortionModel::FOV) {
      // FOV distortion
      aslam::cameras::FovDistortion dist(distCoeffs[0]);
      aslam::cameras::PinholeProjection<aslam::cameras::FovDistortion> proj(
          fu, fv, pu, pv, resolution.x(), resolution.y(), dist);

      using GeometryType = aslam::cameras::FovDistortedPinholeCameraGeometry;
      using FrameType = aslam::Frame<GeometryType>;

      auto geom = std::make_shared<GeometryType>(proj);
      camera->geometry_ = geom;

      camera->frameType_ = typeid(FrameType).hash_code();
      camera->reprojectionErrorType_ =
          typeid(aslam::backend::SimpleReprojectionError<FrameType>)
              .hash_code();

    } else if (distModel == DistortionModel::None) {
      // No distortion
      aslam::cameras::PinholeProjection<aslam::cameras::NoDistortion> proj(
          fu, fv, pu, pv, resolution.x(), resolution.y());

      using GeometryType = aslam::cameras::PinholeCameraGeometry;
      using FrameType = aslam::Frame<GeometryType>;

      auto geom = std::make_shared<GeometryType>(proj);
      camera->geometry_ = geom;

      camera->frameType_ = typeid(FrameType).hash_code();
      camera->reprojectionErrorType_ =
          typeid(aslam::backend::SimpleReprojectionError<FrameType>)
              .hash_code();

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

      using GeometryType = aslam::cameras::DistortedOmniCameraGeometry;
      using FrameType = aslam::Frame<GeometryType>;

      auto geom = std::make_shared<GeometryType>(proj);
      camera->geometry_ = geom;

      camera->frameType_ = typeid(FrameType).hash_code();
      camera->reprojectionErrorType_ =
          typeid(aslam::backend::SimpleReprojectionError<FrameType>)
              .hash_code();

    } else if (distModel == DistortionModel::Equidistant) {
      throw std::runtime_error(
          "Omni with equidistant distortion model not yet supported!");

    } else if (distModel == DistortionModel::None) {
      // No distortion
      aslam::cameras::OmniProjection<aslam::cameras::NoDistortion> proj(
          xi, fu, fv, pu, pv, resolution.x(), resolution.y());

      using GeometryType = aslam::cameras::OmniCameraGeometry;
      using FrameType = aslam::Frame<GeometryType>;

      auto geom = std::make_shared<GeometryType>(proj);
      camera->geometry_ = geom;

      camera->frameType_ = typeid(FrameType).hash_code();
      camera->reprojectionErrorType_ =
          typeid(aslam::backend::SimpleReprojectionError<FrameType>)
              .hash_code();

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

      using GeometryType = aslam::cameras::ExtendedUnifiedCameraGeometry;
      using FrameType = aslam::Frame<GeometryType>;

      auto geom = std::make_shared<GeometryType>(proj);
      camera->geometry_ = geom;

      camera->frameType_ = typeid(FrameType).hash_code();
      camera->reprojectionErrorType_ =
          typeid(aslam::backend::SimpleReprojectionError<FrameType>)
              .hash_code();

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

      using GeometryType = aslam::cameras::DoubleSphereCameraGeometry;
      using FrameType = aslam::Frame<GeometryType>;

      auto geom = std::make_shared<GeometryType>(proj);
      camera->geometry_ = geom;

      camera->frameType_ = typeid(FrameType).hash_code();
      camera->reprojectionErrorType_ =
          typeid(aslam::backend::SimpleReprojectionError<FrameType>)
              .hash_code();

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
