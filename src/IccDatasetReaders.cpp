#include "IccDatasetReaders.hpp"

#include <algorithm>
#include <aslam/cv/Time.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

// Helper function to trim whitespace
static std::string trim(const std::string& str) {
  size_t first = str.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  size_t last = str.find_last_not_of(" \t\r\n");
  return str.substr(first, (last - first + 1));
}

// Helper function to split CSV line
static std::vector<std::string> splitCSV(const std::string& line,
                                         char delimiter = ',') {
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream tokenStream(line);
  while (std::getline(tokenStream, token, delimiter)) {
    tokens.push_back(trim(token));
  }
  return tokens;
}

// ============================================================================
// CsvImageDatasetReader Implementation
// ============================================================================

CsvImageDatasetReader::CsvImageDatasetReader(
    const std::string& csvPath, const std::string& imageFolder,
    const std::pair<double, double>& from_to, double targetFreq)
    : imageFolder_(imageFolder) {
  loadFromCSV(csvPath, from_to, targetFreq);
}

void CsvImageDatasetReader::loadFromCSV(
    const std::string& csvPath, const std::pair<double, double>& from_to,
    double targetFreq) {
  std::ifstream file(csvPath);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open CSV file: " + csvPath);
  }

  std::string line;
  bool firstLine = true;
  size_t lineNum = 0;

  while (std::getline(file, line)) {
    lineNum++;
    line = trim(line);

    // Skip empty lines and comments
    if (line.empty() || line[0] == '#') continue;

    // Skip header if it exists
    if (firstLine) {
      firstLine = false;
      // Check if this is a header line (contains non-numeric characters in
      // first column)
      auto tokens = splitCSV(line);
      if (!tokens.empty()) {
        try {
          std::stod(tokens[0]);
          // First token is numeric, process this line
        } catch (...) {
          // First token is not numeric, skip header
          continue;
        }
      }
    }

    // Parse CSV line: timestamp,image_filename
    auto tokens = splitCSV(line);
    if (tokens.size() < 2) {
      std::cerr << "Warning: Invalid line " << lineNum << " in " << csvPath
                << " (expected at least 2 columns)" << std::endl;
      continue;
    }

    try {
      double timestamp = std::stod(tokens[0]);

      // Apply time filter
      if (from_to.first > 0.0 || from_to.second > 0.0) {
        if (from_to.first > 0.0 && timestamp < from_to.first) continue;
        if (from_to.second > 0.0 && timestamp > from_to.second) continue;
      }

      CameraFrameInfo frame;
      frame.timestamp = timestamp;
      frame.filename = tokens[1];
      frames_.push_back(frame);
    } catch (const std::exception& e) {
      std::cerr << "Warning: Failed to parse line " << lineNum << " in "
                << csvPath << ": " << e.what() << std::endl;
    }
  }

  file.close();

  if (frames_.empty()) {
    throw std::runtime_error("No valid frames loaded from " + csvPath);
  }

  // Sort by timestamp
  std::sort(frames_.begin(), frames_.end(),
            [](const CameraFrameInfo& a, const CameraFrameInfo& b) {
              return a.timestamp < b.timestamp;
            });

  // Apply frequency downsampling if requested
  if (targetFreq > 0.0 && !frames_.empty()) {
    std::vector<CameraFrameInfo> downsampledFrames;
    double minInterval = 1.0 / targetFreq;
    double lastTimestamp = frames_[0].timestamp - minInterval;

    for (const auto& frame : frames_) {
      if (frame.timestamp - lastTimestamp >= minInterval) {
        downsampledFrames.push_back(frame);
        lastTimestamp = frame.timestamp;
      }
    }
    frames_ = std::move(downsampledFrames);
  }

  std::cout << "Loaded " << frames_.size() << " image frames from " << csvPath
            << std::endl;
}

cv::Mat CsvImageDatasetReader::getImage(size_t index) const {
  if (index >= frames_.size()) {
    throw std::out_of_range("Image index out of range");
  }

  std::string imagePath = imageFolder_ + "/" + frames_[index].filename;
  cv::Mat image = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);

  if (image.empty()) {
    // Try without leading slash
    imagePath = imageFolder_ + frames_[index].filename;
    image = cv::imread(imagePath, cv::IMREAD_GRAYSCALE);
  }

  if (image.empty()) {
    throw std::runtime_error("Failed to load image: " + imagePath);
  }

  return image;
}

double CsvImageDatasetReader::getTimestamp(size_t index) const {
  if (index >= frames_.size()) {
    throw std::out_of_range("Image index out of range");
  }
  return frames_[index].timestamp;
}

std::vector<double> CsvImageDatasetReader::getTimestamps() const {
  std::vector<double> timestamps;
  timestamps.reserve(frames_.size());
  for (const auto& frame : frames_) {
    timestamps.push_back(frame.timestamp);
  }
  return timestamps;
}

// ============================================================================
// CsvImuDatasetReader Implementation
// ============================================================================

CsvImuDatasetReader::CsvImuDatasetReader(
    const std::string& csvPath, const std::pair<double, double>& from_to)
    : topic_("imu0") {
  loadFromCSV(csvPath, from_to);
}

void CsvImuDatasetReader::loadFromCSV(
    const std::string& csvPath, const std::pair<double, double>& from_to) {
  std::ifstream file(csvPath);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open CSV file: " + csvPath);
  }

  std::string line;
  bool firstLine = true;
  size_t lineNum = 0;

  while (std::getline(file, line)) {
    lineNum++;
    line = trim(line);

    // Skip empty lines and comments
    if (line.empty() || line[0] == '#') continue;

    // Skip header if it exists
    if (firstLine) {
      firstLine = false;
      auto tokens = splitCSV(line);
      if (!tokens.empty()) {
        try {
          std::stod(tokens[0]);
          // First token is numeric, process this line
        } catch (...) {
          // First token is not numeric, skip header
          continue;
        }
      }
    }

    // Parse CSV line: timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z
    auto tokens = splitCSV(line);
    if (tokens.size() < 7) {
      std::cerr << "Warning: Invalid line " << lineNum << " in " << csvPath
                << " (expected 7 columns: timestamp,omega_x,omega_y,omega_z,"
                   "alpha_x,alpha_y,alpha_z)"
                << std::endl;
      continue;
    }

    try {
      ImuMeasurementRaw measurement;
      measurement.timestamp = std::stod(tokens[0]);

      // Apply time filter
      if (from_to.first > 0.0 || from_to.second > 0.0) {
        if (from_to.first > 0.0 && measurement.timestamp < from_to.first)
          continue;
        if (from_to.second > 0.0 && measurement.timestamp > from_to.second)
          continue;
      }

      measurement.omega(0) = std::stod(tokens[1]);  // omega_x
      measurement.omega(1) = std::stod(tokens[2]);  // omega_y
      measurement.omega(2) = std::stod(tokens[3]);  // omega_z
      measurement.alpha(0) = std::stod(tokens[4]);  // alpha_x
      measurement.alpha(1) = std::stod(tokens[5]);  // alpha_y
      measurement.alpha(2) = std::stod(tokens[6]);  // alpha_z

      measurements_.push_back(measurement);
    } catch (const std::exception& e) {
      std::cerr << "Warning: Failed to parse line " << lineNum << " in "
                << csvPath << ": " << e.what() << std::endl;
    }
  }

  file.close();

  if (measurements_.empty()) {
    throw std::runtime_error("No valid IMU measurements loaded from " +
                             csvPath);
  }

  // Sort by timestamp
  std::sort(measurements_.begin(), measurements_.end(),
            [](const ImuMeasurementRaw& a, const ImuMeasurementRaw& b) {
              return a.timestamp < b.timestamp;
            });

  std::cout << "Loaded " << measurements_.size() << " IMU measurements from "
            << csvPath << std::endl;
  if (!measurements_.empty()) {
    double duration =
        measurements_.back().timestamp - measurements_.front().timestamp;
    std::cout << "  Time range: " << measurements_.front().timestamp << " to "
              << measurements_.back().timestamp << " (" << duration
              << " seconds)" << std::endl;
  }
}

const ImuMeasurementRaw& CsvImuDatasetReader::getMeasurement(
    size_t index) const {
  if (index >= measurements_.size()) {
    throw std::out_of_range("IMU measurement index out of range");
  }
  return measurements_[index];
}
