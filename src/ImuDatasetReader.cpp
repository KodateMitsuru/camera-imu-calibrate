#include <cstdint>
#include <rapidcsv.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <kalibr_common/ImuDatasetReader.hpp>
#include <print>
#include <random>
#include <sstream>
#include <stdexcept>

namespace kalibr {

// ============================================================================
// ImuDatasetReaderIterator Implementation
// ============================================================================

ImuDatasetReaderIterator::ImuDatasetReaderIterator(
    const ImuDatasetReader* dataset, const std::vector<size_t>& indices,
    size_t position)
    : dataset_(dataset), indices_(indices), position_(position) {}

bool ImuDatasetReaderIterator::operator!=(
    const ImuDatasetReaderIterator& other) const {
  return position_ != other.position_;
}

ImuDatasetReaderIterator& ImuDatasetReaderIterator::operator++() {
  ++position_;
  return *this;
}

ImuData ImuDatasetReaderIterator::operator*() const {
  return dataset_->getMessage(indices_[position_]);
}

// ============================================================================
// ImuDatasetReader Implementation
// ============================================================================

ImuDatasetReader::ImuDatasetReader(const std::string& csvFile,
                                   const std::pair<double, double>& from_to)
    : csvFile_(csvFile) {
  if (!std::filesystem::exists(csvFile)) {
    throw std::runtime_error("CSV file does not exist: " + csvFile);
  }

  // Load all IMU measurements
  loadFromCSV();

  if (measurements_.empty()) {
    throw std::runtime_error("No valid IMU measurements found in CSV: " +
                             csvFile);
  }

  // Initially all measurements are valid
  indices_.resize(measurements_.size());
  std::iota(indices_.begin(), indices_.end(), 0);

  // Sort by timestamp
  sortByTime();

  // Apply time-based truncation
  if (from_to.first > 0.0 || from_to.second > 0.0) {
    truncateIndicesFromTime(from_to);
  }

  std::println("ImuDatasetReader: Loaded {} IMU measurements from {}",
               indices_.size(), csvFile);
}

bool ImuDatasetReader::detectCSVFormat(const std::vector<std::string>& header) {
  // Check if we have a header line
  if (header.empty()) {
    return false;
  }

  // Try to detect column format based on header names
  bool hasHeader = false;

  for (size_t i = 0; i < header.size(); ++i) {
    std::string col = header[i];
    // Convert to lowercase for comparison
    std::transform(col.begin(), col.end(), col.begin(), ::tolower);

    // Check for common header names
    if (col.find("time") != std::string::npos || col == "t") {
      colTimestamp_ = i;
      hasHeader = true;
    } else if (col.find("omega_x") != std::string::npos ||
               col.find("gyro_x") != std::string::npos ||
               col.find("wx") != std::string::npos ||
               col.find("gx") != std::string::npos) {
      colOmegaX_ = i;
      hasHeader = true;
    } else if (col.find("omega_y") != std::string::npos ||
               col.find("gyro_y") != std::string::npos ||
               col.find("wy") != std::string::npos ||
               col.find("gy") != std::string::npos) {
      colOmegaY_ = i;
      hasHeader = true;
    } else if (col.find("omega_z") != std::string::npos ||
               col.find("gyro_z") != std::string::npos ||
               col.find("wz") != std::string::npos ||
               col.find("gz") != std::string::npos) {
      colOmegaZ_ = i;
      hasHeader = true;
    } else if (col.find("alpha_x") != std::string::npos ||
               col.find("accel_x") != std::string::npos ||
               col.find("ax") != std::string::npos) {
      colAlphaX_ = i;
      hasHeader = true;
    } else if (col.find("alpha_y") != std::string::npos ||
               col.find("accel_y") != std::string::npos ||
               col.find("ay") != std::string::npos) {
      colAlphaY_ = i;
      hasHeader = true;
    } else if (col.find("alpha_z") != std::string::npos ||
               col.find("accel_z") != std::string::npos ||
               col.find("az") != std::string::npos) {
      colAlphaZ_ = i;
      hasHeader = true;
    }
  }

  return hasHeader;
}

void ImuDatasetReader::loadFromCSV() {
  measurements_.clear();

  try {
    // First, try to read the first line to detect format
    std::ifstream file(csvFile_);
    std::string firstLine;
    if (file.is_open() && std::getline(file, firstLine)) {
      file.close();

      // Parse first line to check if it's a header
      std::vector<std::string> headerTokens;
      std::stringstream ss(firstLine);
      std::string token;
      while (std::getline(ss, token, ',')) {
        // Trim whitespace
        token.erase(0, token.find_first_not_of(" \t\r\n"));
        token.erase(token.find_last_not_of(" \t\r\n") + 1);
        headerTokens.push_back(token);
      }

      // Try to detect if first line is a header
      bool hasHeader = detectCSVFormat(headerTokens);

      // Load CSV with rapidcsv
      rapidcsv::Document doc;
      if (hasHeader) {
        // Skip header line
        doc.Load(csvFile_, rapidcsv::LabelParams(0, -1));
      } else {
        // No header, use default column indices
        doc.Load(csvFile_, rapidcsv::LabelParams(-1, -1));
        colTimestamp_ = 0;
        colOmegaX_ = 1;
        colOmegaY_ = 2;
        colOmegaZ_ = 3;
        colAlphaX_ = 4;
        colAlphaY_ = 5;
        colAlphaZ_ = 6;
      }

      // Check if we have enough columns
      size_t numCols = doc.GetColumnCount();
      size_t maxColIdx =
          std::max({colTimestamp_, colOmegaX_, colOmegaY_, colOmegaZ_,
                    colAlphaX_, colAlphaY_, colAlphaZ_});

      if (numCols <= maxColIdx) {
        throw std::runtime_error(
            "CSV file does not have enough columns. Expected at least " +
            std::to_string(maxColIdx + 1) + " columns, got " +
            std::to_string(numCols));
      }

      // Read all rows
      size_t numRows = doc.GetRowCount();
      measurements_.reserve(numRows);

      for (size_t i = 0; i < numRows; ++i) {
        try {
          uint64_t timestamp = doc.GetCell<uint64_t>(colTimestamp_, i);
          double omega_x = doc.GetCell<double>(colOmegaX_, i);
          double omega_y = doc.GetCell<double>(colOmegaY_, i);
          double omega_z = doc.GetCell<double>(colOmegaZ_, i);
          double alpha_x = doc.GetCell<double>(colAlphaX_, i);
          double alpha_y = doc.GetCell<double>(colAlphaY_, i);
          double alpha_z = doc.GetCell<double>(colAlphaZ_, i);

          aslam::Time aslamTime;
          aslamTime.fromNSec(timestamp);

          Eigen::Vector3d omega(omega_x, omega_y, omega_z);
          Eigen::Vector3d alpha(alpha_x, alpha_y, alpha_z);

          measurements_.emplace_back(aslamTime, omega, alpha);

        } catch (const std::exception& e) {
          std::println(stderr, "Warning: Failed to parse row {}: {}", i,
                       e.what());
          continue;
        }
      }

    } else {
      throw std::runtime_error("Could not open CSV file: " + csvFile_);
    }

  } catch (const std::exception& e) {
    throw std::runtime_error("Failed to load CSV file: " + csvFile_ +
                             "\nError: " + e.what());
  }
}

void ImuDatasetReader::sortByTime() {
  // Sort indices by timestamp
  std::sort(indices_.begin(), indices_.end(), [this](size_t a, size_t b) {
    return measurements_[a].timestamp < measurements_[b].timestamp;
  });
}

void ImuDatasetReader::truncateIndicesFromTime(
    const std::pair<double, double>& from_to) {
  if (indices_.empty()) {
    return;
  }

  // Get the timestamps
  std::vector<double> timestamps;
  timestamps.reserve(indices_.size());
  for (size_t idx : indices_) {
    timestamps.push_back(measurements_[idx].timestamp.toSec());
  }

  double bagstart = *std::min_element(timestamps.begin(), timestamps.end());
  double baglength =
      *std::max_element(timestamps.begin(), timestamps.end()) - bagstart;

  std::println("IMU dataset start: {} s", bagstart);
  std::println("IMU dataset length: {} s", baglength);

  // Value checking
  if (from_to.first >= from_to.second && from_to.second > 0.0) {
    throw std::runtime_error("Start time must be less than end time.");
  }

  if (from_to.first < 0.0) {
    std::println(stderr, "Warning: Start time of {} s is less than 0",
                 from_to.first);
  }

  if (from_to.second > baglength) {
    std::println(
        stderr,
        "Warning: End time of {} s is greater than the total length of {} s",
        from_to.second, baglength);
  }

  // Find valid indices
  std::vector<size_t> validIndices;
  for (size_t i = 0; i < indices_.size(); ++i) {
    double timestamp = timestamps[i];
    double relativeTime = timestamp - bagstart;

    if (relativeTime >= from_to.first &&
        (from_to.second <= 0.0 || relativeTime <= from_to.second)) {
      validIndices.push_back(indices_[i]);
    }
  }

  size_t removed = indices_.size() - validIndices.size();
  if (removed > 0) {
    std::println("ImuDatasetReader: Truncated {} / {} messages (time-based)",
                 removed, indices_.size());
  }

  indices_ = std::move(validIndices);
}

ImuData ImuDatasetReader::getMessage(size_t idx) const {
  if (idx >= indices_.size()) {
    throw std::out_of_range(
        "IMU message index out of range: " + std::to_string(idx) +
        " >= " + std::to_string(indices_.size()));
  }

  return measurements_[indices_[idx]];
}

aslam::Time ImuDatasetReader::getTimestamp(size_t idx) const {
  if (idx >= indices_.size()) {
    throw std::out_of_range("IMU message index out of range");
  }

  return measurements_[indices_[idx]].timestamp;
}

std::vector<aslam::Time> ImuDatasetReader::getTimestamps() const {
  std::vector<aslam::Time> timestamps;
  timestamps.reserve(indices_.size());

  for (size_t idx : indices_) {
    timestamps.push_back(measurements_[idx].timestamp);
  }

  return timestamps;
}

std::vector<ImuData> ImuDatasetReader::getAllMessages() const {
  std::vector<ImuData> messages;
  messages.reserve(indices_.size());

  for (size_t idx : indices_) {
    messages.push_back(measurements_[idx]);
  }

  return messages;
}

ImuDatasetReaderIterator ImuDatasetReader::begin() const {
  return ImuDatasetReaderIterator(this, indices_, 0);
}

ImuDatasetReaderIterator ImuDatasetReader::end() const {
  return ImuDatasetReaderIterator(this, indices_, indices_.size());
}

ImuDatasetReaderIterator ImuDatasetReader::readDatasetShuffle() {
  // Create shuffled indices
  shuffledIndices_ = indices_;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::shuffle(shuffledIndices_.begin(), shuffledIndices_.end(), gen);

  return ImuDatasetReaderIterator(this, shuffledIndices_, 0);
}

}  // namespace kalibr
