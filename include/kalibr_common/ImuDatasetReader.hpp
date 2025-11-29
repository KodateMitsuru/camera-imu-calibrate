#ifndef KALIBR_COMMON_IMU_DATASET_READER_HPP
#define KALIBR_COMMON_IMU_DATASET_READER_HPP

#include <Eigen/Dense>
#include <aslam/Time.hpp>
#include <string>
#include <vector>

namespace kalibr {

/**
 * @brief Structure to hold a single IMU data
 */
struct ImuData {
  aslam::Time timestamp;
  Eigen::Vector3d omega;  // Angular velocity (rad/s)
  Eigen::Vector3d alpha;  // Linear acceleration (m/s^2)

  ImuData() = default;

  ImuData(const aslam::Time& ts, const Eigen::Vector3d& w,
          const Eigen::Vector3d& a)
      : timestamp(ts), omega(w), alpha(a) {}
};

/**
 * @brief Iterator for ImuDatasetReader
 */
class ImuDatasetReaderIterator {
 public:
  ImuDatasetReaderIterator(const class ImuDatasetReader* dataset,
                           const std::vector<size_t>& indices, size_t position);

  // Iterator operations
  bool operator!=(const ImuDatasetReaderIterator& other) const;
  ImuDatasetReaderIterator& operator++();
  ImuData operator*() const;

 private:
  const class ImuDatasetReader* dataset_;
  std::vector<size_t> indices_;
  size_t position_;
};

/**
 * @brief IMU dataset reader that loads IMU data from CSV file
 *
 * Expected CSV format (with or without header):
 * timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z
 * or
 * timestamp,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z
 *
 * Where:
 * - timestamp: in seconds (double)
 * - omega/gyro: angular velocity in rad/s
 * - alpha/accel: linear acceleration in m/s^2
 *
 * The reader supports:
 * - Time-based truncation (only load measurements within a time range)
 * - Automatic sorting by timestamp
 */
class ImuDatasetReader {
 public:
  /**
   * @brief Constructor
   *
   * @param csvFile Path to CSV file containing IMU data
   * @param from_to Time range [start, end] in seconds relative to first
   * measurement (empty = load all)
   */
  ImuDatasetReader(const std::string& csvFile,
                   const std::pair<double, double>& from_to = {0.0, 0.0});

  /**
   * @brief Get number of IMU measurements in dataset
   */
  size_t numMessages() const { return indices_.size(); }

  /**
   * @brief Get IMU measurement at specific index
   * @return IMU measurement with timestamp, omega, and alpha
   */
  ImuData getMessage(size_t idx) const;

  /**
   * @brief Get timestamp at specific index
   */
  aslam::Time getTimestamp(size_t idx) const;

  /**
   * @brief Get all timestamps
   */
  std::vector<aslam::Time> getTimestamps() const;

  /**
   * @brief Get all measurements
   */
  std::vector<ImuData> getAllMessages() const;

  /**
   * @brief Iterator support for range-based for loops
   */
  ImuDatasetReaderIterator begin() const;
  ImuDatasetReaderIterator end() const;

  /**
   * @brief Get shuffled iterator (for randomized reading)
   */
  ImuDatasetReaderIterator readDatasetShuffle();

  /**
   * @brief Get the topic/file name (for compatibility with original API)
   */
  std::string getTopic() const { return csvFile_; }

 private:
  /**
   * @brief Load IMU measurements from CSV file
   */
  void loadFromCSV();

  /**
   * @brief Sort indices by timestamp
   */
  void sortByTime();

  /**
   * @brief Truncate indices to only include measurements within time range
   */
  void truncateIndicesFromTime(const std::pair<double, double>& from_to);

  /**
   * @brief Detect CSV format and column indices
   * @return true if format is valid
   */
  bool detectCSVFormat(const std::vector<std::string>& header);

  std::string csvFile_;
  std::vector<ImuData> measurements_;    // All measurements
  std::vector<size_t> indices_;          // Valid indices after filtering
  std::vector<size_t> shuffledIndices_;  // For shuffled reading

  // CSV column indices (after format detection)
  int colTimestamp_ = 0;
  int colOmegaX_ = 1;
  int colOmegaY_ = 2;
  int colOmegaZ_ = 3;
  int colAlphaX_ = 4;
  int colAlphaY_ = 5;
  int colAlphaZ_ = 6;
};

}  // namespace kalibr

#endif  // KALIBR_COMMON_IMU_DATASET_READER_HPP
