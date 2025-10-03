#ifndef KALIBR_COMMON_IMAGE_DATASET_READER_HPP
#define KALIBR_COMMON_IMAGE_DATASET_READER_HPP

#include <aslam/Time.hpp>
#include <filesystem>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace kalibr {

/**
 * @brief Structure to hold image information
 */
struct ImageInfo {
  aslam::Time timestamp;
  std::filesystem::path filepath;
  size_t index;

  ImageInfo(const aslam::Time& ts, const std::filesystem::path& path,
            size_t idx)
      : timestamp(ts), filepath(path), index(idx) {}
};

/**
 * @brief Iterator for ImageDatasetReader
 */
class ImageDatasetReaderIterator {
 public:
  ImageDatasetReaderIterator(const class ImageDatasetReader* dataset,
                             const std::vector<size_t>& indices,
                             size_t position);

  // Iterator operations
  bool operator!=(const ImageDatasetReaderIterator& other) const;
  ImageDatasetReaderIterator& operator++();
  std::pair<aslam::Time, cv::Mat> operator*() const;

 private:
  const class ImageDatasetReader* dataset_;
  std::vector<size_t> indices_;
  size_t position_;
};

/**
 * @brief Image dataset reader that loads images from a folder
 *
 * This class reads images from a folder structure where:
 * - Images are named by their timestamps (e.g., "1234567890.123456.png")
 * - Supported formats: .png, .jpg, .jpeg, .bmp, .tif, .tiff
 *
 * The reader supports:
 * - Time-based truncation (only load images within a time range)
 * - Frequency-based downsampling
 * - Automatic sorting by timestamp
 */
class ImageDatasetReader {
 public:
  /**
   * @brief Constructor
   *
   * @param imageFolder Path to folder containing images
   * @param from_to Time range [start, end] in seconds relative to first image
   * (empty = load all)
   * @param targetFreq Target frequency for downsampling in Hz (0 = no
   * downsampling)
   */
  ImageDatasetReader(const std::string& imageFolder,
                     const std::pair<double, double>& from_to = {0.0, 0.0},
                     double targetFreq = 0.0);

  /**
   * @brief Get number of images in dataset
   */
  size_t numImages() const { return indices_.size(); }

  /**
   * @brief Get image at specific index
   * @return Pair of (timestamp, image)
   */
  std::pair<aslam::Time, cv::Mat> getImage(size_t idx) const;

  /**
   * @brief Get timestamp at specific index
   */
  aslam::Time getTimestamp(size_t idx) const;

  /**
   * @brief Get all timestamps
   */
  std::vector<aslam::Time> getTimestamps() const;

  /**
   * @brief Iterator support for range-based for loops
   */
  ImageDatasetReaderIterator begin() const;
  ImageDatasetReaderIterator end() const;

  /**
   * @brief Get shuffled iterator (for randomized reading)
   */
  ImageDatasetReaderIterator readDatasetShuffle();

  /**
   * @brief Get the topic/folder name (for compatibility with original API)
   */
  std::string getTopic() const { return imageFolder_; }

 private:
  /**
   * @brief Load image files from folder and sort by timestamp
   */
  void loadImageFiles();

  /**
   * @brief Extract timestamp from filename
   * Supports formats: "timestamp.ext", "timestamp_anything.ext", etc.
   * @return Timestamp in seconds, or -1 if parsing failed
   */
  double extractTimestampFromFilename(const std::string& filename) const;

  /**
   * @brief Sort indices by timestamp
   */
  void sortByTime();

  /**
   * @brief Truncate indices to only include images within time range
   */
  void truncateIndicesFromTime(const std::pair<double, double>& from_to);

  /**
   * @brief Downsample indices to target frequency
   */
  void truncateIndicesFromFreq(double freq);

  /**
   * @brief Check if file extension is a supported image format
   */
  bool isSupportedImageFormat(const std::string& extension) const;

  std::string imageFolder_;
  std::vector<ImageInfo> images_;        // All discovered images
  std::vector<size_t> indices_;          // Valid indices after filtering
  std::vector<size_t> shuffledIndices_;  // For shuffled reading
};

}  // namespace kalibr

#endif  // KALIBR_COMMON_IMAGE_DATASET_READER_HPP
