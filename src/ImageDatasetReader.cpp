#include <algorithm>
#include <iostream>
#include <kalibr_common/ImageDatasetReader.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <random>

namespace kalibr {

// ============================================================================
// ImageDatasetReaderIterator Implementation
// ============================================================================

ImageDatasetReaderIterator::ImageDatasetReaderIterator(
    const ImageDatasetReader* dataset, const std::vector<size_t>& indices,
    size_t position)
    : dataset_(dataset), indices_(indices), position_(position) {}

bool ImageDatasetReaderIterator::operator!=(
    const ImageDatasetReaderIterator& other) const {
  return position_ != other.position_;
}

ImageDatasetReaderIterator& ImageDatasetReaderIterator::operator++() {
  ++position_;
  return *this;
}

std::pair<aslam::Time, cv::Mat> ImageDatasetReaderIterator::operator*() const {
  return dataset_->getImage(indices_[position_]);
}

// ============================================================================
// ImageDatasetReader Implementation
// ============================================================================

ImageDatasetReader::ImageDatasetReader(const std::string& imageFolder,
                                       const std::pair<double, double>& from_to,
                                       double targetFreq)
    : imageFolder_(imageFolder) {
  if (!std::filesystem::exists(imageFolder)) {
    throw std::runtime_error("Image folder does not exist: " + imageFolder);
  }

  if (!std::filesystem::is_directory(imageFolder)) {
    throw std::runtime_error("Path is not a directory: " + imageFolder);
  }

  // Load all image files
  loadImageFiles();

  if (images_.empty()) {
    throw std::runtime_error("No valid images found in folder: " + imageFolder);
  }

  // Initially all images are valid
  indices_.resize(images_.size());
  std::iota(indices_.begin(), indices_.end(), 0);

  // Sort by timestamp
  sortByTime();

  // Apply time-based truncation
  if (from_to.first > 0.0 || from_to.second > 0.0) {
    truncateIndicesFromTime(from_to);
  }

  // Apply frequency-based downsampling
  if (targetFreq > 0.0) {
    truncateIndicesFromFreq(targetFreq);
  }

  std::cout << "ImageDatasetReader: Loaded " << indices_.size()
            << " images from " << imageFolder << std::endl;
}

void ImageDatasetReader::loadImageFiles() {
  images_.clear();

  size_t idx = 0;
  for (const auto& entry : std::filesystem::directory_iterator(imageFolder_)) {
    if (!entry.is_regular_file()) {
      continue;
    }

    const auto& path = entry.path();
    std::string extension = path.extension().string();

    // Convert extension to lowercase for comparison
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   ::tolower);

    if (!isSupportedImageFormat(extension)) {
      continue;
    }

    // Extract timestamp from filename
    std::string filename = path.stem().string();
    double timestamp = extractTimestampFromFilename(filename);

    if (timestamp < 0.0) {
      std::cerr << "Warning: Could not extract timestamp from filename: "
                << path.filename().string() << std::endl;
      continue;
    }

    aslam::Time aslamTime;
    aslamTime.fromSec(timestamp);

    images_.emplace_back(aslamTime, path, idx++);
  }
}

double ImageDatasetReader::extractTimestampFromFilename(
    const std::string& filename) const {
  // Try to parse timestamp from filename
  // Supported formats:
  // - "1234567890.123456" (timestamp with decimal)
  // - "1234567890_123456" (timestamp with underscore for nanoseconds)
  // - "1234567890" (timestamp in seconds)

  try {
    // Find first numeric character
    size_t start = 0;
    while (start < filename.length() && !std::isdigit(filename[start])) {
      ++start;
    }

    if (start >= filename.length()) {
      return -1.0;
    }

    // Find end of numeric part (allowing dots and underscores)
    size_t end = start;
    bool hasDot = false;
    while (end < filename.length() &&
           (std::isdigit(filename[end]) || filename[end] == '.' ||
            filename[end] == '_')) {
      if (filename[end] == '.') {
        if (hasDot) break;  // Second dot means end of timestamp
        hasDot = true;
      }
      ++end;
    }

    std::string timestampStr = filename.substr(start, end - start);

    // Replace underscore with dot for parsing
    std::replace(timestampStr.begin(), timestampStr.end(), '_', '.');

    // Parse as double
    double timestamp = std::stod(timestampStr);

    // Check if timestamp is reasonable (between year 2000 and 2100)
    if (timestamp < 946684800.0 || timestamp > 4102444800.0) {
      // Maybe it's in nanoseconds or milliseconds, try to convert
      if (timestamp > 1e15) {
        // Likely nanoseconds
        timestamp /= 1e9;
      } else if (timestamp > 1e12) {
        // Likely milliseconds
        timestamp /= 1e3;
      } else {
        return -1.0;
      }
    }

    return timestamp;

  } catch (const std::exception& e) {
    return -1.0;
  }
}

bool ImageDatasetReader::isSupportedImageFormat(
    const std::string& extension) const {
  static const std::vector<std::string> supportedFormats = {
      ".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff", ".ppm", ".pgm"};

  return std::find(supportedFormats.begin(), supportedFormats.end(),
                   extension) != supportedFormats.end();
}

void ImageDatasetReader::sortByTime() {
  // Sort indices by timestamp
  std::sort(indices_.begin(), indices_.end(), [this](size_t a, size_t b) {
    return images_[a].timestamp < images_[b].timestamp;
  });
}

void ImageDatasetReader::truncateIndicesFromTime(
    const std::pair<double, double>& from_to) {
  if (indices_.empty()) {
    return;
  }

  // Get the timestamps
  std::vector<double> timestamps;
  timestamps.reserve(indices_.size());
  for (size_t idx : indices_) {
    timestamps.push_back(images_[idx].timestamp.toSec());
  }

  double bagstart = *std::min_element(timestamps.begin(), timestamps.end());
  double baglength =
      *std::max_element(timestamps.begin(), timestamps.end()) - bagstart;

  // Value checking
  if (from_to.first >= from_to.second && from_to.second > 0.0) {
    throw std::runtime_error("Start time must be less than end time.");
  }

  if (from_to.first < 0.0) {
    std::cerr << "Warning: Start time of " << from_to.first
              << " s is less than 0" << std::endl;
  }

  if (from_to.second > baglength) {
    std::cerr << "Warning: End time of " << from_to.second
              << " s is greater than the total length of " << baglength << " s"
              << std::endl;
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
    std::cout << "ImageDatasetReader: Truncated " << removed << " / "
              << indices_.size() << " images (time-based)" << std::endl;
  }

  indices_ = std::move(validIndices);
}

void ImageDatasetReader::truncateIndicesFromFreq(double freq) {
  if (freq <= 0.0) {
    throw std::runtime_error("Frequency must be positive, got: " +
                             std::to_string(freq));
  }

  if (indices_.empty()) {
    return;
  }

  double minInterval = 1.0 / freq;
  double lastTimestamp = -1.0;
  std::vector<size_t> validIndices;

  for (size_t idx : indices_) {
    double timestamp = images_[idx].timestamp.toSec();

    if (lastTimestamp < 0.0) {
      // First image
      lastTimestamp = timestamp;
      validIndices.push_back(idx);
      continue;
    }

    if ((timestamp - lastTimestamp) >= minInterval) {
      lastTimestamp = timestamp;
      validIndices.push_back(idx);
    }
  }

  size_t removed = indices_.size() - validIndices.size();
  if (removed > 0) {
    std::cout << "ImageDatasetReader: Truncated " << removed << " / "
              << indices_.size() << " images (frequency-based)" << std::endl;
  }

  indices_ = std::move(validIndices);
}

std::pair<aslam::Time, cv::Mat> ImageDatasetReader::getImage(size_t idx) const {
  if (idx >= indices_.size()) {
    throw std::out_of_range("Image index out of range: " + std::to_string(idx) +
                            " >= " + std::to_string(indices_.size()));
  }

  const ImageInfo& info = images_[indices_[idx]];

  // Load image
  cv::Mat image = cv::imread(info.filepath.string(), cv::IMREAD_UNCHANGED);

  if (image.empty()) {
    throw std::runtime_error("Failed to load image: " + info.filepath.string());
  }

  // Convert to grayscale if needed
  if (image.channels() == 3) {
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
  } else if (image.channels() == 4) {
    cv::cvtColor(image, image, cv::COLOR_BGRA2GRAY);
  }

  // Convert to 8-bit if needed
  if (image.depth() == CV_16U) {
    // 16-bit to 8-bit conversion
    image.convertTo(image, CV_8U, 1.0 / 256.0);
  }

  return {info.timestamp, image};
}

aslam::Time ImageDatasetReader::getTimestamp(size_t idx) const {
  if (idx >= indices_.size()) {
    throw std::out_of_range("Image index out of range");
  }

  return images_[indices_[idx]].timestamp;
}

std::vector<aslam::Time> ImageDatasetReader::getTimestamps() const {
  std::vector<aslam::Time> timestamps;
  timestamps.reserve(indices_.size());

  for (size_t idx : indices_) {
    timestamps.push_back(images_[idx].timestamp);
  }

  return timestamps;
}

ImageDatasetReaderIterator ImageDatasetReader::begin() const {
  return ImageDatasetReaderIterator(this, indices_, 0);
}

ImageDatasetReaderIterator ImageDatasetReader::end() const {
  return ImageDatasetReaderIterator(this, indices_, indices_.size());
}

ImageDatasetReaderIterator ImageDatasetReader::readDatasetShuffle() {
  // Create shuffled indices
  shuffledIndices_ = indices_;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::shuffle(shuffledIndices_.begin(), shuffledIndices_.end(), gen);

  return ImageDatasetReaderIterator(this, shuffledIndices_, 0);
}

}  // namespace kalibr
