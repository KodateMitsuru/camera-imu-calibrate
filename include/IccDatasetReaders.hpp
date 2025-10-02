#ifndef ICCDATASETREADERS_HPP
#define ICCDATASETREADERS_HPP

#include <Eigen/Dense>
#include <aslam/Frame.hpp>
#include <filesystem>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

// IMU measurement structure for CSV reading
struct ImuMeasurementRaw {
  double timestamp;       // in seconds
  Eigen::Vector3d omega;  // angular velocity (rad/s)
  Eigen::Vector3d alpha;  // linear acceleration (m/s^2)
};

// Camera frame structure for CSV reading
struct CameraFrameInfo {
  double timestamp;      // in seconds
  std::string filename;  // image filename
};

// CSV-based Image Dataset Reader
class CsvImageDatasetReader {
 public:
  // Constructor
  // csvPath: path to CSV file with format: timestamp,image_filename
  // imageFolder: folder containing the images
  // from_to: time range to load (0,0 means all)
  // targetFreq: target frequency for downsampling (0 means no downsampling)
  CsvImageDatasetReader(const std::string& csvPath,
                        const std::string& imageFolder,
                        const std::pair<double, double>& from_to = {0.0, 0.0},
                        double targetFreq = 0.0);

  // Get number of images
  size_t getImageCount() const { return frames_.size(); }

  // Get image at index
  cv::Mat getImage(size_t index) const;

  // Get timestamp at index
  double getTimestamp(size_t index) const;

  // Get all timestamps
  std::vector<double> getTimestamps() const;

  // Iterator support
  class Iterator {
   public:
    Iterator(const CsvImageDatasetReader* reader, size_t index)
        : reader_(reader), index_(index) {}

    bool operator!=(const Iterator& other) const {
      return index_ != other.index_;
    }

    Iterator& operator++() {
      ++index_;
      return *this;
    }

    std::pair<double, cv::Mat> operator*() const {
      return {reader_->getTimestamp(index_), reader_->getImage(index_)};
    }

   private:
    const CsvImageDatasetReader* reader_;
    size_t index_;
  };

  Iterator begin() const { return Iterator(this, 0); }
  Iterator end() const { return Iterator(this, frames_.size()); }

 private:
  void loadFromCSV(const std::string& csvPath,
                   const std::pair<double, double>& from_to, double targetFreq);

  std::string imageFolder_;
  std::vector<CameraFrameInfo> frames_;
};

// CSV-based IMU Dataset Reader
class CsvImuDatasetReader {
 public:
  // Constructor
  // csvPath: path to CSV file with format:
  // timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z
  // from_to: time range to load (0,0 means all)
  CsvImuDatasetReader(const std::string& csvPath,
                      const std::pair<double, double>& from_to = {0.0, 0.0});

  // Get number of measurements
  size_t getMessageCount() const { return measurements_.size(); }

  // Get measurement at index
  const ImuMeasurementRaw& getMeasurement(size_t index) const;

  // Get topic name (for compatibility)
  std::string getTopic() const { return topic_; }

  // Iterator support
  class Iterator {
   public:
    Iterator(const CsvImuDatasetReader* reader, size_t index)
        : reader_(reader), index_(index) {}

    bool operator!=(const Iterator& other) const {
      return index_ != other.index_;
    }

    Iterator& operator++() {
      ++index_;
      return *this;
    }

    const ImuMeasurementRaw& operator*() const {
      return reader_->getMeasurement(index_);
    }

   private:
    const CsvImuDatasetReader* reader_;
    size_t index_;
  };

  Iterator begin() const { return Iterator(this, 0); }
  Iterator end() const { return Iterator(this, measurements_.size()); }

 private:
  void loadFromCSV(const std::string& csvPath,
                   const std::pair<double, double>& from_to);

  std::string topic_;
  std::vector<ImuMeasurementRaw> measurements_;
};

#endif  // ICCDATASETREADERS_HPP
