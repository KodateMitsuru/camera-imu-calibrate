#ifndef ICCCALIBRATOR_HPP
#define ICCCALIBRATOR_HPP

#include <Eigen/Dense>
#include <filesystem>
#include <string>
#include <vector>

struct ImuData {
    double timestamp;
    std::vector<double> accel; // 3 elements
    std::vector<double> gyro;  // 3 elements
};

class IccCalibrator {
  public:
    IccCalibrator() = default;
    ~IccCalibrator() = default;

    void loadConfig(const std::string& configPath);
    void loadData(const std::string& dataPath);
    void calibrate();
    void saveResults(const std::string& outputPath);
  private:
    // Camera parameters
    Eigen::Vector2d resolution;
    Eigen::Matrix3d intrinsics;
    Eigen::Vector4d distortion;

    // Imu parameters
    Eigen::Vector3d accelBias;
    Eigen::Vector3d gyroBias;
    Eigen::Matrix3d accelNoiseCov;
    Eigen::Matrix3d gyroNoiseCov;
    Eigen::Matrix3d accelBiasCov;
    Eigen::Matrix3d gyroBiasCov;

    // IMU data
    std::vector<ImuData> imuData;

    // Image file paths
    std::vector<std::filesystem::path> imageFiles;
};

#endif // ICCCALIBRATOR_HPP