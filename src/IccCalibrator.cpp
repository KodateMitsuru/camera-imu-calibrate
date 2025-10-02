#include "IccCalibrator.hpp"
#include <rapidcsv.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "IccUtils.hpp"

void IccCalibrator::loadConfig(const std::string& configPath) {
    cv::FileStorage fs(configPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("Failed to open config file: " + configPath);
    }
    // Load camera parameters
    cv::FileNode cameraNode = fs["Camera"];
    cv::Mat resolution_cv; 
    cameraNode["Resolution"] >> resolution_cv;
    cv::Mat intrinsics_cv; 
    cameraNode["Intrinsics"] >> intrinsics_cv;
    cv::Mat distortion_cv; 
    cameraNode["Distortion"] >> distortion_cv;

    // Load IMU parameters
    cv::FileNode imuNode = fs["IMU"];
    cv::Mat accelBias_cv;
    imuNode["AccelBias"] >> accelBias_cv;
    cv::Mat gyroBias_cv;
    imuNode["GyroBias"] >> gyroBias_cv;
    cv::Mat accelNoiseCov_cv;
    imuNode["AccelNoiseCov"] >> accelNoiseCov_cv;
    cv::Mat gyroNoiseCov_cv;
    imuNode["GyroNoiseCov"] >> gyroNoiseCov_cv;
    cv::Mat accelBiasCov_cv;
    imuNode["AccelBiasCov"] >> accelBiasCov_cv;
    cv::Mat gyroBiasCov_cv;
    imuNode["GyroBiasCov"] >> gyroBiasCov_cv;


    // Convert OpenCV Mats to Eigen matrices
    cv::cv2eigen(resolution_cv, this->resolution);
    cv::cv2eigen(intrinsics_cv, this->intrinsics);
    cv::cv2eigen(distortion_cv, this->distortion);
    cv::cv2eigen(accelBias_cv, this->accelBias);
    cv::cv2eigen(gyroBias_cv, this->gyroBias);
    cv::cv2eigen(accelNoiseCov_cv, this->accelNoiseCov);
    cv::cv2eigen(gyroNoiseCov_cv, this->gyroNoiseCov);
    cv::cv2eigen(accelBiasCov_cv, this->accelBiasCov);
    cv::cv2eigen(gyroBiasCov_cv, this->gyroBiasCov);
}

void IccCalibrator::loadData(const std::string& dataPath) {
    std::filesystem::path p(dataPath);
    if (!std::filesystem::exists(p)) {
        throw std::runtime_error("Data path does not exist: " + dataPath);
    }
    if (std::filesystem::is_directory(p)) {
        std::filesystem::path imuFile;
        imuFile = p / "imu_data.csv";
        std::ifstream imuStream(imuFile);
        if (!imuStream.is_open()) {
            throw std::runtime_error("Failed to open IMU data file: " + imuFile.string());
        }
        rapidcsv::Document doc(imuStream,
                               rapidcsv::LabelParams(0, 0));
        for (size_t i = 0; i < doc.GetRowCount(); ++i) {
            ImuData imuEntry;
            imuEntry.timestamp = doc.GetCell<double>("timestamp", i);
            imuEntry.accel = {
                doc.GetCell<double>("accel_x", i),
                doc.GetCell<double>("accel_y", i),
                doc.GetCell<double>("accel_z", i)
            };
            imuEntry.gyro = {
                doc.GetCell<double>("gyro_x", i),
                doc.GetCell<double>("gyro_y", i),
                doc.GetCell<double>("gyro_z", i)
            };
            this->imuData.push_back(imuEntry);
        }
        imuStream.close();
        for (const auto& entry : std::filesystem::directory_iterator(p)) {
            if (entry.is_regular_file()) {
                std::string filename = entry.path().filename().string();
                if ((filename.find(".png") != std::string::npos || filename.find(".jpg") != std::string::npos)) {
                    this->imageFiles.push_back(entry.path());
                }
            }
        }
    } else {
        throw std::runtime_error("Data path is not a directory: " + dataPath);
    }
}

void IccCalibrator::calibrate() {
    // Perform calibration
}

void IccCalibrator::saveResults(const std::string& outputPath) {
    // Save results to file
    cv::FileStorage fs(outputPath, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        throw std::runtime_error("Failed to open output file: " + outputPath);
    }
    // Save camera parameters
    cv::Mat rot_cv;
    fs << "Camera" << rot_cv; // Placeholder
    fs << "Offset" <<  0.0;  // Placeholder
    fs.release();
}
