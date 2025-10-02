/**
 * 简单的测试程序，展示如何使用新的CSV数据读取器和KalibrCommon
 */

#include <iostream>

#include "IccDatasetReaders.hpp"
#include "KalibrCommon.hpp"

int main(int argc, char** argv) {
  std::cout << "=== Camera-IMU Calibration Test ===" << std::endl;
  std::cout << std::endl;

  // 1. 测试 KalibrCommon - 读取配置文件
  std::cout << "1. Testing KalibrCommon configuration..." << std::endl;
  try {
    // 假设有一个配置文件
    std::string config_file = (argc > 1) ? argv[1] : "config_example.yaml";
    std::cout << "   Loading config: " << config_file << std::endl;

    auto args = kalibr_common::parseConfigFile(config_file);
    std::cout << "   ✓ Config loaded successfully" << std::endl;
    std::cout << "     - Output file: " << args.output_file << std::endl;
    std::cout << "     - Max iterations: " << args.max_iterations << std::endl;
    std::cout << "     - CSV files: " << args.bagfile.size() << std::endl;

    // 读取相机链配置
    auto chain = kalibr_common::ChainParameters::fromYamlFile(config_file);
    std::cout << "   ✓ Camera chain loaded: " << chain.numCameras()
              << " camera(s)" << std::endl;

    for (int i = 0; i < chain.numCameras(); ++i) {
      auto cam = chain.getCameraParameters(i);
      std::cout << "     Camera " << i << ":" << std::endl;
      std::cout << "       - Model: " << cam.getCameraModel() << std::endl;
      std::cout << "       - Resolution: " << cam.getResolution()[0] << "x"
                << cam.getResolution()[1] << std::endl;
      std::cout << "       - CSV file: " << cam.getCsvFile() << std::endl;
      std::cout << "       - Image folder: " << cam.getImageFolder()
                << std::endl;
    }
  } catch (const std::exception& e) {
    std::cout << "   ✗ Config loading failed: " << e.what() << std::endl;
    std::cout << "   (This is expected if config file doesn't exist)"
              << std::endl;
  }
  std::cout << std::endl;

  // 2. 测试 CSV Image Dataset Reader
  std::cout << "2. Testing CsvImageDatasetReader..." << std::endl;
  try {
    std::string csv_file = "dataset/camera_timestamps.csv";
    std::string image_folder = "dataset/images";

    std::cout << "   Loading: " << csv_file << std::endl;
    auto reader = std::make_shared<CsvImageDatasetReader>(
        csv_file, image_folder, std::make_pair(0.0, 0.0), 0.0);

    std::cout << "   ✓ Loaded " << reader->getImageCount() << " image frames"
              << std::endl;

    if (reader->getImageCount() > 0) {
      std::cout << "     First timestamp: " << reader->getTimestamp(0)
                << std::endl;
      std::cout << "     Last timestamp: "
                << reader->getTimestamp(reader->getImageCount() - 1)
                << std::endl;

      double duration = reader->getTimestamp(reader->getImageCount() - 1) -
                        reader->getTimestamp(0);
      std::cout << "     Duration: " << duration << " seconds" << std::endl;

      if (reader->getImageCount() > 1) {
        double fps = reader->getImageCount() / duration;
        std::cout << "     Average FPS: " << fps << std::endl;
      }
    }
  } catch (const std::exception& e) {
    std::cout << "   ✗ Failed: " << e.what() << std::endl;
    std::cout << "   (This is expected if dataset doesn't exist)" << std::endl;
  }
  std::cout << std::endl;

  // 3. 测试 CSV IMU Dataset Reader
  std::cout << "3. Testing CsvImuDatasetReader..." << std::endl;
  try {
    std::string imu_csv = "dataset/imu_data.csv";

    std::cout << "   Loading: " << imu_csv << std::endl;
    auto reader = std::make_shared<CsvImuDatasetReader>(
        imu_csv, std::make_pair(0.0, 0.0));

    std::cout << "   ✓ Loaded " << reader->getMessageCount()
              << " IMU measurements" << std::endl;

    if (reader->getMessageCount() > 0) {
      auto first = reader->getMeasurement(0);
      auto last = reader->getMeasurement(reader->getMessageCount() - 1);

      std::cout << "     First timestamp: " << first.timestamp << std::endl;
      std::cout << "     Last timestamp: " << last.timestamp << std::endl;

      double duration = last.timestamp - first.timestamp;
      std::cout << "     Duration: " << duration << " seconds" << std::endl;

      if (duration > 0) {
        double rate = reader->getMessageCount() / duration;
        std::cout << "     Average rate: " << rate << " Hz" << std::endl;
      }

      // 显示第一个IMU样本
      std::cout << "     First sample:" << std::endl;
      std::cout << "       omega: [" << first.omega.transpose() << "]"
                << std::endl;
      std::cout << "       alpha: [" << first.alpha.transpose() << "]"
                << std::endl;
    }
  } catch (const std::exception& e) {
    std::cout << "   ✗ Failed: " << e.what() << std::endl;
    std::cout << "   (This is expected if dataset doesn't exist)" << std::endl;
  }
  std::cout << std::endl;

  // 4. 使用建议
  std::cout << "=== Usage Instructions ===" << std::endl;
  std::cout << "To use this tool, you need:" << std::endl;
  std::cout << "1. A configuration file (e.g., config_example.yaml)"
            << std::endl;
  std::cout << "2. Camera CSV file with timestamps and image filenames"
            << std::endl;
  std::cout << "3. Images in a folder" << std::endl;
  std::cout << "4. IMU CSV file with timestamps and measurements" << std::endl;
  std::cout << std::endl;
  std::cout << "Run with: " << argv[0] << " <config_file>" << std::endl;
  std::cout << std::endl;
  std::cout << "For more information, see:" << std::endl;
  std::cout << "- CSV_DATASET_FORMAT.md - Data format specification"
            << std::endl;
  std::cout << "- QUICKSTART.md - Quick start guide" << std::endl;
  std::cout << "- README_CSV.md - Complete documentation" << std::endl;
  std::cout << "- KALIBR_COMMON.md - KalibrCommon implementation details"
            << std::endl;
  std::cout << std::endl;

  return 0;
}
