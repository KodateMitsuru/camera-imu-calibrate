#include <signal.h>

#include <argparse/argparse.hpp>
#include <filesystem>
#include <print>

#include "IccCalibrator.hpp"
#include "IccSensors.hpp"
#include "IccUtils.hpp"
#include "kalibr_common/ConfigReader.hpp"
#include "kalibr_common/ImageDatasetReader.hpp"
#include "kalibr_common/ImuDatasetReader.hpp"

// Signal handler for graceful shutdown
void signalHandler([[maybe_unused]] int signal) {
  std::println("");
  std::println("Shutting down! (CTRL+C)");
  std::exit(1);
}

// Parse command line arguments
void setupArgumentParser(argparse::ArgumentParser& program) {
  const std::string usage = R"(
Example usage to calibrate a camera system against an IMU using an aprilgrid.
Temporal calibration is enabled by default.

camera_imu_calibrate --dataset-name mydata --cams camchain.yaml --imu imu.yaml \
                     --target aprilgrid.yaml 

Note: Image and IMU data paths are specified in the YAML config files.

camchain.yaml: Camera system configuration (includes image folder paths)
               Output from kalibr_calibrate_cameras or manually created

example aprilgrid.yaml:       |  example imu.yaml: (ADIS16448)
    target_type: 'aprilgrid'  |      accelerometer_noise_density: 0.006  
    tagCols: 6                |      accelerometer_random_walk: 0.0002
    tagRows: 6                |      gyroscope_noise_density: 0.0004
    tagSize: 0.088            |      gyroscope_random_walk: 4.0e-06
    tagSpacing: 0.3           |      update_rate: 200.0
                              |      csv_file: /path/to/imu_data.csv
)";

  program.add_description(
      "Calibrate the spatial and temporal parameters of an IMU to a camera "
      "chain.");
  program.add_epilog(usage);

  // Dataset source
  program.add_argument("--dataset-name")
      .help(
          "[string] Dataset identifier (used for output filenames: "
          "<name>-*.yaml/pdf/csv)")
      .required()
      .nargs(1);

  program.add_argument("--time-range")
      .help(
          "[float float] Use only data within this time range [start_sec, "
          "end_sec]")
      .nargs(2)
      .scan<'g', double>();

  program.add_argument("--image-freq")
      .help(
          "[float] Target frequency for image feature extraction [hz] (0 = use "
          "all "
          "frames)")
      .scan<'g', double>();

  program.add_argument("--perform-synchronization")
      .help("[flag] Perform clock synchronization between sensors")
      .default_value(false)
      .implicit_value(true);

  // Camera system configuration
  program.add_argument("--cams")
      .help(
          "[path] Camera chain config file (YAML) - includes camera params and "
          "image "
          "folders")
      .required();

  program.add_argument("--recompute-camera-chain-extrinsics")
      .help("[flag] Recompute camera-to-camera extrinsics (for debugging only)")
      .default_value(false)
      .implicit_value(true);

  program.add_argument("--reprojection-sigma")
      .help("[float] Standard deviation of reprojection errors [pixels]")
      .default_value(1.0)
      .scan<'g', double>();

  // IMU configuration
  program.add_argument("--imu")
      .help(
          "[path...] IMU config files (YAML) - includes noise params and CSV "
          "data file "
          "path")
      .required()
      .nargs(argparse::nargs_pattern::at_least_one);

  program.add_argument("--imu-delay-by-correlation")
      .help("[flag] Estimate time delay between multiple IMUs by correlation")
      .default_value(false)
      .implicit_value(true);

  program.add_argument("--imu-models")
      .help(
          "[string...] IMU error models for each IMU: 'calibrated' (default), "
          "'scale-misalignment', 'scale-misalignment-size-effect'")
      .nargs(argparse::nargs_pattern::at_least_one);

  // Calibration target
  program.add_argument("--target")
      .help(
          "[path] Calibration target config file (YAML) - aprilgrid, "
          "checkerboard, "
          "etc.")
      .required();

  // Optimization options
  program.add_argument("--no-time-calibration")
      .help(
          "[flag] Disable temporal (time offset) calibration between camera "
          "and IMU")
      .default_value(false)
      .implicit_value(true);

  program.add_argument("--max-iter")
      .help("[int] Maximum optimization iterations")
      .default_value(30)
      .scan<'i', int>();

  program.add_argument("--recover-covariance")
      .help("[flag] Recover covariance matrix of calibration parameters (slow)")
      .default_value(false)
      .implicit_value(true);

  program.add_argument("--timeoffset-padding")
      .help(
          "[float] Maximum allowed time offset change during optimization "
          "[seconds]")
      .default_value(30e-3)
      .scan<'g', double>();

  // Output options
  program.add_argument("--show-extraction")
      .help(
          "[flag] Display calibration target detection results (disables "
          "report)")
      .default_value(false)
      .implicit_value(true);

  program.add_argument("--extraction-stepping")
      .help(
          "[flag] Step through each image during target extraction (disables "
          "report)")
      .default_value(false)
      .implicit_value(true);

  program.add_argument("--verbose")
      .help("[flag] Enable verbose output (disables report)")
      .default_value(false)
      .implicit_value(true);

  program.add_argument("--dont-show-report")
      .help("[flag] Do not display PDF report on screen after calibration")
      .default_value(false)
      .implicit_value(true);

  program.add_argument("--export-poses")
      .help("[flag] Export optimized trajectory poses to CSV file")
      .default_value(false)
      .implicit_value(true);
}

int main(int argc, char* argv[]) {
  // Setup signal handler
  signal(SIGINT, signalHandler);

  // Parse arguments
  argparse::ArgumentParser program("camera_imu_calibrate", "1.0.0",
                                   argparse::default_arguments::help);
  setupArgumentParser(program);

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception& err) {
    std::println(stderr, "Error: {}", err.what());
    return 1;
  }

  // Extract arguments
  auto dataset_name =
      program.get<std::vector<std::string>>("--dataset-name")[0];
  auto cams_yaml = program.get<std::string>("--cams");
  auto imu_yamls = program.get<std::vector<std::string>>("--imu");
  auto target_yaml = program.get<std::string>("--target");

  bool verbose = program.get<bool>("--verbose");
  bool show_extraction = program.get<bool>("--show-extraction");
  bool extraction_stepping = program.get<bool>("--extraction-stepping");
  bool no_time_calib = program.get<bool>("--no-time-calibration");
  bool recompute_extrinsics =
      program.get<bool>("--recompute-camera-chain-extrinsics");
  bool recover_cov = program.get<bool>("--recover-covariance");
  bool dont_show_report = program.get<bool>("--dont-show-report");
  bool export_poses = program.get<bool>("--export-poses");
  bool estimate_imu_delay = program.get<bool>("--imu-delay-by-correlation");

  int max_iter = program.get<int>("--max-iter");
  double reprojection_sigma =
      program.get<double>("--reprojection-sigma");  // TODO: Use in IccCamera
  double timeoffset_padding = program.get<double>("--timeoffset-padding");

  // Get time range
  std::pair<double, double> time_range = {0.0, 0.0};
  if (program.is_used("--time-range")) {
    auto times = program.get<std::vector<double>>("--time-range");
    time_range = {times[0], times[1]};
  }

  // Get target frequency for image extraction
  double image_freq = 0.0;
  if (program.is_used("--image-freq")) {
    image_freq = program.get<double>("--image-freq");
  }

  // Get IMU models
  std::vector<std::string> imu_models;
  if (program.is_used("--imu-models")) {
    imu_models = program.get<std::vector<std::string>>("--imu-models");
  } else {
    // Maintain backwards compatibility: default to 'calibrated' for single IMU
    if (imu_yamls.size() == 1) {
      imu_models = {"calibrated"};
    }
  }

  // Validate IMU models
  if (!imu_models.empty() && imu_yamls.size() != imu_models.size()) {
    std::println(stderr, "Error: Number of IMU yamls and models must match.");
    return 2;
  }

  // Disable report display if showing extraction or verbose
  if (show_extraction || extraction_stepping || verbose) {
    dont_show_report = true;
  }

  // Print configuration
  std::println("");
  std::println("=========================================");
  std::println("  Camera-IMU Calibration");
  std::println("=========================================");
  std::println("Dataset: {}", dataset_name);
  std::println("Camera chain: {}", cams_yaml);
  std::println("Target: {}", target_yaml);
  std::println("IMUs: {}", imu_yamls.size());
  for (size_t i = 0; i < imu_yamls.size(); ++i) {
    std::println("  [{}] {} (model: {})", i, imu_yamls[i],
                 imu_models.empty() ? "auto" : imu_models[i]);
  }
  std::println("Max iterations: {}", max_iter);
  std::println("Temporal calibration: {}", !no_time_calib);
  if (time_range.first > 0.0 || time_range.second > 0.0) {
    std::println("Time range: [{}, {}] s", time_range.first, time_range.second);
  }
  if (image_freq > 0.0) {
    std::println("Image extraction frequency: {} Hz", image_freq);
  }
  std::println("");

  try {
    // Load calibration target configuration
    std::println("Initializing calibration target:");
    std::println("  Loading: {}", target_yaml);
    auto targetConfig = kalibr::CalibrationTargetParameters(target_yaml);
    targetConfig.printDetails();
    std::println("");

    // Load camera chain configuration
    std::println("Initializing camera chain:");
    std::println("  Loading: {}", cams_yaml);
    auto chainConfig = kalibr::CameraChainParameters(cams_yaml);
    size_t numCameras = chainConfig.numCameras();
    std::println("  Number of cameras: {}", numCameras);

    // Create image dataset readers for each camera
    std::vector<kalibr::ImageDatasetReader> imageDatasets;
    for (size_t i = 0; i < numCameras; ++i) {
      auto camParams = chainConfig.getCameraParameters(i);
      std::string imageFolder = camParams->getImageFolder();
      std::println("  Camera {}: {}", i, imageFolder);

      imageDatasets.emplace_back(imageFolder, time_range, image_freq);
    }
    std::println("");

    // Create camera chain
    auto camChain = std::make_shared<kalibr::IccCameraChain>(
        chainConfig, targetConfig, imageDatasets);

    // Initialize IMUs
    std::println("Initializing IMUs:");
    std::vector<std::shared_ptr<kalibr::IccImu>> imus;

    for (size_t i = 0; i < imu_yamls.size(); ++i) {
      const auto& imu_yaml = imu_yamls[i];
      const auto& imu_model = imu_models.empty() ? "calibrated" : imu_models[i];

      std::println("  Loading IMU {}: {} ({})", i, imu_yaml, imu_model);

      // Load IMU configuration
      auto imuConfig = kalibr::ImuParameters(imu_yaml);
      std::string csvFile = imuConfig.getCsvFile();
      std::println("    CSV file: {}", csvFile);

      // Create IMU dataset reader
      kalibr::ImuDatasetReader imuDataset(csvFile, time_range);

      bool is_reference_imu = (i == 0);
      int imuNr = static_cast<int>(i);

      // Create IMU based on model type
      if (imu_model == "calibrated") {
        imus.push_back(std::make_shared<kalibr::IccImu>(
            imuConfig, imuDataset, is_reference_imu, estimate_imu_delay,
            imuNr));
        std::println("    Model: Calibrated");
      } else if (imu_model == "scale-misalignment") {
        imus.push_back(std::make_shared<kalibr::IccScaledMisalignedImu>(
            imuConfig, imuDataset, is_reference_imu, estimate_imu_delay,
            imuNr));
        std::println("    Model: Scale-Misalignment");
      } else if (imu_model == "scale-misalignment-size-effect") {
        imus.push_back(
            std::make_shared<kalibr::IccScaledMisalignedSizeEffectImu>(
                imuConfig, imuDataset, is_reference_imu, estimate_imu_delay,
                imuNr));
        std::println("    Model: Scale-Misalignment-Size-Effect");
      } else {
        std::println(stderr, "Error: Unsupported IMU model: {}", imu_model);
        return 2;
      }

      imus.back()->getImuConfig().printDetails();
    }
    std::println("");

    // Create calibrator
    std::println("Creating calibrator...");
    kalibr::IccCalibrator calibrator;

    // Register sensors with calibrator
    calibrator.registerCamChain(camChain);
    for (auto& imu : imus) {
      calibrator.registerImu(imu);
    }

    // Find orientation priors for non-reference IMUs
    if (imus.size() > 1 && estimate_imu_delay) {
      std::println("Finding orientation priors for non-reference IMUs...");
      for (size_t i = 1; i < imus.size(); ++i) {
        imus[i]->findOrientationPrior(*imus[0]);
      }
    }

    // Build problem
    std::println("");
    std::println("Building the problem");
    calibrator.buildProblem(6,                      // splineOrder
                            100,                    // poseKnotsPerSecond
                            50,                     // biasKnotsPerSecond
                            false,                  // doPoseMotionError
                            0.0,                    // mrTranslationVariance
                            0.0,                    // mrRotationVariance
                            true,                   // doBiasMotionError
                            -1,                     // blakeZisserCam
                            -1.0,                   // huberAccel
                            -1.0,                   // huberGyro
                            no_time_calib,          // noTimeCalibration
                            !recompute_extrinsics,  // noChainExtrinsics
                            max_iter,               // maxIterations
                            1.0,                    // gyroNoiseScale
                            1.0,                    // accelNoiseScale
                            timeoffset_padding,     // timeOffsetPadding
                            verbose                 // verbose
    );

    // Print initial statistics
    std::println("");
    std::println("Before Optimization");
    std::println("===================");
    kalibr::printErrorStatistics(calibrator);

    // Optimize
    std::println("");
    std::println("Optimizing...");
    calibrator.optimize(nullptr, max_iter, recover_cov);

    // Print results
    std::println("");
    std::println("After Optimization (Results)");
    std::println("============================");
    kalibr::printErrorStatistics(calibrator);
    kalibr::printResults(calibrator, recover_cov);
    std::println("");

    // Save results
    std::filesystem::path dataset_path(dataset_name);
    std::string basetag = dataset_path.stem().string();

    // Save camera chain calibration
    std::string yaml_camchain = basetag + "-camchain-imucam.yaml";
    std::println("Results written to:");
    std::println("  Saving camera chain calibration: {}", yaml_camchain);
    calibrator.saveCamChainParametersYaml(yaml_camchain);

    // Save IMU calibration
    std::string yaml_imu = basetag + "-imu.yaml";
    std::println("  Saving imu calibration: {}", yaml_imu);
    calibrator.saveImuSetParametersYaml(yaml_imu);

    // Save detailed results
    std::string result_txt = basetag + "-results-imucam.txt";
    std::println("  Detailed results: {}", result_txt);
    kalibr::saveResultTxt(calibrator, result_txt);

    // Generate report
    std::println("");
    std::println("Generating result report...");
    std::string report_pdf = basetag + "-report-imucam.pdf";
    kalibr::generateReport(calibrator, report_pdf, !dont_show_report);
    std::println("  Report written to: {}", report_pdf);

    // Export poses if requested
    if (export_poses) {
      std::println("");
      std::println("Exporting poses...");
      std::string poses_csv = basetag + "-poses-imucam-imu0.csv";
      kalibr::exportPoses(calibrator, poses_csv);
      std::println("  Poses written to: {}", poses_csv);
    }

    std::println("");
    std::println("Calibration completed successfully!");

  } catch (const std::exception& e) {
    std::println(stderr, "Exception: {}", e.what());
    return -1;
  }

  return 0;
}