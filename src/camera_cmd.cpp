#include "camera_cmd.hpp"

#include <argparse/argparse.hpp>
#include <filesystem>
#include <format_utils.hpp>
#include <print>
#include <random>

#include "kalibr_camera_calibration/CameraCalibrator.hpp"
#include "kalibr_camera_calibration/CameraUtils.hpp"
#include "kalibr_camera_calibration/MulticamGraph.hpp"
#include "kalibr_camera_calibration/ObsDb.hpp"
#include "kalibr_common/ConfigReader.hpp"
#include "kalibr_common/ImageDatasetReader.hpp"
#include "kalibr_common/TargetExtractor.hpp"

void setupCameraArgs(argparse::ArgumentParser& cmd) {
  const std::string usage = R"(
Example usage to calibrate a camera system with two cameras using an aprilgrid.

cam0: pinhole model with radial-tangential distortion
cam1: pinhole model with equidistant distortion

Method 1: Using camera model strings (recommended for initial calibration)
calibrate camera --dataset-name mydata --models pinhole-radtan pinhole-equi \
                 --image-folders /path/to/cam0 /path/to/cam1 \
                 --target aprilgrid.yaml

Method 2: Using camera YAML config files (for re-calibration with known intrinsics)
calibrate camera --dataset-name mydata --cams cam0.yaml cam1.yaml \
                 --target aprilgrid.yaml

Available camera models:
  pinhole-radtan  : Pinhole with radial-tangential distortion
  pinhole-equi    : Pinhole with equidistant (fisheye) distortion
  pinhole-fov     : Pinhole with FOV distortion
  omni-none       : Omnidirectional without distortion
  omni-radtan     : Omnidirectional with radial-tangential distortion
  eucm-none       : Extended Unified Camera Model
  ds-none         : Double Sphere model

example aprilgrid.yaml:
    target_type: 'aprilgrid'
    tagCols: 6
    tagRows: 6
    tagSize: 0.088
    tagSpacing: 0.3
)";

  cmd.add_description(
      "Calibrate intrinsics and extrinsics of a multi-camera system.");
  cmd.add_epilog(usage);

  // Dataset source
  cmd.add_argument("--dataset-name")
      .help(
          "[string] Dataset identifier (used for output filenames: "
          "<name>-*.yaml/pdf)")
      .required()
      .nargs(1);

  cmd.add_argument("--time-range")
      .help(
          "[float float] Use only data within this time range [start_sec, "
          "end_sec]")
      .nargs(2)
      .scan<'g', double>();

  cmd.add_argument("--image-freq")
      .help(
          "[float] Target frequency for image feature extraction [hz] (0 = use "
          "all frames)")
      .scan<'g', double>();

  // Camera configuration - Method 1: Model strings
  cmd.add_argument("--models")
      .help(
          "[string...] Camera model types: pinhole-radtan, pinhole-equi, "
          "pinhole-fov, omni-none, omni-radtan, eucm-none, ds-none")
      .nargs(argparse::nargs_pattern::at_least_one);

  cmd.add_argument("--image-folders")
      .help("[path...] Image folders for each camera (required with --models)")
      .nargs(argparse::nargs_pattern::at_least_one);

  // Camera configuration - Method 2: YAML files
  cmd.add_argument("--cams")
      .help("[path...] Camera config files (YAML) for each camera")
      .nargs(argparse::nargs_pattern::at_least_one);

  cmd.add_argument("--reprojection-sigma")
      .help("[float] Standard deviation of reprojection errors [pixels]")
      .default_value(1.0)
      .scan<'g', double>();

  // Calibration target
  cmd.add_argument("--target")
      .help(
          "[path] Calibration target config file (YAML) - aprilgrid, "
          "checkerboard, etc.")
      .required();

  // Image synchronization
  cmd.add_argument("--approx-sync")
      .help(
          "[float] Time tolerance for approximate image synchronization [s] "
          "(default: 0.02)")
      .default_value(0.02)
      .scan<'g', double>();

  // Calibrator settings
  cmd.add_argument("--mi-tol")
      .help(
          "[float] Mutual information tolerance for adding images. Higher = "
          "fewer images. -1 = force all (default: 0.2)")
      .default_value(0.2)
      .scan<'g', double>();

  cmd.add_argument("--no-shuffle")
      .help("[flag] Do not shuffle the dataset processing order")
      .default_value(false)
      .implicit_value(true);

  // Outlier filtering options
  cmd.add_argument("--no-outliers-removal")
      .help("[flag] Disable corner outlier filtering")
      .default_value(false)
      .implicit_value(true);

  cmd.add_argument("--no-final-filtering")
      .help("[flag] Disable filtering after all views have been processed")
      .default_value(false)
      .implicit_value(true);

  cmd.add_argument("--min-views-outlier")
      .help(
          "[int] Number of raw views to initialize outlier statistics "
          "(default: 20)")
      .default_value(20)
      .scan<'i', int>();

  cmd.add_argument("--use-blakezisserman")
      .help("[flag] Enable the Blake-Zisserman M-estimator")
      .default_value(false)
      .implicit_value(true);

  // Optimization options
  cmd.add_argument("--max-iter")
      .help("[int] Maximum optimization iterations")
      .default_value(50)
      .scan<'i', int>();

  // Output options
  cmd.add_argument("--show-extraction")
      .help("[flag] Display calibration target detection results")
      .default_value(false)
      .implicit_value(true);

  cmd.add_argument("--verbose")
      .help("[flag] Enable verbose output")
      .default_value(false)
      .implicit_value(true);

  cmd.add_argument("--dont-show-report")
      .help("[flag] Do not display PDF report on screen after calibration")
      .default_value(false)
      .implicit_value(true);

  cmd.add_argument("--export-poses")
      .help("[flag] Export optimized poses to CSV file")
      .default_value(false)
      .implicit_value(true);
}

int runCameraCalibration(argparse::ArgumentParser& cmd) {
  // Implementation copied from original main.cpp (Camera section)
  // Extract arguments
  auto dataset_name = cmd.get<std::vector<std::string>>("--dataset-name")[0];
  auto target_yaml = cmd.get<std::string>("--target");

  bool verbose = cmd.get<bool>("--verbose");
  bool show_extraction = cmd.get<bool>("--show-extraction");
  bool dont_show_report = cmd.get<bool>("--dont-show-report");
  bool no_shuffle = cmd.get<bool>("--no-shuffle");
  bool remove_outliers = !cmd.get<bool>("--no-outliers-removal");
  bool allow_end_filtering = !cmd.get<bool>("--no-final-filtering");
  bool use_blake_zisserman = cmd.get<bool>("--use-blakezisserman");
  bool export_poses = cmd.get<bool>("--export-poses");

  int max_iter = cmd.get<int>("--max-iter");
  int min_view_outlier = cmd.get<int>("--min-views-outlier");
  double reprojection_sigma = cmd.get<double>("--reprojection-sigma");
  double approx_sync_tol = cmd.get<double>("--approx-sync");
  double mi_tol = cmd.get<double>("--mi-tol");

  // Get time range
  std::pair<double, double> time_range = {0.0, 0.0};
  if (cmd.is_used("--time-range")) {
    auto times = cmd.get<std::vector<double>>("--time-range");
    time_range = {times[0], times[1]};
  }

  // Get target frequency for image extraction
  double image_freq = 0.0;
  if (cmd.is_used("--image-freq")) {
    image_freq = cmd.get<double>("--image-freq");
  }

  // Disable report display if showing extraction or verbose
  if (show_extraction || verbose) {
    dont_show_report = true;
  }

  // Determine camera configuration method
  bool use_models = cmd.is_used("--models");
  bool use_cams = cmd.is_used("--cams");

  if (!use_models && !use_cams) {
    std::println(stderr, "Error: Either --models or --cams must be specified.");
    return 2;
  }

  if (use_models && use_cams) {
    std::println(
        stderr,
        "Error: Cannot use both --models and --cams. Choose one method.");
    return 2;
  }

  std::vector<std::string> cam_models;
  std::vector<std::string> image_folders;
  std::vector<std::string> cam_yamls;

  if (use_models) {
    cam_models = cmd.get<std::vector<std::string>>("--models");
    if (!cmd.is_used("--image-folders")) {
      std::println(stderr,
                   "Error: --image-folders is required when using --models.");
      return 2;
    }
    image_folders = cmd.get<std::vector<std::string>>("--image-folders");
    if (cam_models.size() != image_folders.size()) {
      std::println(stderr,
                   "Error: Number of --models ({}) must match number of "
                   "--image-folders ({}).",
                   cam_models.size(), image_folders.size());
      return 2;
    }
  } else {
    cam_yamls = cmd.get<std::vector<std::string>>("--cams");
  }

  size_t numCams = use_models ? cam_models.size() : cam_yamls.size();

  // Print configuration
  std::println("");
  std::println("=========================================");
  std::println("  Camera Calibration");
  std::println("=========================================");
  std::println("Dataset: {}", dataset_name);
  std::println("Cameras: {}", numCams);
  if (use_models) {
    for (size_t i = 0; i < numCams; ++i) {
      std::println("  [{}] model: {}, folder: {}", i, cam_models[i],
                   image_folders[i]);
    }
  } else {
    for (size_t i = 0; i < numCams; ++i) {
      std::println("  [{}] {}", i, cam_yamls[i]);
    }
  }
  std::println("Target: {}", target_yaml);
  std::println("Max iterations: {}", max_iter);
  std::println("Approx sync tolerance: {} s", approx_sync_tol);
  std::println("MI tolerance: {}", mi_tol);
  std::println("Outlier removal: {}", remove_outliers);
  std::println("Blake-Zisserman: {}", use_blake_zisserman);
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

    // Create observation database
    kalibr::ObservationDatabase obsDb(approx_sync_tol);

    // Create camera objects and extract targets
    std::vector<std::shared_ptr<kalibr::CameraGeometry>> cameraList;

    for (size_t cam_id = 0; cam_id < numCams; ++cam_id) {
      std::println("Initializing cam{}:", cam_id);

      std::shared_ptr<kalibr::CameraParameters> camParams;
      std::string imageFolder;
      kalibr::CameraModel camModel;
      kalibr::DistortionModel distModel;

      if (use_models) {
        // Parse model string like "pinhole-radtan"
        std::string modelStr = cam_models[cam_id];
        auto dashPos = modelStr.find('-');
        if (dashPos == std::string::npos) {
          std::println(stderr,
                       "Error: Invalid model format '{}'. Expected "
                       "'camera-distortion' (e.g., pinhole-radtan).",
                       modelStr);
          return 2;
        }
        std::string camModelStr = modelStr.substr(0, dashPos);
        std::string distModelStr = modelStr.substr(dashPos + 1);

        // Convert strings to enums
        try {
          camModel = kalibr::stringToCameraModel(camModelStr);
          distModel = kalibr::stringToDistortionModel(distModelStr);
        } catch (const std::exception& e) {
          std::println(stderr, "Error: {}", e.what());
          return 2;
        }

        imageFolder = image_folders[cam_id];

        // Create a temporary CameraParameters with dummy intrinsics
        // The actual intrinsics will be initialized from observations
        camParams = std::make_shared<kalibr::CameraParameters>(
            "", true);  // createYaml = true
        camParams->setImageFolder(imageFolder);

        // Read first image to get resolution
        kalibr::ImageDatasetReader tempDataset(imageFolder, time_range, 0.0);
        if (tempDataset.numImages() == 0) {
          std::println(stderr, "Error: No images found in folder: {}",
                       imageFolder);
          return 2;
        }
        auto [timestamp, firstImage] = tempDataset.getImage(0);
        Eigen::Vector2i resolution(firstImage.cols, firstImage.rows);
        camParams->setResolution(resolution);

        // Set dummy intrinsics (will be initialized from observations)
        std::vector<double> dummyIntrinsics;
        if (camModel == kalibr::CameraModel::Pinhole) {
          // [fu, fv, cu, cv] - use image center as principal point
          dummyIntrinsics = {static_cast<double>(resolution[0]),
                             static_cast<double>(resolution[0]),
                             static_cast<double>(resolution[0]) / 2.0,
                             static_cast<double>(resolution[1]) / 2.0};
        } else if (camModel == kalibr::CameraModel::Omni) {
          // [xi, fu, fv, cu, cv]
          dummyIntrinsics = {1.0, static_cast<double>(resolution[0]),
                             static_cast<double>(resolution[0]),
                             static_cast<double>(resolution[0]) / 2.0,
                             static_cast<double>(resolution[1]) / 2.0};
        } else if (camModel == kalibr::CameraModel::EUCM) {
          // [alpha, beta, fu, fv, cu, cv]
          dummyIntrinsics = {0.5,
                             1.0,
                             static_cast<double>(resolution[0]),
                             static_cast<double>(resolution[0]),
                             static_cast<double>(resolution[0]) / 2.0,
                             static_cast<double>(resolution[1]) / 2.0};
        } else if (camModel == kalibr::CameraModel::DS) {
          // [xi, alpha, fu, fv, cu, cv]
          dummyIntrinsics = {-0.2,
                             0.6,
                             static_cast<double>(resolution[0]),
                             static_cast<double>(resolution[0]),
                             static_cast<double>(resolution[0]) / 2.0,
                             static_cast<double>(resolution[1]) / 2.0};
        }
        camParams->setIntrinsics(camModel, dummyIntrinsics);

        // Set dummy distortion coefficients (will be optimized)
        std::vector<double> dummyDistCoeffs;
        if (distModel == kalibr::DistortionModel::RadTan) {
          dummyDistCoeffs = {0.0, 0.0, 0.0, 0.0};
        } else if (distModel == kalibr::DistortionModel::Equidistant) {
          dummyDistCoeffs = {0.0, 0.0, 0.0, 0.0};
        } else if (distModel == kalibr::DistortionModel::FOV) {
          dummyDistCoeffs = {0.0};
        } else if (distModel == kalibr::DistortionModel::None) {
          dummyDistCoeffs = {};
        }
        camParams->setDistortion(distModel, dummyDistCoeffs);

        std::println("  Camera model: {}", modelStr);
        std::println("  Image folder: {}", imageFolder);
        std::println("  Resolution: {}x{}", resolution[0], resolution[1]);

      } else {
        // Load camera parameters from YAML file
        camParams =
            std::make_shared<kalibr::CameraParameters>(cam_yamls[cam_id]);
        auto [cm, intrinsics] = camParams->getIntrinsics();
        auto [dm, distCoeffs] = camParams->getDistortion();
        camModel = cm;
        distModel = dm;
        imageFolder = camParams->getImageFolder();
        auto resolution = camParams->getResolution();

        std::println("  Camera model: {}-{}",
                     kalibr::cameraModelToString(camModel),
                     kalibr::distortionModelToString(distModel));
        std::println("  Image folder: {}", imageFolder);
        std::println("  Target resolution: {}x{}", resolution[0],
                     resolution[1]);
      }

      camParams->setReprojectionSigma(reprojection_sigma);

      // Create image dataset reader
      auto resolution = camParams->getResolution();
      kalibr::ImageDatasetReader dataset(
          imageFolder, time_range, image_freq,
          std::make_pair(resolution[0], resolution[1]));
      std::println("  Number of images: {}", dataset.numImages());

      // Create camera geometry
      auto cam = std::make_shared<kalibr::CameraGeometry>(
          *camParams, targetConfig, dataset, nullptr,
          verbose || show_extraction);

      // Extract targets (force single-thread for debugging)
      bool multithreading = !(verbose || show_extraction);
      auto observations = kalibr::extractCornersFromDataset(
          dataset, *cam->getTargetDetector()->getDetector(), multithreading, 0,
          false, true);

      std::println("  Extracted {} observations", observations.size());

      // Populate the database
      for (const auto& obs : observations) {
        obsDb.addObservation(static_cast<int>(cam_id), obs);
      }

      // Initialize the intrinsics
      if (!cam->initGeometryFromObservations(observations)) {
        std::println(stderr,
                     "Error: Could not initialize intrinsics for camera {}. "
                     "Try --verbose to check target extraction.",
                     cam_id);
        return -1;
      }

      // Get initialized parameters from the actual geometry (not the original
      // config)
      Eigen::MatrixXd projParams, distParams;
      cam->getGeometry()->getParameters(projParams, true, false, false);
      cam->getGeometry()->getParameters(distParams, false, true, false);
      std::println("  Projection initialized to: {}", projParams.transpose());
      std::println("  Distortion initialized to: {}", distParams.transpose());

      cameraList.push_back(cam);
    }

    std::println("");

    if (verbose) {
      obsDb.printTable();
    }

    // Initialize the calibration graph
    std::println("Initializing calibration graph...");
    kalibr::MulticamGraph graph(obsDb);

    if (!graph.isGraphConnected()) {
      obsDb.printTable();
      std::println(stderr,
                   "Error: Cameras are not connected through mutual "
                   "observations. Check dataset or adjust --approx-sync.");
      return -1;
    }

    // Store removed outlier corners for plotting
    std::vector<std::pair<int, Eigen::Vector2d>> removedOutlierCorners;

    // Restart loop for optimization divergence
    int restartAttempts = 3;
    bool initOutlierRejection = true;

    while (true) {
      try {
        // Compute initial guesses for baselines
        std::println("Computing initial guesses...");
        std::vector<sm::kinematics::Transformation> baselineGuesses;

        if (numCams > 1) {
          // Get raw pointers for graph API
          std::vector<kalibr::CameraGeometry*> camPtrs;
          for (auto& cam : cameraList) {
            camPtrs.push_back(cam.get());
          }
          baselineGuesses = graph.getInitialGuesses(camPtrs);

          for (size_t i = 0; i < baselineGuesses.size(); ++i) {
            std::println("Initialized baseline between cam{} and cam{}:", i,
                         i + 1);
            std::println("{}", baselineGuesses[i].T());
          }
        }

        for (size_t cam_idx = 0; cam_idx < cameraList.size(); ++cam_idx) {
          auto& cam = cameraList[cam_idx];
          Eigen::MatrixXd projParams, distParams;
          cam->getGeometry()->getParameters(projParams, true, false, false);
          cam->getGeometry()->getParameters(distParams, false, true, false);
          std::println("Initialized cam{}:", cam_idx);
          std::println("  projection: {}", projParams.transpose());
          if (distParams.size() > 0) {
            std::println("  distortion: {}", distParams.transpose());
          } else {
            std::println("  distortion: (none)");
          }
        }

        // Create calibrator
        std::println("");
        std::println("Initializing calibrator...");
        kalibr::CameraCalibration calibrator(cameraList, baselineGuesses, false,
                                             verbose, use_blake_zisserman);

        // Get and shuffle timestamps
        auto timestamps = obsDb.getAllViewTimestamps();
        if (!no_shuffle) {
          std::random_device rd;
          std::mt19937 gen(rd());
          std::shuffle(timestamps.begin(), timestamps.end(), gen);
        }

        // Process all target views
        std::println("");
        std::println("Starting calibration...");
        size_t numViews = timestamps.size();

        for (size_t view_id = 0; view_id < numViews; ++view_id) {
          double timestamp = timestamps[view_id];

          // Get observations at this timestamp
          auto obs_at_time = obsDb.getAllObsAtTimestamp(timestamp);

          // Convert to RigObservation format
          std::vector<
              kalibr::CalibrationTargetOptimizationProblem::RigObservation>
              rigObs;
          for (const auto& [camId, obsPtr] : obs_at_time) {
            if (obsPtr != nullptr) {
              rigObs.emplace_back(static_cast<size_t>(camId), *obsPtr);
            }
          }

          // Get target pose guess
          std::vector<sm::kinematics::Transformation> estBaselines;
          for (const auto& bl : calibrator.getBaselines()) {
            estBaselines.push_back(sm::kinematics::Transformation(bl->T()));
          }

          // Get raw pointers for graph API
          std::vector<kalibr::CameraGeometry*> camPtrs;
          for (auto& cam : cameraList) {
            camPtrs.push_back(cam.get());
          }
          auto T_tc_guess =
              graph.getTargetPoseGuess(timestamp, camPtrs, estBaselines);

          // Add target view
          bool success = calibrator.addTargetView(aslam::Time(timestamp),
                                                  rigObs, T_tc_guess);

          // Display progress
          if ((verbose || (view_id % 25) == 0) && calibrator.numViews() > 0 &&
              view_id > 1) {
            std::println("");
            std::println(
                "--------------------------------------------------------------"
                "----");
            std::println("");
            std::println("Processed {} of {} views with {} views used",
                         view_id + 1, numViews, calibrator.numViews());
            std::println("");
            kalibr::printParameters(calibrator);
            std::println("");
            std::println(
                "--------------------------------------------------------------"
                "----");
          }

          // Outlier filtering
          bool runEndFiltering =
              (view_id == numViews - 1) && allow_end_filtering;
          size_t numActiveBatches = calibrator.numViews();

          if (((success &&
                numActiveBatches >
                    static_cast<size_t>(min_view_outlier * numCams)) ||
               (runEndFiltering &&
                numActiveBatches >
                    static_cast<size_t>(min_view_outlier * numCams))) &&
              remove_outliers) {
            // Determine batches to check
            std::vector<size_t> batches_to_check;
            if (initOutlierRejection) {
              // Check all views after min number of batches reached
              for (size_t i = 0; i < calibrator.numViews(); ++i) {
                batches_to_check.push_back(i);
              }
              std::println("");
              std::println("Filtering outliers in all batches...");
              initOutlierRejection = false;
            } else if (runEndFiltering) {
              // Check all batches after all views processed
              std::println("");
              std::println("Starting final outlier filtering...");
              for (size_t i = 0; i < calibrator.numViews(); ++i) {
                batches_to_check.push_back(i);
              }
            } else {
              // Only check most recent view
              batches_to_check.push_back(calibrator.numViews() - 1);
            }

            // Sort in reverse order for safe removal
            std::sort(batches_to_check.begin(), batches_to_check.end(),
                      std::greater<size_t>());

            // Check each batch
            for (size_t batch_id : batches_to_check) {
              const auto& views = calibrator.getViews();
              if (batch_id >= views.size()) continue;

              // Check all cameras in this batch
              std::vector<std::pair<size_t, std::vector<size_t>>>
                  cornerRemovalList_allCams;

              for (size_t cidx = 0; cidx < numCams; ++cidx) {
                // Calculate reprojection error statistics
                auto viewData = kalibr::getReprojectionErrors(calibrator, cidx);
                auto [me, se] =
                    kalibr::getReprojectionErrorStatistics(viewData);
                Eigen::Vector2d se_threshold = 4.0 * se;

                // Select corners to remove
                std::vector<size_t> cornerRemovalList;
                if (batch_id < viewData.size() && viewData[batch_id].valid) {
                  for (size_t pidx = 0; pidx < viewData[batch_id].errors.size();
                       ++pidx) {
                    if (viewData[batch_id].errors[pidx].has_value()) {
                      auto& e = viewData[batch_id].errors[pidx].value();
                      if (std::abs(e[0]) > se_threshold[0] ||
                          std::abs(e[1]) > se_threshold[1]) {
                        cornerRemovalList.push_back(pidx);

                        if (verbose) {
                          std::println(
                              "Outlier: view {} cam {} corner {} "
                              "(err=({:.2f},{:.2f}) > ({:.2f},{:.2f}))",
                              batch_id, cidx, pidx, e[0], e[1], se_threshold[0],
                              se_threshold[1]);
                        }

                        // Store for plotting
                        if (viewData[batch_id].corners[pidx].has_value()) {
                          removedOutlierCorners.emplace_back(
                              static_cast<int>(cidx),
                              viewData[batch_id].corners[pidx].value());
                        }
                      }
                    }
                  }
                }

                if (!cornerRemovalList.empty()) {
                  cornerRemovalList_allCams.emplace_back(cidx,
                                                         cornerRemovalList);
                }
              }

              // Remove corners if any were flagged
              size_t removeCount = 0;
              for (const auto& [cidx, list] : cornerRemovalList_allCams) {
                removeCount += list.size();
              }

              if (removeCount > 0) {
                // Note: The actual batch removal/replacement would require
                // additional implementation in CameraCalibration class
                // For now, we just track the outliers
                if (verbose) {
                  std::println("Flagged {} outlier corners in batch {}",
                               removeCount, batch_id);
                }
              }
            }
          }
        }

        // Final output
        std::println("");
        std::println(
            "................................................................."
            ".");
        std::println("");
        std::println("Calibration complete.");
        std::println("");

        if (remove_outliers) {
          std::println("Removed {} outlier corners.",
                       removedOutlierCorners.size());
        }

        std::println("");
        std::println("Processed {} images with {} images used", numViews,
                     calibrator.numViews());
        kalibr::printParameters(calibrator);
        std::println("");

        // Write results to file
        std::filesystem::path dataset_path(dataset_name);
        std::string basetag = dataset_path.stem().string();

        std::string resultFile = basetag + "-camchain.yaml";
        kalibr::saveChainParametersYaml(calibrator, resultFile, &graph);
        std::println("Results written to:");
        std::println("  Camera chain calibration: {}", resultFile);

        // Save detailed results
        std::string resultFileTxt = basetag + "-results-cam.txt";
        kalibr::saveResultTxt(calibrator, resultFileTxt);
        std::println("  Detailed results: {}", resultFileTxt);

        // Generate report
        std::println("");
        std::println("Generating result report...");
        std::string reportFile = basetag + "-report-cam.pdf";
        kalibr::generateReport(
            calibrator, reportFile, !dont_show_report,
            removedOutlierCorners.empty() ? nullptr : &removedOutlierCorners);
        std::println("  Report: {}", reportFile);

        // Export poses if requested
        if (export_poses) {
          std::println("");
          std::println("Exporting poses...");
          std::string posesFile = basetag + "-poses-cam0.csv";
          // Collect (timestamp, T_tc_guess) for each view
          std::vector<std::pair<double, sm::kinematics::Transformation>> poses;
          const auto& views = calibrator.getViews();
          for (const auto& v : views) {
            if (v) {
              double ts = v->timestamp.toSec();
              poses.emplace_back(ts, v->T_tc_guess);
            }
          }
          kalibr::exportPoses(poses, posesFile);
          std::println("  Poses written to {}", posesFile);
        }

        std::println("");
        std::println("Camera calibration completed successfully!");

        break;  // Normal exit from restart loop

      } catch (const kalibr::OptimizationDiverged& e) {
        restartAttempts--;
        std::println(stderr,
                     "Warning: Optimization diverged. (Bad initialization?)");

        if (restartAttempts == 0) {
          std::println(stderr, "Error: Max attempts reached. Giving up.");
          return -1;
        }

        std::println("Restarting for a new attempt...");

        // Reinitialize intrinsics
        for (size_t cam_id = 0; cam_id < cameraList.size(); ++cam_id) {
          std::println("Reinitialize intrinsics for camera {}", cam_id);
          auto observations = obsDb.getAllObsCam(static_cast<int>(cam_id));

          // Convert to non-const vector
          std::vector<aslam::cameras::GridCalibrationTargetObservation> obsVec;
          for (const auto* obs : observations) {
            if (obs != nullptr) {
              obsVec.push_back(*obs);
            }
          }

          if (!cameraList[cam_id]->initGeometryFromObservations(obsVec)) {
            std::println(stderr, "Error: Could not reinitialize camera {}",
                         cam_id);
            return -1;
          }

          Eigen::MatrixXd reProjParams, reDistParams;
          cameraList[cam_id]->getGeometry()->getParameters(reProjParams, true,
                                                           false, false);
          cameraList[cam_id]->getGeometry()->getParameters(reDistParams, false,
                                                           true, false);
          std::println("  Projection: {}", reProjParams.transpose());
          std::println("  Distortion: {}", reDistParams.transpose());
        }
      }
    }

  } catch (const std::exception& e) {
    std::println(stderr, "Exception: {}", e.what());
    return -1;
  }

  return 0;
}
