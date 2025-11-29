#include <algorithm>
#include <atomic>
#include <chrono>
#include <format_utils.hpp>
#include <kalibr_common/TargetExtractor.hpp>
#include <opencv2/highgui.hpp>
#include <print>
#include <sm/progress/Progress.hpp>
#include <thread>

namespace kalibr {

void multicoreExtractionWorker(aslam::cameras::GridDetector detector,
                               const ImageDatasetReader* dataset,
                               const std::vector<size_t>& indices,
                               std::vector<ExtractionResult>& results,
                               bool clearImages, bool noTransformation) {
  results.reserve(indices.size());

  for (size_t idx : indices) {
    auto [timestamp, image] = dataset->getImage(idx);

    bool success = false;
    aslam::cameras::GridCalibrationTargetObservation obs;

    if (noTransformation) {
      success = detector.findTargetNoTransformation(image, timestamp, obs);
    } else {
      success = detector.findTarget(image, timestamp, obs);
    }

    if (clearImages) {
      obs.clearImage();
    }

    if (success) {
      results.emplace_back(obs, idx);
    }
  }
}

std::vector<aslam::cameras::GridCalibrationTargetObservation>
extractCornersFromDataset(const ImageDatasetReader& dataset,
                          const aslam::cameras::GridDetector& detector,
                          bool multithreading, unsigned int numProcesses,
                          bool clearImages, bool noTransformation) {
  std::vector<aslam::cameras::GridCalibrationTargetObservation>
      targetObservations;
  size_t numImages = dataset.numImages();
  std::println("Extracting corners from {} images...", numImages);
  auto iProgress = sm::progress::Progress2(numImages);
  iProgress.sample();

  if (multithreading) {
    // Auto-detect number of threads
    if (numProcesses == 0) {
      numProcesses = std::max(1u, std::thread::hardware_concurrency() - 1);
    }

    std::println("  Using {} threads for extraction", numProcesses);

    try {
      // Prepare index lists for each thread (no image loading yet)
      std::vector<std::vector<size_t>> threadIndices(numProcesses);

      // Distribute indices evenly across threads
      for (size_t i = 0; i < numImages; ++i) {
        threadIndices[i % numProcesses].push_back(i);
      }

      // Prepare result containers for each thread
      std::vector<std::vector<ExtractionResult>> threadResults(numProcesses);

      // Launch threads - each thread loads and processes its own images
      std::vector<std::thread> threads;
      threads.reserve(numProcesses);

      for (unsigned int threadIdx = 0; threadIdx < numProcesses; ++threadIdx) {
        // Create a copy of the detector for each thread
        aslam::cameras::GridDetector detectorCopy = detector;

        threads.emplace_back([detectorCopy = std::move(detectorCopy), &dataset,
                              &indices = threadIndices[threadIdx],
                              &results = threadResults[threadIdx], clearImages,
                              noTransformation]() mutable {
          multicoreExtractionWorker(std::move(detectorCopy), &dataset, indices,
                                    results, clearImages, noTransformation);
        });
      }

      // Monitor progress
      std::atomic<bool> allThreadsComplete{false};
      std::thread progressThread([&]() {
        size_t lastCompleted = 0;
        while (!allThreadsComplete) {
          size_t totalCompleted = 0;
          for (const auto& results : threadResults) {
            totalCompleted += results.size();
          }

          if (totalCompleted > lastCompleted) {
            iProgress.sample(totalCompleted - lastCompleted);
            lastCompleted = totalCompleted;
          }

          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      });

      // Wait for all threads to complete
      for (auto& thread : threads) {
        if (thread.joinable()) {
          thread.join();
        }
      }

      allThreadsComplete = true;
      if (progressThread.joinable()) {
        progressThread.join();
      }

      // Collect and sort results by index
      std::vector<ExtractionResult> allResults;
      for (const auto& results : threadResults) {
        allResults.insert(allResults.end(), results.begin(), results.end());
      }

      std::sort(allResults.begin(), allResults.end(),
                [](const ExtractionResult& a, const ExtractionResult& b) {
                  return a.idx < b.idx;
                });

      // Extract observations
      targetObservations.reserve(allResults.size());
      for (const auto& result : allResults) {
        targetObservations.push_back(result.observation);
      }

    } catch (const std::exception& e) {
      throw std::runtime_error("Exception during multithreaded extraction: " +
                               std::string(e.what()));
    }

  } else {
    // Single-threaded implementation
    for (const auto& [timestamp, image] : dataset) {
      bool success = false;
      aslam::cameras::GridCalibrationTargetObservation observation;

      if (noTransformation) {
        success =
            detector.findTargetNoTransformation(image, timestamp, observation);
      } else {
        success = detector.findTarget(image, timestamp, observation);
      }

      if (clearImages) {
        observation.clearImage();
      }

      if (success) {
        targetObservations.push_back(observation);
      }

      iProgress.sample();
    }
  }

  if (targetObservations.empty()) {
    std::println(stderr, "\r");
    throw std::runtime_error(
        "No corners could be extracted for camera " + dataset.getTopic() +
        "! Check the calibration target configuration and dataset.");
  } else {
    std::println(
        "\r  Extracted corners for {} images (of {} images)                    "
        "          ",
        targetObservations.size(), numImages);
  }

  // Close all OpenCV windows that might be open
  cv::destroyAllWindows();

  return targetObservations;
}

}  // namespace kalibr
