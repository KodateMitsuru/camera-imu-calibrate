#ifndef KALIBR_COMMON_TARGET_EXTRACTOR_HPP
#define KALIBR_COMMON_TARGET_EXTRACTOR_HPP

#include <aslam/Time.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <aslam/cameras/GridDetector.hpp>
#include <kalibr_common/ImageDatasetReader.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <vector>

namespace kalibr {

/**
 * @brief Helper structure for multithreaded extraction tasks
 */
struct ExtractionTask {
  size_t idx;
  aslam::Time timestamp;
  cv::Mat image;

  ExtractionTask(size_t idx_, const aslam::Time& ts, const cv::Mat& img)
      : idx(idx_), timestamp(ts), image(img) {}
};

/**
 * @brief Helper structure for extraction results
 */
struct ExtractionResult {
  aslam::cameras::GridCalibrationTargetObservation observation;
  size_t idx;

  ExtractionResult(const aslam::cameras::GridCalibrationTargetObservation& obs,
                   size_t idx_)
      : observation(obs), idx(idx_) {}
};

/**
 * @brief Worker function for multithreaded corner extraction
 *
 * This function is called by each worker thread to process extraction tasks
 */
void multicoreExtractionWorker(aslam::cameras::GridDetector detector,
                               const std::vector<ExtractionTask>& tasks,
                               std::vector<ExtractionResult>& results,
                               size_t startIdx, size_t endIdx, bool clearImages,
                               bool noTransformation);

/**
 * @brief Extract calibration target corners from a dataset
 *
 * @param dataset The dataset to extract corners from
 * @param detector The grid detector to use for corner extraction
 * @param multithreading Enable multithreaded extraction
 * @param numProcesses Number of processes to use (0 = auto-detect)
 * @param clearImages Clear images from observations to save memory
 * @param noTransformation Use findTargetNoTransformation instead of findTarget
 * @return Vector of successfully extracted target observations
 */
std::vector<aslam::cameras::GridCalibrationTargetObservation>
extractCornersFromDataset(const ImageDatasetReader& dataset,
                          const aslam::cameras::GridDetector& detector,
                          bool multithreading = false,
                          unsigned int numProcesses = 0,
                          bool clearImages = true,
                          bool noTransformation = false);

}  // namespace kalibr

#endif  // KALIBR_COMMON_TARGET_EXTRACTOR_HPP
