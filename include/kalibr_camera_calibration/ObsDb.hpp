#ifndef KALIBR_CAMERA_CALIBRATION_OBSDB_HPP
#define KALIBR_CAMERA_CALIBRATION_OBSDB_HPP

#include <aslam/Time.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <cstddef>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

namespace kalibr {

// simple data structure that stores all observations for a multi-cam system
// and can approx. sync observations
//
// target view table in the form of:
// -----------------------------------------------------------------
// |           |         cam0        |  ...  |         camN        |
// | timestamp |---------------------|-------|---------------------|
// |           | obs_id | corner_ids |       | obs_id | corner_ids |
// -----------------------------------------------------------------
// |   float   |  int   | list(int)  |       |  int   | list(int)  |
// -----------------------------------------------------------------
//
class ObservationDatabase {
 public:
  ObservationDatabase(double max_delta_approxsync = 0.0);

  // Add an observation from camera `cam_id`.
  void addObservation(
      int cam_id, const aslam::cameras::GridCalibrationTargetObservation& obs);

  // Returns list of (cam_id, observation pointer) for a given timestamp.
  std::vector<
      std::pair<int, const aslam::cameras::GridCalibrationTargetObservation*>>
  getAllObsAtTimestamp(double timestamp) const;

  // Number of cameras we have observations for.
  std::size_t numCameras() const { return observations_.size(); }

  // All view timestamps (sorted)
  std::vector<double> getAllViewTimestamps() const;

  // Camera ids that observe at given timestamp
  std::vector<int> getCamIdsAtTimestamp(double timestamp) const;

  // Get observation for given cam_id at timestamp
  const aslam::cameras::GridCalibrationTargetObservation& getObservationAtTime(
      double timestamp, int cam_id) const;

  // Get obs index (into per-camera archive) for a camera at given time
  std::size_t getObsIdForCamAtTime(double timestamp, int cam_id) const;

  // Get observed corner ids for camera at given time
  const std::set<unsigned int>& getCornerIdsAtTime(double timestamp,
                                                   int cam_id) const;

  // Return list of tuples for all observations of a camera pair. nullptr
  // indicates missing observation for that camera at that timestamp.
  std::vector<
      std::pair<const aslam::cameras::GridCalibrationTargetObservation*,
                const aslam::cameras::GridCalibrationTargetObservation*>>
  getAllObsTwoCams(int camA, int camB) const;

  // Return list of observations for one camera across all timestamps.
  std::vector<const aslam::cameras::GridCalibrationTargetObservation*>
  getAllObsCam(int cam_id) const;

  // Print a simple table to the given ostream
  void printTable(std::ostream& os = std::cout) const;

 private:
  double max_delta_approxsync_ = 0.0;

  // Per-camera archive of observations (append-only)
  std::unordered_map<
      int, std::vector<aslam::cameras::GridCalibrationTargetObservation>>
      observations_;

  struct TargetViewEntry {
    std::size_t obs_id;
    std::set<unsigned int> observed_corners;
  };

  // Map: timestamp -> (cam_id -> TargetViewEntry). Using std::map to keep
  // timestamps ordered (similar to Python OrderedDict keyed by timestamp).
  std::map<double, std::unordered_map<int, TargetViewEntry>> targetViews_;
};

}  // namespace kalibr

#endif  // KALIBR_CAMERA_CALIBRATION_OBSDB_HPP