#include "kalibr_camera_calibration/ObsDb.hpp"

#include <cmath>
#include <print>

namespace kalibr {

ObservationDatabase::ObservationDatabase(double max_delta_approxsync)
    : max_delta_approxsync_(max_delta_approxsync) {}

void ObservationDatabase::addObservation(
    int cam_id, const aslam::cameras::GridCalibrationTargetObservation& obs) {
  // create camera list (initialization)
  if (this->observations_.find(cam_id) == observations_.end()) {
    this->observations_[cam_id] =
        std::vector<aslam::cameras::GridCalibrationTargetObservation>();
  }

  // add to archive (make a copy)
  this->observations_[cam_id].push_back(obs);
  std::size_t obs_idx = this->observations_[cam_id].size() - 1;

  // find nearest timestamp in the table

  double timestamp_obs = obs.time().toSec();

  double nearest_timestamp = 0.0;
  if (targetViews_.empty()) {
    nearest_timestamp = timestamp_obs + 5.0 * (max_delta_approxsync_ + 1.0);
  } else {
    // find timestamp key with minimal absolute difference
    nearest_timestamp =
        std::min_element(targetViews_.begin(), targetViews_.end(),
                         [timestamp_obs](const auto& a, const auto& b) {
                           return std::fabs(a.first - timestamp_obs) <
                                  std::fabs(b.first - timestamp_obs);
                         })
            ->first;
  }

  double timestamp = 0.0;
  if (std::fabs(nearest_timestamp - timestamp_obs) <= max_delta_approxsync_) {
    timestamp = nearest_timestamp;
  } else {
    timestamp = timestamp_obs;
    targetViews_[timestamp] = std::unordered_map<int, TargetViewEntry>();
  }

  // fill in observation data
  auto& map_for_time = targetViews_[timestamp];
  if (map_for_time.find(cam_id) == map_for_time.end()) {
    this->targetViews_[timestamp][cam_id] = TargetViewEntry();
    this->targetViews_[timestamp][cam_id].obs_id = obs_idx;
    std::vector<unsigned int> dummy_corners;
    this->targetViews_[timestamp][cam_id].observed_corners =
        std::set<unsigned int>{obs.getCornersIdx(dummy_corners)};
  } else {
    std::println(std::cerr,
                 "[TargetViewTable]: Tried to add second view to a given "
                 "cameraId & timestamp."
                 " Maybe try to reduce the approximate syncing tolerance..");
  }
}

std::vector<
    std::pair<int, const aslam::cameras::GridCalibrationTargetObservation*>>
ObservationDatabase::getAllObsAtTimestamp(double timestamp) const {
  std::vector<
      std::pair<int, const aslam::cameras::GridCalibrationTargetObservation*>>
      observations_at_time;
  for (auto cam_id : this->getCamIdsAtTimestamp(timestamp)) {
    auto it = targetViews_.find(timestamp);
    if (it == targetViews_.end()) continue;
    auto it2 = it->second.find(cam_id);
    if (it2 == it->second.end()) continue;
    auto obs_id = it2->second.obs_id;
    const auto& obs = observations_.at(cam_id).at(obs_id);
    observations_at_time.emplace_back(cam_id, &obs);
  }
  return observations_at_time;
}

std::vector<double> ObservationDatabase::getAllViewTimestamps() const {
  std::vector<double> times;
  times.reserve(targetViews_.size());
  for (const auto& kv : targetViews_) times.push_back(kv.first);
  return times;
}

std::vector<int> ObservationDatabase::getCamIdsAtTimestamp(
    double timestamp) const {
  std::vector<int> cams;
  auto it = targetViews_.find(timestamp);
  if (it == targetViews_.end()) return cams;
  cams.reserve(it->second.size());
  for (const auto& kv : it->second) cams.push_back(kv.first);
  return cams;
}

const aslam::cameras::GridCalibrationTargetObservation&
ObservationDatabase::getObservationAtTime(double timestamp, int cam_id) const {
  std::size_t obs_id = getObsIdForCamAtTime(timestamp, cam_id);
  return observations_.at(cam_id).at(obs_id);
}

std::size_t ObservationDatabase::getObsIdForCamAtTime(double timestamp,
                                                      int cam_id) const {
  auto it = targetViews_.find(timestamp);
  if (it == targetViews_.end()) throw std::out_of_range("timestamp not found");
  auto it2 = it->second.find(cam_id);
  if (it2 == it->second.end())
    throw std::out_of_range("cam_id not found at timestamp");
  return it2->second.obs_id;
}

const std::set<unsigned int>& ObservationDatabase::getCornerIdsAtTime(
    double timestamp, int cam_id) const {
  auto it = targetViews_.find(timestamp);
  if (it == targetViews_.end()) throw std::out_of_range("timestamp not found");
  auto it2 = it->second.find(cam_id);
  if (it2 == it->second.end())
    throw std::out_of_range("cam_id not found at timestamp");
  return it2->second.observed_corners;
}

std::vector<std::pair<const aslam::cameras::GridCalibrationTargetObservation*,
                      const aslam::cameras::GridCalibrationTargetObservation*>>
ObservationDatabase::getAllObsTwoCams(int cam_id_A, int cam_id_B) const {
  std::vector<
      std::pair<const aslam::cameras::GridCalibrationTargetObservation*,
                const aslam::cameras::GridCalibrationTargetObservation*>>
      tuples;
  for (auto timestamp : this->getAllViewTimestamps()) {
    try {
      auto obsA = this->getObservationAtTime(timestamp, cam_id_A);
      auto obsB = this->getObservationAtTime(timestamp, cam_id_B);
      tuples.emplace_back(&obsA, &obsB);
    } catch (...) {
    }
  }
  return tuples;
}

std::vector<const aslam::cameras::GridCalibrationTargetObservation*>
ObservationDatabase::getAllObsCam(int cam_id) const {
  std::vector<const aslam::cameras::GridCalibrationTargetObservation*> out;
  for (const auto& kv : targetViews_) {
    const auto& cammap = kv.second;
    auto it = cammap.find(cam_id);
    if (it != cammap.end()) {
      out.push_back(&observations_.at(cam_id).at(it->second.obs_id));
    }
  }
  return out;
}

void ObservationDatabase::printTable(std::ostream& os) const {
  std::print(os, "timestamp \t ");
  for (std::size_t cam_id = 0; cam_id < numCameras(); ++cam_id) {
    std::print(os, "cam{} \t ", cam_id);
  }
  std::println(os, "");

  // gather sorted times
  std::vector<double> times = getAllViewTimestamps();

  for (double t : times) {
    std::print(os, "{} ", t);
    for (std::size_t cam_id = 0; cam_id < numCameras(); ++cam_id) {
      try {
        auto it = targetViews_.find(t);
        if (it == targetViews_.end()) {
          std::print(os, "\t - ");
          continue;
        }
        auto it2 = it->second.find(static_cast<int>(cam_id));
        if (it2 == it->second.end()) {
          std::print(os, "\t - ");
        } else {
          std::print(os, "\t {}", it2->second.observed_corners.size());
        }
      } catch (...) {
        std::print(os, "\t - ");
      }
    }
    std::println(os, "");
  }
}

}  // namespace kalibr
