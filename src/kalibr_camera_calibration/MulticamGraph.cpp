#include "kalibr_camera_calibration/MulticamGraph.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <print>
#include <queue>
#include <sm/logging.hpp>

#include "kalibr_camera_calibration/CameraInitializers.hpp"

namespace kalibr {

MulticamGraph::MulticamGraph(const ObservationDatabase& obsDb)
    : obsDb_(obsDb), numCams_(static_cast<int>(obsDb.numCameras())) {
  initializeGraphFromObsDb();
}

void MulticamGraph::initializeGraphFromObsDb() {
  auto t0 = std::chrono::high_resolution_clock::now();

  // Go through all timestamps
  for (double timestamp : obsDb_.getAllViewTimestamps()) {
    // Cameras that have a target view at this timestamp
    std::vector<int> camIdsAtTimestamp = obsDb_.getCamIdsAtTimestamp(timestamp);

    // Generate all pairs of cameras at this timestamp
    for (size_t i = 0; i < camIdsAtTimestamp.size(); ++i) {
      for (size_t j = i + 1; j < camIdsAtTimestamp.size(); ++j) {
        int camIdA = camIdsAtTimestamp[i];
        int camIdB = camIdsAtTimestamp[j];

        // Ensure camIdA < camIdB (lower ID first)
        if (camIdA > camIdB) {
          std::swap(camIdA, camIdB);
        }

        // Get observed corners for both cameras
        const std::set<unsigned int>& cornersA =
            obsDb_.getCornerIdsAtTime(timestamp, camIdA);
        const std::set<unsigned int>& cornersB =
            obsDb_.getCornerIdsAtTime(timestamp, camIdB);

        // Find common corners
        std::set<unsigned int> commonCorners;
        std::set_intersection(
            cornersA.begin(), cornersA.end(), cornersB.begin(), cornersB.end(),
            std::inserter(commonCorners, commonCorners.begin()));

        // Add graph edge if we found common corners
        if (!commonCorners.empty()) {
          std::size_t obsIdA = obsDb_.getObsIdForCamAtTime(timestamp, camIdA);
          std::size_t obsIdB = obsDb_.getObsIdForCamAtTime(timestamp, camIdB);

          // Find or create edge
          int edgeIdx = findEdgeIndex(camIdA, camIdB);
          if (edgeIdx < 0) {
            // Create new edge
            edgeIdx = static_cast<int>(edges_.size());
            edges_.push_back(GraphEdge{.camA = camIdA,
                                       .camB = camIdB,
                                       .weight = 0,
                                       .obs_ids = {},
                                       .baseline_HL = std::nullopt});
            adjacency_[camIdA].push_back(static_cast<size_t>(edgeIdx));
            adjacency_[camIdB].push_back(static_cast<size_t>(edgeIdx));
          }

          // Update edge
          edges_[edgeIdx].weight += static_cast<int>(commonCorners.size());
          edges_[edgeIdx].obs_ids.emplace_back(obsIdA, obsIdB);
        }
      }
    }
  }

  auto t1 = std::chrono::high_resolution_clock::now();
  double total = std::chrono::duration<double>(t1 - t0).count();
  SM_DEBUG_STREAM("It took " << total << "s to build the graph.");
}

int MulticamGraph::findEdgeIndex(int camA, int camB) const {
  // Ensure camA < camB
  if (camA > camB) {
    std::swap(camA, camB);
  }

  auto it = adjacency_.find(camA);
  if (it == adjacency_.end()) {
    return -1;
  }

  for (size_t edgeIdx : it->second) {
    const GraphEdge& edge = edges_[edgeIdx];
    if (edge.camA == camA && edge.camB == camB) {
      return static_cast<int>(edgeIdx);
    }
  }
  return -1;
}

const GraphEdge* MulticamGraph::getEdge(int camA, int camB) const {
  int idx = findEdgeIndex(camA, camB);
  return idx >= 0 ? &edges_[idx] : nullptr;
}

bool MulticamGraph::isGraphConnected() const {
  if (numCams_ <= 1) {
    // Single camera or no cameras is trivially connected
    return true;
  }

  // BFS to check connectivity
  std::unordered_set<int> visited;
  std::queue<int> queue;
  queue.push(0);  // Start from camera 0
  visited.insert(0);

  while (!queue.empty()) {
    int current = queue.front();
    queue.pop();

    auto it = adjacency_.find(current);
    if (it != adjacency_.end()) {
      for (size_t edgeIdx : it->second) {
        const GraphEdge& edge = edges_[edgeIdx];
        int neighbor = (edge.camA == current) ? edge.camB : edge.camA;
        if (visited.find(neighbor) == visited.end()) {
          visited.insert(neighbor);
          queue.push(neighbor);
        }
      }
    }
  }

  return static_cast<int>(visited.size()) == numCams_;
}

std::vector<int> MulticamGraph::getCamOverlaps(int camId) const {
  std::vector<int> overlaps;
  auto it = adjacency_.find(camId);
  if (it != adjacency_.end()) {
    for (size_t edgeIdx : it->second) {
      const GraphEdge& edge = edges_[edgeIdx];
      int neighbor = (edge.camA == camId) ? edge.camB : edge.camA;
      overlaps.push_back(neighbor);
    }
  }
  return overlaps;
}

std::unordered_map<int, int> MulticamGraph::dijkstraShortestPaths(
    int sourceVertex, const std::vector<double>& weights) const {
  // Distance map
  std::unordered_map<int, double> dist;
  std::unordered_map<int, int> predecessor;

  for (int i = 0; i < numCams_; ++i) {
    dist[i] = std::numeric_limits<double>::infinity();
    predecessor[i] = -1;
  }
  dist[sourceVertex] = 0.0;

  // Priority queue: (distance, vertex)
  using PQElement = std::pair<double, int>;
  std::priority_queue<PQElement, std::vector<PQElement>, std::greater<>> pq;
  pq.emplace(0.0, sourceVertex);

  while (!pq.empty()) {
    auto [d, u] = pq.top();
    pq.pop();

    if (d > dist[u]) {
      continue;  // Skip stale entry
    }

    auto it = adjacency_.find(u);
    if (it == adjacency_.end()) {
      continue;
    }

    for (size_t edgeIdx : it->second) {
      const GraphEdge& edge = edges_[edgeIdx];
      int v = (edge.camA == u) ? edge.camB : edge.camA;
      double w = weights[edgeIdx];
      double newDist = dist[u] + w;

      if (newDist < dist[v]) {
        dist[v] = newDist;
        predecessor[v] = u;
        pq.emplace(newDist, v);
      }
    }
  }

  return predecessor;
}

std::set<size_t> MulticamGraph::getEdgesOnShortestPaths(
    int sourceVertex, const std::vector<double>& weights) const {
  auto predecessor = dijkstraShortestPaths(sourceVertex, weights);

  std::set<size_t> edgesOnPath;
  for (int v = 0; v < numCams_; ++v) {
    if (v == sourceVertex) continue;

    // Trace back path from v to source
    int current = v;
    while (predecessor[current] >= 0) {
      int prev = predecessor[current];
      int edgeIdx = findEdgeIndex(prev, current);
      if (edgeIdx >= 0) {
        edgesOnPath.insert(static_cast<size_t>(edgeIdx));
      }
      current = prev;
    }
  }

  return edgesOnPath;
}

std::vector<int> MulticamGraph::getShortestPath(int from, int to) const {
  if (from == to) {
    return {from};
  }

  // Build weights for BFS (unweighted shortest path)
  std::unordered_map<int, int> predecessor;
  std::unordered_map<int, bool> visited;
  std::queue<int> queue;

  queue.push(from);
  visited[from] = true;
  predecessor[from] = -1;

  while (!queue.empty()) {
    int current = queue.front();
    queue.pop();

    if (current == to) {
      break;
    }

    auto it = adjacency_.find(current);
    if (it != adjacency_.end()) {
      for (size_t edgeIdx : it->second) {
        const GraphEdge& edge = edges_[edgeIdx];
        int neighbor = (edge.camA == current) ? edge.camB : edge.camA;
        if (!visited[neighbor]) {
          visited[neighbor] = true;
          predecessor[neighbor] = current;
          queue.push(neighbor);
        }
      }
    }
  }

  // Reconstruct path
  std::vector<int> path;
  int current = to;
  while (current >= 0) {
    path.push_back(current);
    current = predecessor[current];
  }
  std::reverse(path.begin(), path.end());

  return path;
}

std::vector<sm::kinematics::Transformation> MulticamGraph::getInitialGuesses(
    std::vector<CameraGeometry*>& cameras) {
  // =========================================================================
  // STEP 0: Check if all cameras are connected
  // =========================================================================
  if (!isGraphConnected()) {
    SM_ERROR_STREAM(
        "The cameras are not connected through mutual target observations! "
        "Please provide another dataset...");
    plotGraph();
    std::exit(0);
  }

  // =========================================================================
  // STEP 1: Find optimal camera pairs using weighted shortest paths
  // =========================================================================

  // Build inverse weights (more common corners = lower weight = better path)
  std::vector<double> weights;
  weights.reserve(edges_.size());
  for (const auto& edge : edges_) {
    weights.push_back(1.0 / static_cast<double>(edge.weight));
  }

  // Choose the camera with the least edges as base camera
  std::vector<int> outdegrees(numCams_, 0);
  for (int camId = 0; camId < numCams_; ++camId) {
    auto it = adjacency_.find(camId);
    if (it != adjacency_.end()) {
      outdegrees[camId] = static_cast<int>(it->second.size());
    }
  }
  int baseCamId =
      static_cast<int>(std::min_element(outdegrees.begin(), outdegrees.end()) -
                       outdegrees.begin());

  // Get edges on shortest paths from base camera to all others
  optimalBaselineEdges_ = getEdgesOnShortestPaths(baseCamId, weights);

  // =========================================================================
  // STEP 2: Solve stereo calibration for optimal baseline pairs
  // =========================================================================

  for (size_t edgeIdx : optimalBaselineEdges_) {
    GraphEdge& edge = edges_[edgeIdx];
    int camL = edge.camA;
    int camH = edge.camB;

    std::println("\t initializing camera pair ({},{})...", camL, camH);

    // Get observation pairs
    auto obsPtrs = getAllMutualObsBetweenTwoCams(camL, camH);
    std::vector<std::pair<
        std::optional<aslam::cameras::GridCalibrationTargetObservation>,
        std::optional<aslam::cameras::GridCalibrationTargetObservation>>>
        obsList;

    for (size_t i = 0; i < obsPtrs.first.size(); ++i) {
      std::optional<aslam::cameras::GridCalibrationTargetObservation> obsL;
      std::optional<aslam::cameras::GridCalibrationTargetObservation> obsH;
      if (obsPtrs.first[i]) {
        obsL = *obsPtrs.first[i];
      }
      if (obsPtrs.second[i]) {
        obsH = *obsPtrs.second[i];
      }
      obsList.emplace_back(std::move(obsL), std::move(obsH));
    }

    // Run stereo calibration
    auto result = stereoCalibrate(*cameras[camL], *cameras[camH], obsList,
                                  /*distortionActive=*/false);

    if (result.success) {
      SM_DEBUG_STREAM("baseline_" << camL << "_" << camH << "="
                                  << result.baseline_HL.T());
    } else {
      SM_ERROR_STREAM("initialization of camera pair (" << camL << "," << camH
                                                        << ") failed");
      SM_ERROR_STREAM("estimated baseline_" << camL << "_" << camH << "="
                                            << result.baseline_HL.T());
    }

    // Store baseline in graph edge
    edge.baseline_HL = result.baseline_HL;
  }

  // =========================================================================
  // STEP 3: Transform from optimal baseline chain to camera chain ordering
  // =========================================================================

  std::vector<sm::kinematics::Transformation> baselines;
  baselines.reserve(numCams_ - 1);

  for (int baselineId = 0; baselineId < numCams_ - 1; ++baselineId) {
    // Find shortest path from camera baselineId to baselineId+1
    std::vector<int> path = getShortestPath(baselineId, baselineId + 1);

    // Chain transformations along the path
    sm::kinematics::Transformation baseline_HL;
    for (size_t pathIdx = 0; pathIdx + 1 < path.size(); ++pathIdx) {
      int sourceVert = path[pathIdx];
      int targetVert = path[pathIdx + 1];

      int edgeIdx = findEdgeIndex(sourceVert, targetVert);
      if (edgeIdx < 0 || !edges_[edgeIdx].baseline_HL) {
        SM_ERROR_STREAM("Missing baseline between cameras "
                        << sourceVert << " and " << targetVert);
        continue;
      }

      sm::kinematics::Transformation T_edge = *edges_[edgeIdx].baseline_HL;

      // Correct direction (baselines are always stored from low to high cam ID)
      if (sourceVert > targetVert) {
        T_edge = T_edge.inverse();
      }

      // Chain up
      baseline_HL = T_edge * baseline_HL;
    }

    baselines.push_back(baseline_HL);
  }

  // =========================================================================
  // STEP 4: Refine guess in full batch
  // =========================================================================

  auto result = solveFullBatch(cameras, baselines, *this);

  if (!result.success) {
    SM_WARN_STREAM("Full batch refinement failed!");
  } else {
    baselines = result.baselines;
  }

  return baselines;
}

sm::kinematics::Transformation MulticamGraph::getTargetPoseGuess(
    double timestamp, const std::vector<CameraGeometry*>& cameras,
    const std::vector<sm::kinematics::Transformation>& baselines_HL) const {
  // Find camera with most observed corners at this timestamp
  std::vector<int> camIds = obsDb_.getCamIdsAtTimestamp(timestamp);
  if (camIds.empty()) {
    SM_WARN_STREAM("No cameras observing target at timestamp " << timestamp);
    return sm::kinematics::Transformation();
  }

  int maxCorners = 0;
  int camIdMax = camIds[0];
  for (int camId : camIds) {
    int numCorners =
        static_cast<int>(obsDb_.getCornerIdsAtTime(timestamp, camId).size());
    if (numCorners > maxCorners) {
      maxCorners = numCorners;
      camIdMax = camId;
    }
  }

  // Solve PnP problem
  auto geometry = cameras[camIdMax]->getGeometry();
  const auto& obs = obsDb_.getObservationAtTime(timestamp, camIdMax);

  sm::kinematics::Transformation T_t_cN;
  bool success = geometry->estimateTransformation(obs, T_t_cN);

  if (!success) {
    SM_WARN_STREAM(
        "getTargetPoseGuess: solvePnP failed with solution: " << T_t_cN.T());
  }

  // Transform back to cam0: T_t_cN --> T_t_c0
  sm::kinematics::Transformation T_cN_c0;
  for (int i = 0; i < camIdMax && i < static_cast<int>(baselines_HL.size());
       ++i) {
    T_cN_c0 = baselines_HL[i] * T_cN_c0;
  }

  sm::kinematics::Transformation T_t_c0 = T_t_cN * T_cN_c0;
  return T_t_c0;
}

std::pair<std::vector<const aslam::cameras::GridCalibrationTargetObservation*>,
          std::vector<const aslam::cameras::GridCalibrationTargetObservation*>>
MulticamGraph::getAllMutualObsBetweenTwoCams(int camA, int camB) const {
  std::vector<const aslam::cameras::GridCalibrationTargetObservation*> obsA;
  std::vector<const aslam::cameras::GridCalibrationTargetObservation*> obsB;

  int edgeIdx = findEdgeIndex(camA, camB);
  if (edgeIdx < 0) {
    SM_ERROR_STREAM(
        "getAllMutualObsBetweenTwoCams: no mutual observations between cams "
        << camA << " and " << camB);
    return {obsA, obsB};
  }

  const GraphEdge& edge = edges_[edgeIdx];

  // Determine which camera is lower/higher ID
  bool camAIsLower = (camA < camB);

  for (const auto& [obsIdL, obsIdH] : edge.obs_ids) {
    // obs_ids stores (lower_cam_obs_id, higher_cam_obs_id)
    std::size_t obsIdA = camAIsLower ? obsIdL : obsIdH;
    std::size_t obsIdB = camAIsLower ? obsIdH : obsIdL;

    // Get observation pointers from database
    // We need to access the raw observations from obsDb_
    // Using getAllObsCam and indexing
    auto allObsA = obsDb_.getAllObsCam(camA);
    auto allObsB = obsDb_.getAllObsCam(camB);

    if (obsIdA < allObsA.size()) {
      obsA.push_back(allObsA[obsIdA]);
    } else {
      obsA.push_back(nullptr);
    }
    if (obsIdB < allObsB.size()) {
      obsB.push_back(allObsB[obsIdB]);
    } else {
      obsB.push_back(nullptr);
    }
  }

  return {obsA, obsB};
}

std::string MulticamGraph::plotGraph(const std::string& filepath) const {
  // Simple text-based graph representation
  // For full visualization, would need graphviz or matplot++ integration
  std::ofstream file(filepath + ".txt");
  if (file.is_open()) {
    printGraph(file);
    file.close();
  }

  SM_INFO_STREAM("Graph saved to " << filepath << ".txt");
  return filepath + ".txt";
}

void MulticamGraph::printGraph(std::ostream& os) const {
  os << "MulticamGraph:\n";
  os << "  Cameras: " << numCams_ << "\n";
  os << "  Edges: " << edges_.size() << "\n";
  os << "  Connected: " << (isGraphConnected() ? "yes" : "no") << "\n";
  os << "\n";

  os << "  Adjacency:\n";
  for (int camId = 0; camId < numCams_; ++camId) {
    os << "    cam" << camId << ": ";
    auto overlaps = getCamOverlaps(camId);
    for (size_t i = 0; i < overlaps.size(); ++i) {
      if (i > 0) os << ", ";
      os << "cam" << overlaps[i];
    }
    os << "\n";
  }

  os << "\n  Edges:\n";
  for (size_t i = 0; i < edges_.size(); ++i) {
    const auto& edge = edges_[i];
    os << "    [" << i << "] cam" << edge.camA << " <-> cam" << edge.camB
       << " (weight=" << edge.weight << ", obs_pairs=" << edge.obs_ids.size()
       << ")";
    if (optimalBaselineEdges_.count(i) > 0) {
      os << " [OPTIMAL]";
    }
    os << "\n";
  }
}

}  // namespace kalibr
