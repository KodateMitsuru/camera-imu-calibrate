#ifndef KALIBR_CAMERA_CALIBRATION_MULTICAM_GRAPH_HPP
#define KALIBR_CAMERA_CALIBRATION_MULTICAM_GRAPH_HPP

#include <Eigen/Core>
#include <format_utils.hpp>
#include <optional>
#include <set>
#include <sm/kinematics/Transformation.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "CameraCalibrator.hpp"
#include "ObsDb.hpp"

namespace kalibr {

/**
 * @brief Graph edge representing a camera pair relationship
 */
struct GraphEdge {
  int camA;        // Always the lower camera ID
  int camB;        // Always the higher camera ID
  int weight = 0;  // Number of common corners observed

  // Observation ID pairs: (obs_id_lower_cam, obs_id_higher_cam)
  std::vector<std::pair<std::size_t, std::size_t>> obs_ids;

  // Baseline transformation from lower to higher camera ID
  std::optional<sm::kinematics::Transformation> baseline_HL;
};

/**
 * @brief Multi-camera calibration graph
 *
 * Builds and manages a graph structure representing camera overlap
 * relationships based on common target point observations. Used for:
 * - Finding optimal camera pairs for initial baseline estimation
 * - Computing initial transformation guesses via stereo calibration
 * - Retrieving mutual observations between camera pairs
 *
 * Graph structure:
 * - Vertices: Cameras (cam_id)
 * - Edges: Camera pairs that share common target corner observations
 * - Edge weights: Number of common corners observed
 */
class MulticamGraph {
 public:
  /**
   * @brief Construct graph from observation database
   * @param obsDb Reference to observation database
   */
  explicit MulticamGraph(const ObservationDatabase& obsDb);

  // ===========================================================================
  // System Properties
  // ===========================================================================

  /**
   * @brief Check if all cameras are connected through observations
   * @return true if graph is connected (all cameras reachable)
   */
  bool isGraphConnected() const;

  /**
   * @brief Get camera IDs that share observations with given camera
   * @param camId Camera ID to query
   * @return Vector of overlapping camera IDs
   */
  std::vector<int> getCamOverlaps(int camId) const;

  /**
   * @brief Get number of cameras in the graph
   */
  int numCameras() const { return numCams_; }

  // ===========================================================================
  // Initial Guess Computation
  // ===========================================================================

  /**
   * @brief Compute initial baseline guesses between cameras
   *
   * Algorithm:
   * 1. Check graph connectivity
   * 2. Find optimal camera pairs using Dijkstra's algorithm
   * 3. Run stereo calibration for each pair
   * 4. Transform baselines to camera chain ordering (T_c1_c0, T_c2_c1, ...)
   * 5. Refine in full batch optimization
   *
   * @param cameras Vector of camera geometries
   * @return Vector of baseline transformations (cam[i] to cam[i+1])
   */
  std::vector<sm::kinematics::Transformation> getInitialGuesses(
      std::vector<CameraGeometry*>& cameras);

  /**
   * @brief Get target pose guess at a timestamp
   *
   * Uses PnP solution from the camera with most observed corners.
   *
   * @param timestamp Observation timestamp
   * @param cameras Camera geometries
   * @param baselines_HL Baseline transformations (optional, for multi-cam)
   * @return Target-to-camera0 transformation
   */
  sm::kinematics::Transformation getTargetPoseGuess(
      double timestamp, const std::vector<CameraGeometry*>& cameras,
      const std::vector<sm::kinematics::Transformation>& baselines_HL = {})
      const;

  /**
   * @brief Get all mutual observations between two cameras
   * @param camA First camera ID
   * @param camB Second camera ID
   * @return Pair of observation vectors (obsA, obsB) with same indexing
   */
  std::pair<
      std::vector<const aslam::cameras::GridCalibrationTargetObservation*>,
      std::vector<const aslam::cameras::GridCalibrationTargetObservation*>>
  getAllMutualObsBetweenTwoCams(int camA, int camB) const;

  // ===========================================================================
  // Graph Access
  // ===========================================================================

  /**
   * @brief Get edge between two cameras
   * @param camA First camera ID
   * @param camB Second camera ID
   * @return Pointer to edge or nullptr if not found
   */
  const GraphEdge* getEdge(int camA, int camB) const;

  /**
   * @brief Get all edges in the graph
   */
  const std::vector<GraphEdge>& getEdges() const { return edges_; }

  /**
   * @brief Get the observation database
   */
  const ObservationDatabase& getObsDb() const { return obsDb_; }

  /**
   * @brief Get optimal baseline edge indices (selected for stereo calibration)
   */
  const std::set<size_t>& getOptimalBaselineEdges() const {
    return optimalBaselineEdges_;
  }

  // ===========================================================================
  // Plotting and Printing
  // ===========================================================================

  /**
   * @brief Save graph visualization to file
   * @param filepath Output file path (e.g., "/tmp/graph.png")
   * @return Path to saved file
   */
  std::string plotGraph(const std::string& filepath = "/tmp/graph.png") const;

  /**
   * @brief Print graph structure to output stream
   * @param os Output stream
   */
  void printGraph(std::ostream& os = std::cout) const;

 private:
  /**
   * @brief Initialize graph structure from observation database
   */
  void initializeGraphFromObsDb();

  /**
   * @brief Find edge index between two cameras
   * @param camA First camera ID
   * @param camB Second camera ID
   * @return Edge index or -1 if not found
   */
  int findEdgeIndex(int camA, int camB) const;

  /**
   * @brief Compute shortest paths using Dijkstra's algorithm
   * @param sourceVertex Starting vertex
   * @param weights Edge weights (inverse of common corners)
   * @return Map of vertex -> predecessor vertex in shortest path
   */
  std::unordered_map<int, int> dijkstraShortestPaths(
      int sourceVertex, const std::vector<double>& weights) const;

  /**
   * @brief Get edges on shortest path from source to all vertices
   * @param sourceVertex Starting vertex
   * @param weights Edge weights
   * @return Set of edge indices on shortest paths
   */
  std::set<size_t> getEdgesOnShortestPaths(
      int sourceVertex, const std::vector<double>& weights) const;

  /**
   * @brief Find shortest path between two vertices
   * @param from Source vertex
   * @param to Destination vertex
   * @return Ordered list of vertices on path
   */
  std::vector<int> getShortestPath(int from, int to) const;

  // Reference to observation database
  const ObservationDatabase& obsDb_;

  // Number of cameras
  int numCams_;

  // Graph edges (adjacency list style)
  std::vector<GraphEdge> edges_;

  // Adjacency list: camId -> list of edge indices
  std::unordered_map<int, std::vector<size_t>> adjacency_;

  // Optimal baseline edges selected for stereo calibration
  std::set<size_t> optimalBaselineEdges_;
};

}  // namespace kalibr

#endif  // KALIBR_CAMERA_CALIBRATION_MULTICAM_GRAPH_HPP
