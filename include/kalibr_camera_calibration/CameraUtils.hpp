#ifndef KALIBR_CAMERA_CALIBRATION_CAMERA_UTILS_HPP
#define KALIBR_CAMERA_CALIBRATION_CAMERA_UTILS_HPP

#include <matplot/matplot.h>

#include <Eigen/Core>
#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/cameras/CameraGeometryBase.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <sm/kinematics/Transformation.hpp>
#include <sm/plot/PlotCollection.hpp>
#include <string>
#include <vector>

namespace kalibr {

// Forward declarations from this module
class CameraCalibration;
class CameraGeometry;
class CalibrationTargetOptimizationProblem;
// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Normalize a vector
 */
inline Eigen::Vector3d normalize(const Eigen::Vector3d& v) {
  double n = v.norm();
  if (n < 1e-12) return v;
  return v / n;
}

/**
 * @brief Get the ray through the image center for a camera
 * @param geometry Camera geometry
 * @return Normalized ray direction in camera frame
 */
Eigen::Vector3d getImageCenterRay(
    const std::shared_ptr<aslam::cameras::CameraGeometryBase>& geometry);

// ============================================================================
// Point Statistics
// ============================================================================

/**
 * @brief Statistics for a single observed calibration point
 */
struct PointStatistics {
  bool valid = false;
  int view_id = -1;
  int cam_id = -1;
  int pidx = -1;                         // point index on target
  Eigen::Vector2d y;                     // observed image point
  std::optional<Eigen::Vector2d> yhat;   // predicted (reprojected) point
  std::optional<double> azimuthalAngle;  // atan2(v[1], v[0])
  std::optional<double> polarAngle;      // acos(v[2])
  std::optional<double> squaredError;    // raw squared reprojection error
  std::optional<Eigen::Vector2d> e;      // reprojection error vector
};

/**
 * @brief Get statistics for a single point in a view
 * @param geometry Camera geometry
 * @param obs Observation
 * @param pidx Point index on calibration target
 * @param rerr Reprojection error term (may be nullptr)
 * @return PointStatistics struct
 */
PointStatistics getPointStatistics(
    const std::shared_ptr<aslam::cameras::CameraGeometryBase>& geometry,
    const aslam::cameras::GridCalibrationTargetObservation& obs, int pidx,
    const std::shared_ptr<aslam::backend::ErrorTerm>& rerr = nullptr);

/**
 * @brief Get statistics for all observed points across all views for a camera
 * @param calibrator CameraCalibration object
 * @param camId Camera ID
 * @return Vector of PointStatistics for all valid observations
 */
std::vector<PointStatistics> getAllPointStatistics(
    const CameraCalibration& calibrator, size_t camId);

// ============================================================================
// Reprojection Error Statistics
// ============================================================================

/**
 * @brief Reprojection errors for one view: (corners, reprojections, errors)
 *        Each inner vector has one entry per corner; entries may be nullopt if
 * not observed.
 */
struct ViewReprojectionData {
  bool valid = false;  // false if camera doesn't see target in this view
  std::vector<std::optional<Eigen::Vector2d>> corners;
  std::vector<std::optional<Eigen::Vector2d>> reprojections;
  std::vector<std::optional<Eigen::Vector2d>> errors;
};

/**
 * @brief Get reprojection errors for a camera across all views
 * @param calibrator CameraCalibration object
 * @param camId Camera ID
 * @return Vector of ViewReprojectionData, one per view/observation
 */
std::vector<ViewReprojectionData> getReprojectionErrors(
    const CameraCalibration& calibrator, size_t camId);

/**
 * @brief Compute mean and std of reprojection error components (x, y)
 * @param allViewData Vector of ViewReprojectionData from getReprojectionErrors
 * @return pair of (mean, std) each as 2D vector
 */
std::pair<Eigen::Vector2d, Eigen::Vector2d> getReprojectionErrorStatistics(
    const std::vector<ViewReprojectionData>& allViewData);

/**
 * @brief Compute mean and std of reprojection error norm (pixel distance)
 * @param allViewData Vector of ViewReprojectionData from getReprojectionErrors
 * @return pair of (mean, std) as double
 */
std::pair<double, double> getReprojectionErrorNormStatistics(
    const std::vector<ViewReprojectionData>& allViewData);

// ============================================================================
// Covariance Recovery (Calibration Results)
// ============================================================================

/**
 * @brief Recovered standard deviations from calibration
 */
struct CovarianceResult {
  // For each baseline (cam i to cam i+1): 6 elements [dq(3), dt(3)]
  std::vector<std::vector<double>> std_baselines;
  // For each camera: [distortion params..., projection params..., shutter
  // params...]
  std::vector<std::vector<double>> std_cameras;
};

// ============================================================================
// Result Export / Printing
// ============================================================================

/**
 * @brief Export camera poses to a CSV file (ETH groundtruth format)
 * @param poses Vector of (timestamp, T_target_camera) pairs
 * @param filename Output filename
 */
void exportPoses(
    const std::vector<std::pair<double, sm::kinematics::Transformation>>& poses,
    const std::string& filename);

// ============================================================================
// Plotting Functions (using matplot++)
// ============================================================================

/**
 * @brief Get the global figure registry
 */
inline sm::plot::PlotCollection& getCameraUtilsFigureRegistry() {
  return sm::plot::PlotCollection::global();
}

/**
 * @brief Get or create a figure by number
 */
inline std::shared_ptr<matplot::figure_type> getOrCreateCameraUtilsFigure(
    int figNum) {
  return getCameraUtilsFigureRegistry().get_or_create_figure(figNum);
}

/**
 * @brief Plot polar angle vs reprojection error
 * @param calibrator CameraCalibration object
 * @param camId Camera ID
 * @param figureNumber Figure number
 * @param clearFigure Whether to clear the figure first
 * @param stats Optional pre-computed statistics
 * @param noShow If true, don't call show()
 * @param title Plot title
 */
void plotPolarError(const CameraCalibration& calibrator, size_t camId,
                    int figureNumber = 1, bool clearFigure = true,
                    const std::vector<PointStatistics>* stats = nullptr,
                    bool noShow = false, const std::string& title = "");

/**
 * @brief Plot azimuthal angle vs reprojection error
 */
void plotAzimuthalError(const CameraCalibration& calibrator, size_t camId,
                        int figureNumber = 1, bool clearFigure = true,
                        const std::vector<PointStatistics>* stats = nullptr,
                        bool noShow = false, const std::string& title = "");

/**
 * @brief Plot all reprojection errors (corners + scatter)
 * @param calibrator CameraCalibration object
 * @param camId Camera ID
 * @param figureNumber Figure number
 * @param noShow If true, don't call show()
 * @param clearFigure Whether to clear the figure first
 * @param title Plot title
 */
void plotAllReprojectionErrors(const CameraCalibration& calibrator,
                               size_t camId, int figureNumber = 1,
                               bool noShow = false, bool clearFigure = true,
                               const std::string& title = "");

/**
 * @brief Plot corners and their reprojections for a single observation
 * @param obs Grid observation
 * @param reprojections Reprojected points (may contain nullopt)
 * @param figureNumber Figure number
 * @param cornerList Optional list of corner indices to plot
 * @param clearFigure Whether to clear the figure
 * @param plotImage Whether to plot the image
 * @param color RGBA color for reprojection markers
 * @param title Plot title
 */
void plotCornersAndReprojection(
    const aslam::cameras::GridCalibrationTargetObservation& obs,
    const std::vector<std::optional<Eigen::Vector2d>>& reprojections,
    int figureNumber = 1, const std::vector<int>* cornerList = nullptr,
    bool clearFigure = true, bool plotImage = true,
    const std::array<float, 4>& color = {1.0f, 0.0f, 0.0f, 0.5f},
    const std::string& title = "");

/**
 * @brief Plot detected corners
 * @param obs Grid observation
 * @param figureNumber Figure number
 * @param cornerList Optional list of corner indices to plot
 * @param clearFigure Whether to clear the figure
 * @param plotImage Whether to plot the image
 * @param color RGBA color for corner markers
 * @param subplot Subplot index (0 = no subplot)
 */
void plotCorners(const aslam::cameras::GridCalibrationTargetObservation& obs,
                 int figureNumber = 1,
                 const std::vector<int>* cornerList = nullptr,
                 bool clearFigure = true, bool plotImage = true,
                 const std::array<float, 4>& color = {0.0f, 1.0f, 1.0f, 0.3f},
                 int subplot = 0);

/**
 * @brief Plot camera trajectory
 * @param poses Vector of (timestamp, transformation) pairs
 * @param figureNumber Figure number
 * @param clearFigure Whether to clear the figure
 * @param title Plot title
 */
void plotTrajectory(
    const std::vector<std::pair<double, sm::kinematics::Transformation>>& poses,
    int figureNumber = 1, bool clearFigure = true,
    const std::string& title = "");

/**
 * @brief Plot camera rig (multi-camera system geometry)
 * @param baselines Vector of baseline transformations (cam_i to cam_{i+1})
 * @param figureNumber Figure number
 * @param clearFigure Whether to clear the figure
 * @param title Plot title
 */
void plotCameraRig(const std::vector<sm::kinematics::Transformation>& baselines,
                   int figureNumber = 1, bool clearFigure = true,
                   const std::string& title = "");

/**
 * @brief Plot outlier corners that were removed
 * @param calibrator CameraCalibration object
 * @param removedOutlierCorners Vector of (cam_id, corner_point)
 * @param figureNumber Figure number
 * @param clearFigure Whether to clear the figure
 * @param title Plot title
 */
void plotOutlierCorners(
    const CameraCalibration& calibrator,
    const std::vector<std::pair<int, Eigen::Vector2d>>& removedOutlierCorners,
    int figureNumber = 1, bool clearFigure = true,
    const std::string& title = "");

/**
 * @brief Generate a PDF calibration report
 * @param calibrator CameraCalibration object
 * @param filename Output PDF filename
 * @param showOnScreen Whether to show plots on screen
 * @param removedOutlierCorners Optional vector of removed outlier corners
 */
void generateReport(const CameraCalibration& calibrator,
                    const std::string& filename = "report.pdf",
                    bool showOnScreen = true,
                    const std::vector<std::pair<int, Eigen::Vector2d>>*
                        removedOutlierCorners = nullptr);

/**
 * @brief Save calibration results to text file
 * @param calibrator CameraCalibration object
 * @param filename Output filename
 */
void saveResultTxt(
    const CameraCalibration& calibrator,
    const std::string& filename = "camera_calibration_result.txt");

/**
 * @brief Print calibration parameters to stream
 * @param calibrator CameraCalibration object
 * @param out Output stream
 */
void printParameters(const CameraCalibration& calibrator,
                     std::ostream& out = std::cout);

}  // namespace kalibr

#endif  // KALIBR_CAMERA_CALIBRATION_CAMERA_UTILS_HPP
