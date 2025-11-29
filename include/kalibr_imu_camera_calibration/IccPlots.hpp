#ifndef ICC_PLOTS_HPP
#define ICC_PLOTS_HPP

#include <matplot/matplot.h>

#include <Eigen/Core>
#include <memory>
#include <sm/plot/PlotCollection.hpp>
#include <string>
#include <vector>

#include "IccCalibrator.hpp"

namespace kalibr {

// ============================================================================
// Global Figure Registry (now using sm::plot::PlotCollection)
// ============================================================================

/**
 * @brief Get the global figure registry
 * @return Reference to the global PlotCollection instance
 */
inline sm::plot::PlotCollection& getFigureRegistry() {
  return sm::plot::PlotCollection::global();
}

/**
 * @brief Get or create a figure by number
 * @param figNum Figure number
 * @return Shared pointer to the figure
 */
inline std::shared_ptr<matplot::figure_type> getOrCreateFigure(int figNum) {
  return getFigureRegistry().get_or_create_figure(figNum);
}

// ============================================================================
// Plotting Functions
// ============================================================================

/**
 * @brief Plot IMU sample rates
 */
void plotIMURates(const IccCalibrator& calibrator, int imuIdx,
                  int figureNumber = 1, bool clearFigure = true,
                  bool noShow = false);

/**
 * @brief Plot gyroscope errors
 */
void plotGyroError(const IccCalibrator& calibrator, int imuIdx,
                   int figureNumber = 1, bool clearFigure = true,
                   bool noShow = false);

/**
 * @brief Plot gyroscope errors per axis
 */
void plotGyroErrorPerAxis(const IccCalibrator& calibrator, int imuIdx,
                          int figureNumber = 1, bool clearFigure = true,
                          bool noShow = false);

/**
 * @brief Plot accelerometer errors
 */
void plotAccelError(const IccCalibrator& calibrator, int imuIdx,
                    int figureNumber = 1, bool clearFigure = true,
                    bool noShow = false);

/**
 * @brief Plot accelerometer errors per axis
 */
void plotAccelErrorPerAxis(const IccCalibrator& calibrator, int imuIdx,
                           int figureNumber = 1, bool clearFigure = true,
                           bool noShow = false);

/**
 * @brief Plot accelerometer bias over time
 */
void plotAccelBias(const IccCalibrator& calibrator, int imuIdx,
                   int figureNumber = 1, bool clearFigure = true,
                   bool noShow = false);

/**
 * @brief Plot angular velocity bias over time
 */
void plotAngularVelocityBias(const IccCalibrator& calibrator, int imuIdx,
                             int figureNumber = 1, bool clearFigure = true,
                             bool noShow = false);

/**
 * @brief Plot angular velocities (spline vs measurements)
 */
void plotAngularVelocities(const IccCalibrator& calibrator, int imuIdx,
                           int figureNumber = 1, bool clearFigure = true,
                           bool noShow = false);

/**
 * @brief Plot accelerations (spline vs measurements)
 */
void plotAccelerations(const IccCalibrator& calibrator, int imuIdx,
                       int figureNumber = 1, bool clearFigure = true,
                       bool noShow = false);

/**
 * @brief Plot vector over time (generic)
 */
std::vector<matplot::axes_handle> plotVectorOverTime(
    const std::vector<double>& times,
    const std::vector<Eigen::Vector3d>& values, const std::string& title = "",
    const std::string& ylabel = "", const std::string& label = "",
    int figureNumber = 1, bool clearFigure = true, bool noShow = false,
    double lineWidth = 3.0);

/**
 * @brief Plot reprojection error scatter
 */
void plotReprojectionScatter(const IccCalibrator& calibrator, int camId,
                             int figureNumber = 1, bool clearFigure = true,
                             bool noShow = false,
                             const std::string& title = "");

/**
 * @brief 3D visualization class for camera pose and calibration target
 */
class CameraPlot {
 public:
  /**
   * @brief Constructor
   * @param figureNumber Figure number to use
   * @param targetPoints Nx3 matrix of target corner points in 3D
   * @param camSize Size of camera coordinate axes for visualization
   */
  CameraPlot(int figureNumber, const Eigen::MatrixXd& targetPoints,
             double camSize);

  /**
   * @brief Update camera pose visualization
   * @param T 4x4 transformation matrix (homogeneous)
   */
  void plot3DCamera(const Eigen::Matrix4d& T);

 private:
  void setupFigure();
  void plot3Dgrid();

  int figureNumber_;
  Eigen::MatrixXd targetPoints_;
  double camSize_;
  bool initialized_;
};

}  // namespace kalibr

#endif  // ICC_PLOTS_HPP
