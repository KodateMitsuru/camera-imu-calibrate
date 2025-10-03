#ifndef ICC_PLOTS_HPP
#define ICC_PLOTS_HPP

#include <matplot/matplot.h>

#include <Eigen/Core>
#include <string>
#include <vector>

namespace kalibr {

class IccCalibrator;

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
void plotVectorOverTime(const std::vector<double>& times,
                        const std::vector<Eigen::Vector3d>& values,
                        const std::string& title = "",
                        const std::string& ylabel = "",
                        const std::string& label = "", int figureNumber = 1,
                        bool clearFigure = true, bool noShow = false,
                        double lineWidth = 3.0);

/**
 * @brief Plot reprojection error scatter
 */
void plotReprojectionScatter(const IccCalibrator& calibrator, int camId,
                             int figureNumber = 1, bool clearFigure = true,
                             bool noShow = false,
                             const std::string& title = "");

/**
 * @brief Camera plot helper class
 */
class CameraPlot {
 public:
  CameraPlot(const IccCalibrator& calibrator, int camId);

  void plotReprojectionErrors(int figureNumber = 1, bool clearFigure = true,
                              bool noShow = false);

  void plotReprojectionScatter(int figureNumber = 1, bool clearFigure = true,
                               bool noShow = false);

 private:
  const IccCalibrator& calibrator_;
  int camId_;
};

}  // namespace kalibr

#endif  // ICC_PLOTS_HPP
