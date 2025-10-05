#include "matplot/freestanding/axes_functions.h"
#include <matplot/matplot.h>

#include <Eigen/Core>
#include <IccCalibrator.hpp>
#include <IccPlots.hpp>
#include <IccSensors.hpp>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

namespace kalibr {

void plotIMURates(const IccCalibrator& calibrator, int imuIdx, int figureNumber,
                  bool clearFigure, bool noShow) {
  auto& imuList = calibrator.getImuList();
  auto& imu = imuList[imuIdx];
  auto bodyspline = calibrator.getPoseDv()->spline();

  // Get timestamps
  std::vector<double> timestamps;
  for (const auto& im : imu->getImuData()) {
    double t = im.stamp.toSec() + imu->getTimeOffset();
    if (t > bodyspline.t_min() && t < bodyspline.t_max()) {
      timestamps.push_back(t);
    }
  }

  double scale = 1000.0;
  std::string unit = "ms";
  double z_thresh = 1.2;

  // Calculate relative rate between readings
  std::vector<double> times;
  std::vector<double> rates;

  for (size_t idx = 1; idx < timestamps.size(); ++idx) {
    times.push_back(timestamps[idx]);
    rates.push_back((timestamps[idx] - timestamps[idx - 1]) * scale);
  }

  if (rates.empty()) return;

  double rate_avg =
      std::accumulate(rates.begin(), rates.end(), 0.0) / rates.size();
  double rate_std = std::accumulate(rates.begin(), rates.end(), 0.0,
                                    [rate_avg](double acc, double r) {
                                      return acc + (r - rate_avg) * (r - rate_avg);
                                    });
  rate_std = std::sqrt(rate_std / rates.size());

  // Z-test to find outliers
  std::vector<int> sizes(rates.size(), 1);
  std::vector<char> colors(rates.size(), 'b');  // Blue for inliers

  // Plot
  auto fig = matplot::subplot(2, 1, 1);

  matplot::scatter(times, rates);
  matplot::title("imu" + std::to_string(imuIdx) + ": sample inertial rate");
  matplot::xlabel("time (s)");
  matplot::ylabel("sample rate (" + unit + ")");
  matplot::grid(matplot::on);

  if (!noShow) {
    matplot::show();
  }
}

// void plotGyroError(const IccCalibrator& calibrator, int imuIdx,
//                    int figureNumber, bool clearFigure, bool noShow) {
//   auto& imuList = calibrator.getImuList();
//   if (imuIdx >= static_cast<int>(imuList.size())) {
//     std::cerr << "IMU index out of range" << std::endl;
//     return;
//   }

//   // Collect gyro errors
//   std::vector<double> errors;
//   // This would access gyroErrors from the IMU
//   // for (const auto& re : imu->gyroErrors) {
//   //     Eigen::Vector3d err = re->error();
//   //     errors.push_back(err.dot(err));
//   // }

//   auto fig = matplot::figure(figureNumber);
//   if (clearFigure) {
//     fig->clear();
//   }

//   matplot::subplot(2, 1, 1);
//   matplot::plot(errors);
//   matplot::xlabel("error index");
//   matplot::ylabel("error (rad/sec) squared");
//   matplot::grid(matplot::on);
//   matplot::title("imu" + std::to_string(imuIdx) + ": angular velocities error");

//   // Filter errors (only plot till 5*sigma)
//   if (!errors.empty()) {
//     double mean =
//         std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
//     double sigma = 0.0;
//     for (double e : errors) {
//       sigma += (e - mean) * (e - mean);
//     }
//     sigma = std::sqrt(sigma / errors.size());

//     std::vector<double> filtered_errors;
//     for (double e : errors) {
//       if (e < 5 * sigma) {
//         filtered_errors.push_back(e);
//       }
//     }

//     matplot::subplot(2, 1, 2);
//     matplot::hist(filtered_errors);
//     matplot::xlabel("error (rad/s) squared");
//     matplot::ylabel("count");
//     matplot::grid(matplot::on);
//   }

//   if (!noShow) {
//     matplot::show();
//   }
// }

// void plotGyroErrorPerAxis(const IccCalibrator& calibrator, int imuIdx,
//                           int figureNumber, bool clearFigure, bool noShow) {
//   auto fig = matplot::figure(figureNumber);
//   if (clearFigure) {
//     fig->clear();
//   }

//   // Plot errors for each axis
//   for (int i = 0; i < 3; ++i) {
//     matplot::subplot(3, 1, i + 1);
//     // Plot axis-specific errors
//     std::vector<double> axis_errors;
//     // Extract from gyroErrors

//     matplot::plot(axis_errors);
//     matplot::xlabel("error index");
//     matplot::ylabel("error (rad/sec)");
//     matplot::grid(matplot::on);
//     matplot::title("Axis " + std::to_string(i));
//   }

//   if (!noShow) {
//     matplot::show();
//   }
// }

// void plotAccelError(const IccCalibrator& calibrator, int imuIdx,
//                     int figureNumber, bool clearFigure, bool noShow) {
//   auto& imuList = calibrator.getImuList();
//   if (imuIdx >= static_cast<int>(imuList.size())) {
//     std::cerr << "IMU index out of range" << std::endl;
//     return;
//   }

//   std::vector<double> errors;
//   // Collect accel errors

//   auto fig = matplot::figure(figureNumber);
//   if (clearFigure) {
//     fig->clear();
//   }

//   matplot::subplot(2, 1, 1);
//   matplot::plot(errors);
//   matplot::xlabel("error index");
//   matplot::ylabel("error (m/s^2) squared");
//   matplot::grid(matplot::on);
//   matplot::title("imu" + std::to_string(imuIdx) + ": acceleration error");

//   if (!noShow) {
//     matplot::show();
//   }
// }

// void plotAccelErrorPerAxis(const IccCalibrator& calibrator, int imuIdx,
//                            int figureNumber, bool clearFigure, bool noShow) {
//   // Similar to plotGyroErrorPerAxis
// }

// void plotAccelBias(const IccCalibrator& calibrator, int imuIdx,
//                    int figureNumber, bool clearFigure, bool noShow) {
//   std::vector<double> times;
//   std::vector<Eigen::Vector3d> biases;

//   // Extract bias values over time from spline

//   plotVectorOverTime(times, biases,
//                      "imu" + std::to_string(imuIdx) + ": Accelerometer bias",
//                      "bias (m/s^2)", "bias", figureNumber, clearFigure, noShow);
// }

// void plotAngularVelocityBias(const IccCalibrator& calibrator, int imuIdx,
//                              int figureNumber, bool clearFigure, bool noShow) {
//   std::vector<double> times;
//   std::vector<Eigen::Vector3d> biases;

//   // Extract bias values over time from spline

//   plotVectorOverTime(times, biases,
//                      "imu" + std::to_string(imuIdx) + ": Gyroscope bias",
//                      "bias (rad/s)", "bias", figureNumber, clearFigure, noShow);
// }

// void plotAngularVelocities(const IccCalibrator& calibrator, int imuIdx,
//                            int figureNumber, bool clearFigure, bool noShow) {
//   std::vector<double> times;
//   std::vector<Eigen::Vector3d> spline_omega;
//   std::vector<Eigen::Vector3d> measured_omega;

//   // Get angular velocities from spline and measurements

//   auto fig = matplot::figure(figureNumber);
//   if (clearFigure) {
//     fig->clear();
//   }

//   // Plot each axis
//   for (int axis = 0; axis < 3; ++axis) {
//     matplot::subplot(3, 1, axis + 1);

//     std::vector<double> spline_vals, measured_vals;
//     for (size_t i = 0; i < times.size(); ++i) {
//       spline_vals.push_back(spline_omega[i][axis]);
//       if (i < measured_omega.size()) {
//         measured_vals.push_back(measured_omega[i][axis]);
//       }
//     }

//     matplot::plot(times, spline_vals)->line_width(2);
//     matplot::hold(matplot::on);
//     matplot::scatter(times, measured_vals);
//     matplot::xlabel("time (s)");
//     matplot::ylabel("omega (rad/s)");
//     matplot::grid(matplot::on);
//     matplot::title("Axis " + std::to_string(axis));
//     matplot::legend({"Spline", "Measured"});
//   }

//   if (!noShow) {
//     matplot::show();
//   }
// }

// void plotAccelerations(const IccCalibrator& calibrator, int imuIdx,
//                        int figureNumber, bool clearFigure, bool noShow) {
//   // Similar to plotAngularVelocities but for accelerations
// }

// void plotVectorOverTime(const std::vector<double>& times,
//                         const std::vector<Eigen::Vector3d>& values,
//                         const std::string& title, const std::string& ylabel,
//                         const std::string& label, int figureNumber,
//                         bool clearFigure, bool noShow, double lineWidth) {
//   if (times.size() != values.size()) {
//     std::cerr << "Times and values size mismatch" << std::endl;
//     return;
//   }

//   auto fig = matplot::figure(figureNumber);
//   if (clearFigure) {
//     fig->clear();
//   }

//   // Plot each component
//   for (int i = 0; i < 3; ++i) {
//     matplot::subplot(3, 1, i + 1);

//     std::vector<double> component;
//     for (const auto& v : values) {
//       component.push_back(v[i]);
//     }

//     matplot::plot(times, component)->line_width(lineWidth);
//     matplot::xlabel("time (s)");
//     matplot::ylabel(ylabel);
//     matplot::grid(matplot::on);

//     if (i == 0 && !title.empty()) {
//       matplot::title(title);
//     }
//   }

//   if (!noShow) {
//     matplot::show();
//   }
// }

// void plotReprojectionScatter(const IccCalibrator& calibrator, int camId,
//                              int figureNumber, bool clearFigure, bool noShow,
//                              const std::string& title) {
//   auto cameraChain = calibrator.getCameraChain();
//   if (!cameraChain || camId >= static_cast<int>(cameraChain->numCameras())) {
//     std::cerr << "Camera index out of range" << std::endl;
//     return;
//   }

//   // Collect reprojection errors
//   std::vector<double> errors_x, errors_y;

//   // Extract from observations

//   auto fig = matplot::figure(figureNumber);
//   if (clearFigure) {
//     fig->clear();
//   }

//   matplot::scatter(errors_x, errors_y);
//   matplot::xlabel("x error (pixels)");
//   matplot::ylabel("y error (pixels)");
//   matplot::grid(matplot::on);

//   if (!title.empty()) {
//     matplot::title(title);
//   } else {
//     matplot::title("cam" + std::to_string(camId) + ": Reprojection errors");
//   }

//   if (!noShow) {
//     matplot::show();
//   }
// }

// // ============================================================================
// // CameraPlot Implementation
// // ============================================================================

// CameraPlot::CameraPlot(const IccCalibrator& calibrator, int camId)
//     : calibrator_(calibrator), camId_(camId) {}

// void CameraPlot::plotReprojectionErrors(int figureNumber, bool clearFigure,
//                                         bool noShow) {
//   plotReprojectionScatter(calibrator_, camId_, figureNumber, clearFigure,
//                           noShow);
// }

// void CameraPlot::plotReprojectionScatter(int figureNumber, bool clearFigure,
//                                          bool noShow) {
//   kalibr::plotReprojectionScatter(calibrator_, camId_, figureNumber,
//                                   clearFigure, noShow);
// }

}  // namespace kalibr
