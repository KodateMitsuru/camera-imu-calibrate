#include <matplot/matplot.h>

#include <Eigen/Core>
#include <kalibr_imu_camera_calibration/IccSensors.hpp>
#include <cmath>
#include <cstdio>
#include <kalibr_imu_camera_calibration/IccCalibrator.hpp>
#include <kalibr_imu_camera_calibration/IccPlots.hpp>
#include <map>
#include <memory>
#include <numeric>
#include <vector>

#include "bsplines/BSpline.hpp"
#include "bsplines/BSplinePose.hpp"
#include "format_utils.hpp"
#include "kalibr_errorterms/EuclideanError.hpp"
#include "matplot/freestanding/axes_functions.h"

namespace kalibr {

// Note: getFigureRegistry() and getOrCreateFigure() are now inline functions
// in IccPlots.hpp, using sm::plot::PlotCollection::global()

// ============================================================================
// Plotting Functions Implementation
// ============================================================================

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

  // Convert to relative time from first timestamp
  if (!timestamps.empty()) {
    const double t0 = timestamps.front();
    for (size_t idx = 1; idx < timestamps.size(); ++idx) {
      times.push_back(timestamps[idx] - t0);  // Relative time in seconds
      rates.push_back((timestamps[idx] - timestamps[idx - 1]) * scale);
    }
  }

  if (rates.empty()) return;

  double rate_avg =
      std::accumulate(rates.begin(), rates.end(), 0.0) / rates.size();
  double rate_std = std::accumulate(
      rates.begin(), rates.end(), 0.0, [rate_avg](double acc, double r) {
        return acc + (r - rate_avg) * (r - rate_avg);
      });
  rate_std = std::sqrt(rate_std / rates.size());

  // Z-test to find outliers and separate inliers/outliers for plotting
  std::vector<double> inlier_times, inlier_rates;
  std::vector<double> outlier_times, outlier_rates;

  for (size_t i = 0; i < rates.size(); ++i) {
    double z =
        (rate_std > 0.0) ? std::abs((rates[i] - rate_avg) / rate_std) : 0.0;
    if (z > z_thresh) {
      outlier_times.push_back(times[i]);
      outlier_rates.push_back(rates[i]);
    } else {
      inlier_times.push_back(times[i]);
      inlier_rates.push_back(rates[i]);
    }
  }

  // Get or create figure (matplot++ manages figure registry internally)
  // Note: matplot::figure() without args returns current figure,
  // matplot::figure(bool) creates figure in quiet mode
  // For multiple figures, matplot++ uses internal registry
  auto fig = getOrCreateFigure(figureNumber);  // Use our registry
  auto ax = fig->current_axes();

  if (clearFigure) {
    ax->clear();
  }

  // Plot inliers in blue with 'x' marker
  if (!inlier_times.empty()) {
    matplot::scatter(inlier_times, inlier_rates)
        ->marker("x")
        .marker_size(6)
        .marker_color("b");
  }

  // Overlay outliers in red with larger 'x' markers
  if (!outlier_times.empty()) {
    matplot::hold(matplot::on);
    matplot::scatter(outlier_times, outlier_rates)
        ->marker("x")
        .marker_size(10)
        .marker_color("r");
    matplot::hold(matplot::off);
  }

  // Set title, labels, grid
  matplot::title("imu" + std::to_string(imuIdx) + ": sample inertial rate");
  matplot::grid(matplot::on);
  matplot::xlabel("time (s)");
  matplot::ylabel("sample rate (" + unit + ")");

  // Set axis limits
  if (!times.empty()) {
    double xmin = *std::min_element(times.begin(), times.end());
    double xmax = *std::max_element(times.begin(), times.end());
    double ymax = *std::max_element(rates.begin(), rates.end());

    matplot::xlim({xmin, xmax});
    matplot::ylim({0.0, ymax});

    // Add text annotation with statistics (positioned in data coordinates)
    double xpos = xmin + 0.05 * (xmax - xmin);
    double ypos = ymax * 0.9;

    // Format the statistics text
    char buffer[128];
    std::snprintf(buffer, sizeof(buffer), "avg dt (%s) = %.2f +- %.4f",
                  unit.c_str(), rate_avg, rate_std);
    matplot::text(xpos, ypos, buffer);
  }

  // Show or just draw based on noShow flag
  if (!noShow) {
    matplot::show();
  }
}

void plotGyroError(const IccCalibrator& calibrator, int imuIdx,
                   int figureNumber, bool clearFigure, bool noShow) {
  auto& imuList = calibrator.getImuList();
  if (imuIdx >= static_cast<int>(imuList.size())) {
    return;  // Invalid index
  }

  auto& imu = imuList[imuIdx];

  // Collect gyro errors (squared norm)
  std::vector<double> errors;
  for (const auto& re : imu->getGyroErrors()) {
    Eigen::VectorXd err = re->vsError();
    errors.push_back(err.dot(err));  // squared error
  }

  if (errors.empty()) return;

  // Get or create figure
  auto fig = getOrCreateFigure(figureNumber);
  auto ax = fig->current_axes();

  if (clearFigure) {
    ax->clear();
  }

  // Subplot 1: Plot all errors
  matplot::subplot(2, 1, 0);
  std::vector<double> indices(errors.size());
  std::iota(indices.begin(), indices.end(), 0.0);
  matplot::plot(indices, errors);
  matplot::xlabel("error index");
  matplot::ylabel("error (rad/sec) squared");
  matplot::grid(matplot::on);
  matplot::title("imu" + std::to_string(imuIdx) + ": angular velocities error");

  // Filter errors: only plot till 5*sigma
  double mean =
      std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
  double sigma = 0.0;
  for (double e : errors) {
    sigma += (e - mean) * (e - mean);
  }
  sigma = std::sqrt(sigma / errors.size());

  std::vector<double> filtered_errors;
  for (double e : errors) {
    if (e < 5 * sigma) {
      filtered_errors.push_back(e);
    }
  }

  // Subplot 2: Histogram of filtered errors
  if (!filtered_errors.empty()) {
    matplot::subplot(2, 1, 1);
    int nbins = std::max(10, static_cast<int>(filtered_errors.size() / 100));
    matplot::hist(filtered_errors, nbins);
    matplot::xlabel("error (rad/s) squared");
    matplot::ylabel("count");
    matplot::grid(matplot::on);
  }

  if (!noShow) {
    matplot::show();
  }
}

void plotGyroErrorPerAxis(const IccCalibrator& calibrator, int imuIdx,
                          int figureNumber, bool clearFigure, bool noShow) {
  auto& imuList = calibrator.getImuList();
  if (imuIdx >= static_cast<int>(imuList.size())) {
    return;  // Invalid index
  }

  auto& imu = imuList[imuIdx];

  // Collect gyro errors per axis
  std::vector<Eigen::VectorXd> errors;
  for (const auto& re : imu->getGyroErrors()) {
    errors.push_back(re->vsError());
  }

  if (errors.empty()) return;

  // Get or create figure
  auto fig = getOrCreateFigure(figureNumber);
  auto ax = fig->current_axes();

  if (clearFigure) {
    ax->clear();
  }

  // Get gyro noise sigma from IMU config
  auto [sigma, _, __] = imu->getImuConfig().getGyroscopeStatistics();

  // Plot errors for each axis (x, y, z)
  for (int axis = 0; axis < 3; ++axis) {
    matplot::subplot(3, 1, axis);

    // Extract errors for this axis
    std::vector<double> axis_errors;
    std::vector<double> indices;
    for (size_t i = 0; i < errors.size(); ++i) {
      if (axis < errors[i].size()) {
        axis_errors.push_back(errors[i][axis]);
        indices.push_back(static_cast<double>(i));
      }
    }

    if (!axis_errors.empty()) {
      // Plot the errors
      matplot::plot(indices, axis_errors);
      matplot::hold(matplot::on);

      // Plot ±3σ reference lines
      double n = static_cast<double>(axis_errors.size());
      std::vector<double> x_ref = {0.0, n};
      std::vector<double> y_pos = {sigma * 3.0, sigma * 3.0};
      std::vector<double> y_neg = {-sigma * 3.0, -sigma * 3.0};

      matplot::plot(x_ref, y_pos, "r--");
      matplot::plot(x_ref, y_neg, "r--");
      matplot::hold(matplot::off);

      // Set labels and limits
      matplot::xlabel("error index");
      matplot::ylabel("error (rad/s)");
      matplot::grid(matplot::on);
      matplot::xlim({0.0, n});

      // Title for first subplot only
      if (axis == 0) {
        matplot::title("imu" + std::to_string(imuIdx) +
                       ": angular velocities error");
      }
    }
  }

  if (!noShow) {
    matplot::show();
  }
}

void plotAccelError(const IccCalibrator& calibrator, int imuIdx,
                    int figureNumber, bool clearFigure, bool noShow) {
  auto& imuList = calibrator.getImuList();
  if (imuIdx >= static_cast<int>(imuList.size())) {
    return;  // Invalid index
  }

  auto& imu = imuList[imuIdx];

  // Collect accel errors (squared norm)
  std::vector<double> errors;
  for (const auto& re : imu->getAccelErrors()) {
    Eigen::VectorXd err = re->vsError();
    errors.push_back(err.dot(err));  // squared error
  }

  if (errors.empty()) return;

  // Get or create figure
  auto fig = getOrCreateFigure(figureNumber);
  auto ax = fig->current_axes();

  if (clearFigure) {
    ax->clear();
  }

  // Subplot 1: Plot all errors
  matplot::subplot(2, 1, 0);
  std::vector<double> indices(errors.size());
  std::iota(indices.begin(), indices.end(), 0.0);
  matplot::plot(indices, errors);
  matplot::xlabel("error index");
  matplot::ylabel("(m/sec*sec) squared");
  matplot::grid(matplot::on);
  matplot::title("imu" + std::to_string(imuIdx) + ": acceleration error");

  // Filter errors: only plot till 5*sigma
  double mean =
      std::accumulate(errors.begin(), errors.end(), 0.0) / errors.size();
  double sigma = 0.0;
  for (double e : errors) {
    sigma += (e - mean) * (e - mean);
  }
  sigma = std::sqrt(sigma / errors.size());

  std::vector<double> filtered_errors;
  for (double e : errors) {
    if (e < 5 * sigma) {
      filtered_errors.push_back(e);
    }
  }

  // Subplot 2: Histogram of filtered errors
  if (!filtered_errors.empty()) {
    matplot::subplot(2, 1, 1);
    int nbins = std::max(10, static_cast<int>(filtered_errors.size() / 100));
    matplot::hist(filtered_errors, nbins);
    matplot::xlabel("(m/s^2) squared");
    matplot::ylabel("Error Number");
    matplot::grid(matplot::on);
  }

  if (!noShow) {
    matplot::show();
  }
}

void plotAccelErrorPerAxis(const IccCalibrator& calibrator, int imuIdx,
                           int figureNumber, bool clearFigure, bool noShow) {
  auto& imuList = calibrator.getImuList();
  if (imuIdx >= static_cast<int>(imuList.size())) {
    return;  // Invalid index
  }

  auto& imu = imuList[imuIdx];

  // Collect accel errors per axis
  std::vector<Eigen::VectorXd> errors;
  for (const auto& re : imu->getAccelErrors()) {
    errors.push_back(re->vsError());
  }

  if (errors.empty()) return;

  // Get or create figure
  auto fig = getOrCreateFigure(figureNumber);
  auto ax = fig->current_axes();

  if (clearFigure) {
    ax->clear();
  }

  // Get accel noise sigma from IMU config
  auto [sigma, _, __] = imu->getImuConfig().getAccelerometerStatistics();

  // Plot errors for each axis (x, y, z)
  for (int axis = 0; axis < 3; ++axis) {
    matplot::subplot(3, 1, axis);

    // Extract errors for this axis
    std::vector<double> axis_errors;
    std::vector<double> indices;
    for (size_t i = 0; i < errors.size(); ++i) {
      if (axis < errors[i].size()) {
        axis_errors.push_back(errors[i][axis]);
        indices.push_back(static_cast<double>(i));
      }
    }

    if (!axis_errors.empty()) {
      // Plot the errors
      matplot::plot(indices, axis_errors);
      matplot::hold(matplot::on);

      // Plot ±3σ reference lines
      double n = static_cast<double>(axis_errors.size());
      std::vector<double> x_ref = {0.0, n};
      std::vector<double> y_pos = {sigma * 3.0, sigma * 3.0};
      std::vector<double> y_neg = {-sigma * 3.0, -sigma * 3.0};

      matplot::plot(x_ref, y_pos, "r--");
      matplot::plot(x_ref, y_neg, "r--");
      matplot::hold(matplot::off);

      // Set labels and limits
      matplot::xlabel("error index");
      matplot::ylabel("error (m/s^2)");
      matplot::grid(matplot::on);
      matplot::xlim({0.0, n});

      // Title for first subplot only
      if (axis == 0) {
        matplot::title("imu" + std::to_string(imuIdx) + ": acceleration error");
      }
    }
  }

  if (!noShow) {
    matplot::show();
  }
}

void plotAccelBias(const IccCalibrator& calibrator, int imuIdx,
                   int figureNumber, bool clearFigure, bool noShow) {
  auto& imuList = calibrator.getImuList();
  if (imuIdx >= static_cast<int>(imuList.size())) {
    return;  // Invalid index
  }

  auto& imu = imuList[imuIdx];

  // Get the accelerometer bias spline via design variable
  auto accelBiasDv = imu->getAccelBiasDv();

  auto& bias = accelBiasDv->spline();

  // Collect timestamps within spline time range
  std::vector<double> timestamps;
  for (const auto& im : imu->getImuData()) {
    double t = im.stamp.toSec();
    if (t > bias.t_min() && t < bias.t_max()) {
      timestamps.push_back(t);
    }
  }

  if (timestamps.empty()) return;

  // Evaluate bias at each timestamp
  std::vector<Eigen::Vector3d> biases;
  for (double t : timestamps) {
    // evalD(t, 0) evaluates the spline at time t with derivative order 0
    biases.push_back(bias.evalD(t, 0));
  }

  // Convert to relative time (remove time offset)
  std::vector<double> times;
  const double t0 = timestamps.front();
  for (double t : timestamps) {
    times.push_back(t - t0);
  }

  // Plot the bias over time
  auto subplots = plotVectorOverTime(times, biases,
                     "imu" + std::to_string(imuIdx) +
                         ": estimated accelerometer bias (imu frame)",
                     "bias (m/s^2)", "", figureNumber, clearFigure, noShow);

  // Get accelerometer random walk sigma for bounds calculation
  auto [_, sigma_rw, __] = imu->getImuConfig().getAccelerometerStatistics();

  // Add ±3σ bounds to each subplot
  for (int i = 0; i < 3; ++i) {
    auto plot = subplots[i];
    plot->hold(matplot::on);

    // Calculate bounds: 3 * sigma_rw * sqrt(time)
    std::vector<double> upper_bound, lower_bound;
    for (size_t j = 0; j < times.size(); ++j) {
      double bound = 3.0 * sigma_rw * std::sqrt(times[j]);
      upper_bound.push_back(biases[j][i] + bound);
      lower_bound.push_back(biases[j][i] - bound);
    }

    // Plot bounds as red dashed lines
    if (!times.empty()) {
      plot->plot(times, upper_bound, "r--");
      plot->plot(times, lower_bound, "r--");
    }

    plot->hold(matplot::off);
  }

  if (!noShow) {
    matplot::show();
  }
}

void plotAngularVelocityBias(const IccCalibrator& calibrator, int imuIdx,
                             int figureNumber, bool clearFigure, bool noShow) {
  auto& imuList = calibrator.getImuList();
  if (imuIdx >= static_cast<int>(imuList.size())) {
    return;  // Invalid index
  }

  auto& imu = imuList[imuIdx];

  // Get the gyroscope bias spline via design variable
  auto gyroBiasDv = imu->getGyroBiasDv();
  auto& bias = gyroBiasDv->spline();

  // Collect timestamps within spline time range
  std::vector<double> timestamps;
  for (const auto& im : imu->getImuData()) {
    double t = im.stamp.toSec();
    if (t > bias.t_min() && t < bias.t_max()) {
      timestamps.push_back(t);
    }
  }

  if (timestamps.empty()) return;

  // Evaluate bias at each timestamp
  std::vector<Eigen::Vector3d> biases;
  for (double t : timestamps) {
    // evalD(t, 0) evaluates the spline at time t with derivative order 0
    biases.push_back(bias.evalD(t, 0));
  }

  // Convert to relative time (remove time offset)
  std::vector<double> times;
  const double t0 = timestamps.front();
  for (double t : timestamps) {
    times.push_back(t - t0);
  }

  // Plot the bias over time
  auto subplots = plotVectorOverTime(
      times, biases,
      "imu" + std::to_string(imuIdx) + ": estimated gyro bias (imu frame)",
      "bias (rad/s)", "", figureNumber, clearFigure, noShow);

  // Get gyroscope random walk sigma for bounds calculation
  auto [_, sigma_rw, __] = imu->getImuConfig().getGyroscopeStatistics();

  // Add ±3σ bounds to each subplot
  for (int i = 0; i < 3; ++i) {
    auto plot = subplots[i];
    plot->hold(matplot::on);

    // Calculate bounds: 3 * sigma_rw * sqrt(time)
    std::vector<double> upper_bound, lower_bound;
    for (size_t j = 0; j < times.size(); ++j) {
      double bound = 3.0 * sigma_rw * std::sqrt(times[j]);
      upper_bound.push_back(biases[j][i] + bound);
      lower_bound.push_back(biases[j][i] - bound);
    }

    // Plot bounds as red dashed lines
    if (!times.empty()) {
      plot->plot(times, upper_bound, "r--");
      plot->plot(times, lower_bound, "r--");
    }

    plot->hold(matplot::off);
  }

  if (!noShow) {
    matplot::show();
  }
}

void plotAngularVelocities(const IccCalibrator& calibrator, int imuIdx,
                           int figureNumber, bool clearFigure, bool noShow) {
  auto& imuList = calibrator.getImuList();
  if (imuIdx >= static_cast<int>(imuList.size())) {
    return;  // Invalid index
  }

  auto& imu = imuList[imuIdx];
  auto bodyspline = calibrator.getPoseDv()->spline();

  // Collect timestamps within spline time range (with time offset)
  std::vector<double> timestamps;
  for (const auto& im : imu->getImuData()) {
    double t = im.stamp.toSec() + imu->getTimeOffset();
    if (t > bodyspline.t_min() && t < bodyspline.t_max()) {
      timestamps.push_back(t);
    }
  }

  if (timestamps.empty()) return;

  // Extract predicted and measured angular velocities from gyro errors
  std::vector<Eigen::Vector3d> predicted_omega;
  std::vector<Eigen::Vector3d> measured_omega;

  for (const auto& err : imu->getGyroErrors()) {
    predicted_omega.push_back(err->getPredictedMeasurement());
    measured_omega.push_back(err->getMeasurement());
  }

  if (predicted_omega.empty()) return;

  // Convert to relative time (remove time offset)
  std::vector<double> times;
  const double t0 = timestamps.front();
  for (double t : timestamps) {
    times.push_back(t - t0);
  }

  // Plot predicted measurements as lines
  auto subplots = plotVectorOverTime(
      times, predicted_omega,
      "Comparison of predicted and measured angular velocities (body frame)",
      "ang. velocity (rad/s)", "est. bodyspline", figureNumber, clearFigure,
      noShow, 3.0);
  // Overlay measured values as scatter points on each subplot
  for (int axis = 0; axis < 3; ++axis) {
    auto plot = subplots[axis];
    plot->hold(matplot::on);

    // Extract measured values for this axis
    std::vector<double> measured_vals;
    for (const auto& omega : measured_omega) {
      measured_vals.push_back(omega[axis]);
    }

    // Plot as 'x' markers
    if (!times.empty() && !measured_vals.empty()) {
      auto scatter_plot = plot->scatter(times, measured_vals);
      scatter_plot->marker("x");
      scatter_plot->marker_size(6);
      scatter_plot->display_name("imu" + std::to_string(imuIdx));
    }

    // Add legend
    plot->legend();
    plot->hold(matplot::off);
  }

  if (!noShow) {
    matplot::show();
  }
}

void plotAccelerations(const IccCalibrator& calibrator, int imuIdx,
                       int figureNumber, bool clearFigure, bool noShow) {
  auto& imuList = calibrator.getImuList();
  if (imuIdx >= static_cast<int>(imuList.size())) {
    return;  // Invalid index
  }

  auto& imu = imuList[imuIdx];
  auto bodyspline = calibrator.getPoseDv()->spline();

  // Collect timestamps within spline time range (with time offset)
  std::vector<double> timestamps;
  for (const auto& im : imu->getImuData()) {
    double t = im.stamp.toSec() + imu->getTimeOffset();
    if (t > bodyspline.t_min() && t < bodyspline.t_max()) {
      timestamps.push_back(t);
    }
  }

  if (timestamps.empty()) return;

  // Extract predicted and measured accelerations from accel errors
  std::vector<Eigen::Vector3d> predicted_accel;
  std::vector<Eigen::Vector3d> measured_accel;

  for (const auto& err : imu->getAccelErrors()) {
    // Cast ErrorTerm to EuclideanError to access measurement methods
    auto euclidean_err =
        std::dynamic_pointer_cast<kalibr_errorterms::EuclideanError>(err);
    if (euclidean_err) {
      predicted_accel.push_back(euclidean_err->getPredictedMeasurement());
      measured_accel.push_back(euclidean_err->getMeasurement());
    }
  }

  if (predicted_accel.empty()) return;

  // Convert to relative time (remove time offset)
  std::vector<double> times;
  const double t0 = timestamps.front();
  for (double t : timestamps) {
    times.push_back(t - t0);
  }

  // Plot predicted measurements as lines
  auto subplots = plotVectorOverTime(
      times, predicted_accel,
      "Comparison of predicted and measured specific force (imu0 frame)",
      "specific force (m/s^2)", "est. bodyspline", figureNumber, clearFigure,
      noShow, 3.0);
  // Overlay measured values as scatter points on each subplot
  for (int axis = 0; axis < 3; ++axis) {
    auto plot = subplots[axis];
    plot->hold(matplot::on);

    // Extract measured values for this axis
    std::vector<double> measured_vals;
    for (const auto& accel : measured_accel) {
      measured_vals.push_back(accel[axis]);
    }

    // Plot as 'x' markers
    if (!times.empty() && !measured_vals.empty()) {
      auto scatter_plot = plot->scatter(times, measured_vals);
      scatter_plot->marker("x");
      scatter_plot->marker_size(6);
      scatter_plot->display_name("imu" + std::to_string(imuIdx));
    }

    // Add legend
    plot->legend();
    plot->hold(matplot::off);
  }

  if (!noShow) {
    matplot::show();
  }
}

std::vector<matplot::axes_handle> plotVectorOverTime(
    const std::vector<double>& times,
    const std::vector<Eigen::Vector3d>& values, const std::string& title,
    const std::string& ylabel, const std::string& label, int figureNumber,
    bool clearFigure, bool noShow, double lineWidth) {
  if (times.size() != values.size()) {
    return {};  // Size mismatch
  }

  // Get or create figure
  auto fig = getOrCreateFigure(figureNumber);
  auto ax = fig->current_axes();
  if (clearFigure) {
    ax->clear();
  }

  // Set overall figure title (suptitle)
  if (!title.empty()) {
    // Note: matplot++ doesn't have direct suptitle, so we'll add it to first
    // subplot
  }
  std::vector<matplot::axes_handle> subplots;
  // Plot each component (X, Y, Z)
  for (int i = 0; i < 3; ++i) {
    auto plot = matplot::subplot(3, 1, i);

    // Extract component values
    std::vector<double> component;
    for (const auto& v : values) {
      component.push_back(v[i]);
    }

    // Plot the component with blue color
    if (!times.empty() && !component.empty()) {
      auto line = plot->plot(times, component, "b-");
      line->line_width(lineWidth);

      // Set display name if label is provided
      if (!label.empty()) {
        line->display_name(label);
      }
    }

    matplot::xlabel("time (s)");
    matplot::ylabel(ylabel);
    matplot::grid(matplot::on);

    // Add title only to first subplot
    if (i == 0 && !title.empty()) {
      matplot::title(title);
    }

    // Show legend only if label is not empty
    if (!label.empty()) {
      matplot::legend();
    }
    subplots.push_back(plot);
  }

  if (!noShow) {
    matplot::show();
  }
  return subplots;
}

void plotReprojectionScatter(const IccCalibrator& calibrator, int camId,
                             int figureNumber, bool clearFigure, bool noShow,
                             const std::string& title) {
  auto cameraChain = calibrator.getCameraChain();
  if (!cameraChain) {
    return;
  }

  auto& camList = cameraChain->getCamList();
  if (camId >= static_cast<int>(camList.size())) {
    return;  // Invalid camera index
  }

  auto& cam = camList[camId];

  // Get or create figure
  auto fig = getOrCreateFigure(figureNumber);
  auto ax = fig->current_axes();

  if (clearFigure) {
    ax->clear();
  }

  // Get all reprojection errors
  auto& allReprojectionErrors = cam->getAllReprojectionErrors();
  size_t numImages = allReprojectionErrors.size();

  if (numImages == 0) return;

  // Plot reprojection errors for each image with different colors
  // Using a simple color gradient from blue to red
  matplot::hold(matplot::on);

  for (size_t image_id = 0; image_id < numImages; ++image_id) {
    const auto& rerrs_image = allReprojectionErrors[image_id];

    // Collect error x and y coordinates for this image
    std::vector<double> errors_x, errors_y;
    for (const auto& rerr : rerrs_image) {
      // Get error from the error term
      Eigen::VectorXd err = rerr->vsError();
      if (err.size() >= 2) {
        errors_x.push_back(err[0]);
        errors_y.push_back(err[1]);
      }
    }

    // Calculate color based on image index (gradient from blue to red)
    // Using HSV color space: hue from 240° (blue) to 0° (red)
    if (!errors_x.empty()) {
      float t =
          static_cast<float>(image_id) / static_cast<float>(numImages - 1);

      // Create color array: interpolate from blue [0, 0, 1] to red [1, 0, 0]
      std::array<float, 4> color = {t, 0.0f, 1.0f - t, 0.5f};  // RGBA with alpha=0.5

      auto scatter = matplot::scatter(errors_x, errors_y);
      scatter->marker("x");
      scatter->marker_size(8);
      scatter->marker_face_color(color);
      scatter->marker_color(color);
    }
  }

  // Add uncertainty bound circle (3 * cornerUncertainty)
  double cornerUncertainty = cam->getCornerUncertainty();
  double radius = 3.0 * cornerUncertainty;

  // Draw circle using parametric equation
  const int npoints = 100;
  std::vector<double> circle_x, circle_y;
  for (int i = 0; i <= npoints; ++i) {
    double theta = 2.0 * M_PI * i / npoints;
    circle_x.push_back(radius * std::cos(theta));
    circle_y.push_back(radius * std::sin(theta));
  }

  auto circle = matplot::plot(circle_x, circle_y, "k--");
  circle->line_width(2.0);
  circle->display_name("3σ uncertainty");

  matplot::hold(matplot::off);

  // Set equal axis
  matplot::axis(matplot::equal);
  matplot::grid(matplot::on);
  matplot::xlabel("error x (pix)");
  matplot::ylabel("error y (pix)");

  // Set title
  if (!title.empty()) {
    matplot::title(title);
  } else {
    matplot::title("cam" + std::to_string(camId) + ": Reprojection errors");
  }

  // Note: Matplot++ doesn't have full colorbar support like matplotlib
  // The color gradient from blue to red represents image index

  if (!noShow) {
    matplot::show();
  }
}

// ============================================================================
// CameraPlot3D Implementation
// ============================================================================

CameraPlot::CameraPlot(int figureNumber, const Eigen::MatrixXd& targetPoints,
                       double camSize)
    : figureNumber_(figureNumber),
      targetPoints_(targetPoints),
      camSize_(camSize),
      initialized_(false) {
  setupFigure();
  plot3Dgrid();

  // Initialize camera at identity pose
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  plot3DCamera(T);
}

void CameraPlot::setupFigure() {
  // Get or create figure
  auto fig = getOrCreateFigure(figureNumber_);

  // Create 3D axes
  auto ax = fig->current_axes();
  ax->clear();

  // Hack to enforce axis equal (set visible range)
  // Matplot++ will handle 3D projection automatically
  const double MAX = 1.0;

  // Set axis limits to enforce equal aspect
  matplot::xlim({-MAX, MAX});
  matplot::ylim({-MAX, MAX});
  matplot::zlim({-MAX, MAX});

  // Enable grid
  matplot::grid(matplot::on);

  // Show the figure
  matplot::show();
}

void CameraPlot::plot3Dgrid() {
  if (targetPoints_.rows() == 0) return;

  // Draw target corners as green scatter points
  std::vector<double> x, y, z;
  for (int i = 0; i < targetPoints_.rows(); ++i) {
    x.push_back(targetPoints_(i, 0));
    y.push_back(targetPoints_(i, 1));
    z.push_back(targetPoints_(i, 2));
  }

  if (!x.empty()) {
    auto scatter = matplot::scatter3(x, y, z);
    scatter->marker_color("g");
    scatter->marker_size(1);
  }

  matplot::hold(matplot::on);

  // Draw coordinate axes from origin to last target point
  if (targetPoints_.rows() > 0) {
    double max_x = targetPoints_(targetPoints_.rows() - 1, 0);
    double max_y = targetPoints_(targetPoints_.rows() - 1, 1);
    double max_z = targetPoints_(targetPoints_.rows() - 1, 0);  // Use x for z

    // X-axis (red)
    std::vector<double> x_axis = {0.0, max_x};
    std::vector<double> y_zero = {0.0, 0.0};
    std::vector<double> z_zero = {0.0, 0.0};
    auto line_x = matplot::plot3(x_axis, y_zero, z_zero);
    line_x->color("r");

    // Y-axis (green)
    std::vector<double> x_zero = {0.0, 0.0};
    std::vector<double> y_axis = {0.0, max_y};
    auto line_y = matplot::plot3(x_zero, y_axis, z_zero);
    line_y->color("g");

    // Z-axis (blue)
    std::vector<double> z_axis = {0.0, max_z};
    auto line_z = matplot::plot3(x_zero, y_zero, z_axis);
    line_z->color("b");
  }

  matplot::hold(matplot::off);
}

void CameraPlot::plot3DCamera(const Eigen::Matrix4d& T) {
  // Transform points
  Eigen::Vector4d ori(0, 0, 0, 1);
  Eigen::Vector4d v1(camSize_, 0, 0, 1);
  Eigen::Vector4d v2(0, camSize_, 0, 1);
  Eigen::Vector4d v3(0, 0, camSize_, 1);

  // Apply transformation
  ori = T * ori;
  v1 = T * v1;
  v2 = T * v2;
  v3 = T * v3;

  // Prepare line data
  std::vector<double> x_line = {ori[0], v1[0]};
  std::vector<double> y_line = {ori[1], v1[1]};
  std::vector<double> z_line = {ori[2], v1[2]};

  std::vector<double> x_line2 = {ori[0], v2[0]};
  std::vector<double> y_line2 = {ori[1], v2[1]};
  std::vector<double> z_line2 = {ori[2], v2[2]};

  std::vector<double> x_line3 = {ori[0], v3[0]};
  std::vector<double> y_line3 = {ori[1], v3[1]};
  std::vector<double> z_line3 = {ori[2], v3[2]};

  if (!initialized_) {
    // First time: create the lines
    matplot::hold(matplot::on);

    auto line_x = matplot::plot3(x_line, y_line, z_line);
    line_x->color("r");
    line_x->line_width(2.0);

    auto line_y = matplot::plot3(x_line2, y_line2, z_line2);
    line_y->color("g");
    line_y->line_width(2.0);

    auto line_z = matplot::plot3(x_line3, y_line3, z_line3);
    line_z->color("b");
    line_z->line_width(2.0);

    matplot::hold(matplot::off);

    initialized_ = true;
  } else {
    // Update: redraw the camera axes
    // Note: Matplot++ requires redrawing for updates
    // We need to clear and redraw, or just overlay new lines
    matplot::hold(matplot::on);

    auto line_x = matplot::plot3(x_line, y_line, z_line);
    line_x->color("r");
    line_x->line_width(2.0);

    auto line_y = matplot::plot3(x_line2, y_line2, z_line2);
    line_y->color("g");
    line_y->line_width(2.0);

    auto line_z = matplot::plot3(x_line3, y_line3, z_line3);
    line_z->color("b");
    line_z->line_width(2.0);

    matplot::hold(matplot::off);

    // Refresh display (equivalent to pl.pause in Python)
    matplot::show();
  }
}

}  // namespace kalibr
