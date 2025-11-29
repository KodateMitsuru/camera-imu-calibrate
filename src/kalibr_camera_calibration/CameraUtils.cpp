#include "kalibr_camera_calibration/CameraUtils.hpp"

#include <Magick++.h>
#include <lunasvg.h>
#include <matplot/matplot.h>

#include <algorithm>
#include <aslam/backend/ReprojectionError.hpp>
#include <aslam/cameras/CameraGeometry.hpp>
#include <cmath>
#include <filesystem>
#include <format>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <print>
#include <sstream>

#include "aslam/cameras.hpp"
#include "kalibr_camera_calibration/CameraCalibrator.hpp"
#include "sm/kinematics/Transformation.hpp"

namespace kalibr {

// ============================================================================
// Helper: Try to cast ErrorTerm to ReprojectionError and get measurement
// ============================================================================

namespace {

// Helper to try dynamic_cast to various ReprojectionError types
template <typename CameraGeometry>
bool tryGetMeasurementFromReprojectionError(
    const aslam::backend::ErrorTerm* errorTerm, Eigen::Vector2d& measurement,
    Eigen::Vector2d& prediction) {
  using ReprojError = aslam::backend::ReprojectionError<CameraGeometry>;
  auto* rerr = dynamic_cast<const ReprojError*>(errorTerm);
  if (rerr) {
    auto m = const_cast<ReprojError*>(rerr)->getMeasurement();
    auto p = const_cast<ReprojError*>(rerr)->getPredictedMeasurement();
    measurement = Eigen::Vector2d(m[0], m[1]);
    prediction = Eigen::Vector2d(p[0], p[1]);
    return true;
  }
  return false;
}

// Try all known camera geometry types
bool getMeasurementFromErrorTerm(const aslam::backend::ErrorTerm* errorTerm,
                                 Eigen::Vector2d& measurement,
                                 Eigen::Vector2d& prediction) {
  if (!errorTerm) return false;

  // Try each camera geometry type
  if (tryGetMeasurementFromReprojectionError<
          aslam::cameras::DistortedPinholeCameraGeometry>(
          errorTerm, measurement, prediction))
    return true;
  if (tryGetMeasurementFromReprojectionError<
          aslam::cameras::EquidistantDistortedPinholeCameraGeometry>(
          errorTerm, measurement, prediction))
    return true;
  if (tryGetMeasurementFromReprojectionError<
          aslam::cameras::FovDistortedPinholeCameraGeometry>(
          errorTerm, measurement, prediction))
    return true;
  if (tryGetMeasurementFromReprojectionError<
          aslam::cameras::PinholeCameraGeometry>(errorTerm, measurement,
                                                 prediction))
    return true;
  if (tryGetMeasurementFromReprojectionError<
          aslam::cameras::DistortedOmniCameraGeometry>(errorTerm, measurement,
                                                       prediction))
    return true;
  if (tryGetMeasurementFromReprojectionError<
          aslam::cameras::OmniCameraGeometry>(errorTerm, measurement,
                                              prediction))
    return true;
  if (tryGetMeasurementFromReprojectionError<
          aslam::cameras::ExtendedUnifiedCameraGeometry>(errorTerm, measurement,
                                                         prediction))
    return true;
  if (tryGetMeasurementFromReprojectionError<
          aslam::cameras::DoubleSphereCameraGeometry>(errorTerm, measurement,
                                                      prediction))
    return true;

  return false;
}

}  // namespace

// ============================================================================
// Utility Functions
// ============================================================================

Eigen::Vector3d getImageCenterRay(
    const std::shared_ptr<aslam::cameras::CameraGeometryBase>& geometry) {
  // Get image center (use width/height as proxy for principal point)
  double cu = geometry->width() / 2.0;
  double cv = geometry->height() / 2.0;
  Eigen::VectorXd yc(2);
  yc << cu, cv;

  // Convert to Euclidean ray using virtual function
  Eigen::Vector3d ray;
  geometry->vsKeypointToEuclidean(yc, ray);
  return normalize(ray);
}

// ============================================================================
// Point Statistics
// ============================================================================

PointStatistics getPointStatistics(
    const std::shared_ptr<aslam::cameras::CameraGeometryBase>& geometry,
    const aslam::cameras::GridCalibrationTargetObservation& obs, int pidx,
    const std::shared_ptr<aslam::backend::ErrorTerm>& rerr) {
  PointStatistics result;
  result.pidx = pidx;

  // Check if point was observed
  Eigen::Vector2d y;
  bool valid = obs.imagePoint(pidx, y);
  result.valid = valid;

  if (!valid) {
    return result;
  }

  result.y = y;

  // Get image center ray for angle calculations
  Eigen::Vector3d vc = getImageCenterRay(geometry);
  Eigen::Vector3d z(0, 0, 1);
  double dz = z.dot(vc);

  if (std::abs(dz - 1.0) > 1e-3) {
    std::cerr << "Warning: Point statistics assume camera points down z-axis. "
              << "Image center ray: [" << vc.transpose() << "]" << std::endl;
  }

  // Convert keypoint to Euclidean ray using virtual function
  Eigen::VectorXd yv(2);
  yv << y[0], y[1];
  Eigen::Vector3d v;
  geometry->vsKeypointToEuclidean(yv, v);
  v = normalize(v);

  // Calculate polar and azimuthal angles
  result.polarAngle = std::acos(std::clamp(v[2], -1.0, 1.0));
  result.azimuthalAngle = std::atan2(v[1], v[0]);

  // If we have a reprojection error term, extract error info
  if (rerr) {
    Eigen::VectorXd e = rerr->vsError();
    if (e.size() >= 2) {
      result.e = Eigen::Vector2d(e[0], e[1]);
      result.yhat = y - Eigen::Vector2d(e[0], e[1]);
      result.squaredError = e.squaredNorm();
    }
  }

  return result;
}

std::vector<PointStatistics> getAllPointStatistics(
    const CameraCalibration& calibrator, size_t camId) {
  std::vector<PointStatistics> stats;

  const auto& cameras = calibrator.getCameras();
  const auto& views = calibrator.getViews();
  const auto& target = calibrator.getTarget();

  if (camId >= cameras.size()) {
    return stats;
  }

  auto geometry = cameras[camId]->getGeometry();
  size_t targetSize = target->size();

  // Iterate over all views
  for (size_t viewIdx = 0; viewIdx < views.size(); ++viewIdx) {
    const auto& view = views[viewIdx];

    // Check if this camera has observations in this view
    auto it = view->rerrs.find(camId);
    if (it == view->rerrs.end()) {
      continue;
    }

    // Find observation for this camera
    const aslam::cameras::GridCalibrationTargetObservation* obsPtr = nullptr;
    for (const auto& [cid, obs] : view->rig_observations) {
      if (cid == camId) {
        obsPtr = &obs;
        break;
      }
    }

    if (!obsPtr) continue;

    const auto& rerrs = it->second;

    // For each target point
    for (size_t pidx = 0; pidx < targetSize; ++pidx) {
      std::shared_ptr<aslam::backend::ErrorTerm> rerr = nullptr;

      // Find error term for this point (rerrs is indexed by corner order)
      std::vector<unsigned int> cornerIdx;
      obsPtr->getCornersIdx(cornerIdx);
      auto cornerIt = std::find(cornerIdx.begin(), cornerIdx.end(),
                                static_cast<unsigned int>(pidx));
      if (cornerIt != cornerIdx.end()) {
        size_t errIdx = std::distance(cornerIdx.begin(), cornerIt);
        if (errIdx < rerrs.size()) {
          rerr = rerrs[errIdx];
        }
      }

      PointStatistics stat =
          getPointStatistics(geometry, *obsPtr, static_cast<int>(pidx), rerr);
      stat.view_id = static_cast<int>(viewIdx);
      stat.cam_id = static_cast<int>(camId);

      if (stat.valid) {
        stats.push_back(stat);
      }
    }
  }

  return stats;
}

// ============================================================================
// Reprojection Error Statistics
// ============================================================================

std::vector<ViewReprojectionData> getReprojectionErrors(
    const CameraCalibration& calibrator, size_t camId) {
  std::vector<ViewReprojectionData> result;

  const auto& views = calibrator.getViews();

  for (size_t viewIdx = 0; viewIdx < views.size(); ++viewIdx) {
    ViewReprojectionData viewData;
    const auto& view = views[viewIdx];

    // Check if this camera has reprojection errors in this view
    auto it = view->rerrs.find(camId);
    if (it == view->rerrs.end()) {
      viewData.valid = false;
      result.push_back(viewData);
      continue;
    }

    viewData.valid = true;
    const auto& viewErrors = it->second;

    for (const auto& rerr : viewErrors) {
      if (rerr) {
        Eigen::Vector2d measurement, prediction;
        if (getMeasurementFromErrorTerm(rerr.get(), measurement, prediction)) {
          // Successfully cast to ReprojectionError
          Eigen::Vector2d err = measurement - prediction;
          viewData.corners.push_back(measurement);
          viewData.reprojections.push_back(prediction);
          viewData.errors.push_back(err);
        } else {
          // Fallback: use vsError() only
          Eigen::VectorXd error = rerr->vsError();
          if (error.size() >= 2) {
            Eigen::Vector2d err(error[0], error[1]);
            viewData.corners.push_back(std::nullopt);
            viewData.reprojections.push_back(std::nullopt);
            viewData.errors.push_back(err);
          } else {
            viewData.corners.push_back(std::nullopt);
            viewData.reprojections.push_back(std::nullopt);
            viewData.errors.push_back(std::nullopt);
          }
        }
      } else {
        viewData.corners.push_back(std::nullopt);
        viewData.reprojections.push_back(std::nullopt);
        viewData.errors.push_back(std::nullopt);
      }
    }

    result.push_back(viewData);
  }

  return result;
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> getReprojectionErrorStatistics(
    const std::vector<ViewReprojectionData>& allViewData) {
  std::vector<Eigen::Vector2d> allErrors;

  for (const auto& viewData : allViewData) {
    if (!viewData.valid) continue;

    for (const auto& errOpt : viewData.errors) {
      if (errOpt.has_value()) {
        allErrors.push_back(errOpt.value());
      }
    }
  }

  if (allErrors.empty()) {
    return {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};
  }

  // Compute mean of error components (x, y)
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const auto& e : allErrors) {
    mean += e;
  }
  mean /= static_cast<double>(allErrors.size());

  // Compute std of error components
  Eigen::Vector2d var = Eigen::Vector2d::Zero();
  for (const auto& e : allErrors) {
    Eigen::Vector2d diff = e - mean;
    var += diff.cwiseProduct(diff);
  }
  var /= static_cast<double>(allErrors.size());
  Eigen::Vector2d stddev = var.cwiseSqrt();

  return {mean, stddev};
}

// Get reprojection error norm statistics (mean and std of pixel distance)
std::pair<double, double> getReprojectionErrorNormStatistics(
    const std::vector<ViewReprojectionData>& allViewData) {
  std::vector<double> allNorms;

  for (const auto& viewData : allViewData) {
    if (!viewData.valid) continue;

    for (const auto& errOpt : viewData.errors) {
      if (errOpt.has_value()) {
        allNorms.push_back(errOpt.value().norm());
      }
    }
  }

  if (allNorms.empty()) {
    return {0.0, 0.0};
  }

  // Compute mean of error norms
  double mean = 0.0;
  for (double n : allNorms) {
    mean += n;
  }
  mean /= static_cast<double>(allNorms.size());

  // Compute std of error norms
  double var = 0.0;
  for (double n : allNorms) {
    double diff = n - mean;
    var += diff * diff;
  }
  var /= static_cast<double>(allNorms.size());
  double stddev = std::sqrt(var);

  return {mean, stddev};
}

// ============================================================================
// Result Export / Printing
// ============================================================================

void exportPoses(
    const std::vector<std::pair<double, sm::kinematics::Transformation>>& poses,
    const std::string& filename) {
  std::ofstream f(filename);
  if (!f.is_open()) {
    std::cerr << "Failed to open file for writing: " << filename << std::endl;
    return;
  }

  // Header (ETH groundtruth format)
  f << "#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], "
       "q_RS_x [], q_RS_y [], q_RS_z []"
    << std::endl;

  f << std::fixed << std::setprecision(6);

  // Sort by timestamp
  auto sortedPoses = poses;
  std::sort(sortedPoses.begin(), sortedPoses.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

  for (const auto& [timestamp, T] : sortedPoses) {
    Eigen::Vector3d position = T.t();
    Eigen::Vector4d orientation = T.q();  // w, x, y, z

    // Format: timestamp (ns), px, py, pz, qw, qx, qy, qz
    f << std::setprecision(0) << (timestamp * 1e9) << ",";
    f << std::setprecision(6) << position[0] << "," << position[1] << ","
      << position[2] << ",";
    f << orientation[0] << "," << orientation[1] << "," << orientation[2] << ","
      << orientation[3] << std::endl;
  }

  f.close();
}

void saveResultTxt(const CameraCalibration& calibrator,
                   const std::string& filename) {
  std::ofstream f(filename);
  if (!f.is_open()) {
    std::cerr << "Failed to open file for writing: " << filename << std::endl;
    return;
  }
  printParameters(calibrator, f);
  f.close();
}

void printParameters(const CameraCalibration& calibrator, std::ostream& out) {
  out << "Calibration results" << std::endl;
  out << "====================" << std::endl;
  out << std::endl;

  const auto& cameras = calibrator.getCameras();
  const auto& baselines = calibrator.getBaselines();

  // Print camera parameters
  out << "Camera-system parameters:" << std::endl;
  for (size_t cidx = 0; cidx < cameras.size(); ++cidx) {
    const auto& cam = cameras[cidx];
    auto geometry = cam->getGeometry();

    out << "cam" << cidx << " (" << cam->getDataset().getTopic()
        << "):" << std::endl;

    // Get projection and distortion parameters using virtual functions
    Eigen::MatrixXd projParams, distParams;
    geometry->getParameters(projParams, true, false, false);  // projection
    geometry->getParameters(distParams, false, true, false);  // distortion

    out << "    distortion: [";
    for (int i = 0; i < distParams.size(); ++i) {
      if (i > 0) out << ", ";
      out << distParams(i);
    }
    out << "]" << std::endl;

    out << "    projection: [";
    for (int i = 0; i < projParams.size(); ++i) {
      if (i > 0) out << ", ";
      out << projParams(i);
    }
    out << "]" << std::endl;

    // Reprojection error statistics
    auto viewData = getReprojectionErrors(calibrator, cidx);
    if (!viewData.empty()) {
      auto [mean, stddev] = getReprojectionErrorStatistics(viewData);
      auto [normMean, normStd] = getReprojectionErrorNormStatistics(viewData);
      out << "    reprojection error: [" << mean[0] << ", " << mean[1]
          << "] +- [" << stddev[0] << ", " << stddev[1] << "]" << std::endl;
      out << "    reprojection error (norm): " << normMean << " +- " << normStd
          << " [pixels]" << std::endl;
    }
    out << std::endl;
  }

  // Print baselines
  for (size_t bidx = 0; bidx < baselines.size(); ++bidx) {
    sm::kinematics::Transformation T(baselines[bidx]->T());

    out << "baseline T_" << (bidx + 1) << "_" << bidx << ":" << std::endl;
    out << "    q: [" << T.q().transpose() << "]" << std::endl;
    out << "    t: [" << T.t().transpose() << "]" << std::endl;
    out << std::endl;
  }
}

// ============================================================================
// Plotting Functions
// ============================================================================

void plotPolarError(const CameraCalibration& calibrator, size_t camId,
                    int figureNumber, bool clearFigure,
                    const std::vector<PointStatistics>* stats, bool noShow,
                    const std::string& title) {
  // Get or compute statistics
  std::vector<PointStatistics> localStats;
  if (stats == nullptr) {
    localStats = getAllPointStatistics(calibrator, camId);
    stats = &localStats;
  }

  // Extract polar angle and error pairs
  std::vector<std::pair<double, double>> angleError;
  for (const auto& s : *stats) {
    if (s.polarAngle.has_value() && s.squaredError.has_value()) {
      double angleDeg = s.polarAngle.value() * 180.0 / M_PI;
      double error = std::sqrt(s.squaredError.value());
      angleError.emplace_back(angleDeg, error);
    }
  }

  if (angleError.empty()) return;

  // Sort by polar angle
  std::sort(angleError.begin(), angleError.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

  // Extract for plotting
  std::vector<double> angles, errors;
  for (const auto& [a, e] : angleError) {
    angles.push_back(a);
    errors.push_back(e);
  }

  // Create figure
  auto fig = getOrCreateCameraUtilsFigure(figureNumber);
  if (clearFigure) {
    fig->current_axes()->clear();
  }

  // Subplot 1: error vs angle
  matplot::subplot(1, 2, 0);
  if (!title.empty()) {
    matplot::title(title);
  }
  matplot::plot(angles, errors, "bx-");
  matplot::grid(matplot::on);
  matplot::xlabel("polar angle (deg)");
  matplot::ylabel("reprojection error (pixels)");

  // Subplot 2: histogram of angles
  matplot::subplot(1, 2, 1);
  matplot::hist(angles);
  matplot::grid(matplot::on);
  matplot::xlabel("polar angle (deg)");
  matplot::ylabel("count");

  if (!noShow) {
    matplot::show();
  }
}

void plotAzimuthalError(const CameraCalibration& calibrator, size_t camId,
                        int figureNumber, bool clearFigure,
                        const std::vector<PointStatistics>* stats, bool noShow,
                        const std::string& title) {
  // Get or compute statistics
  std::vector<PointStatistics> localStats;
  if (stats == nullptr) {
    localStats = getAllPointStatistics(calibrator, camId);
    stats = &localStats;
  }

  // Extract azimuthal angle and error pairs
  std::vector<std::pair<double, double>> angleError;
  for (const auto& s : *stats) {
    if (s.azimuthalAngle.has_value() && s.squaredError.has_value()) {
      double angleDeg = s.azimuthalAngle.value() * 180.0 / M_PI;
      double error = std::sqrt(s.squaredError.value());
      angleError.emplace_back(angleDeg, error);
    }
  }

  if (angleError.empty()) return;

  // Sort by azimuthal angle
  std::sort(angleError.begin(), angleError.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

  // Extract for plotting
  std::vector<double> angles, errors;
  for (const auto& [a, e] : angleError) {
    angles.push_back(a);
    errors.push_back(e);
  }

  // Create figure
  auto fig = getOrCreateCameraUtilsFigure(figureNumber);
  if (clearFigure) {
    fig->current_axes()->clear();
  }

  // Subplot 1: error vs angle
  matplot::subplot(1, 2, 0);
  if (!title.empty()) {
    matplot::title(title);
  }
  matplot::plot(angles, errors, "bx-");
  matplot::grid(matplot::on);
  matplot::xlabel("azimuthal angle (deg)");
  matplot::ylabel("reprojection error (pixels)");

  // Subplot 2: histogram of angles
  matplot::subplot(1, 2, 1);
  matplot::hist(angles);
  matplot::grid(matplot::on);
  matplot::xlabel("azimuthal angle (deg)");
  matplot::ylabel("count");

  if (!noShow) {
    matplot::show();
  }
}

void plotAllReprojectionErrors(const CameraCalibration& calibrator,
                               size_t camId, int figureNumber, bool noShow,
                               bool clearFigure, const std::string& title) {
  auto viewData = getReprojectionErrors(calibrator, camId);

  // Get resolution using virtual functions
  const auto& cameras = calibrator.getCameras();
  if (camId >= cameras.size()) return;

  auto geometry = cameras[camId]->getGeometry();
  int resU = geometry->width();
  int resV = geometry->height();

  // Create figure
  auto fig = getOrCreateCameraUtilsFigure(figureNumber);
  if (clearFigure) {
    fig->current_axes()->clear();
  }

  size_t numViews = viewData.size();

  // Left: detected corners
  matplot::subplot(1, 2, 0);
  if (!title.empty()) {
    matplot::title(title);
  }
  matplot::hold(matplot::on);

  for (size_t viewIdx = 0; viewIdx < numViews; ++viewIdx) {
    if (!viewData[viewIdx].valid) continue;

    // Color based on view index (blue to red gradient)
    float t = numViews > 1 ? static_cast<float>(viewIdx) /
                                 static_cast<float>(numViews - 1)
                           : 0.5f;

    std::vector<double> cx, cy;
    for (const auto& cOpt : viewData[viewIdx].corners) {
      if (cOpt.has_value()) {
        cx.push_back(cOpt.value()[0]);
        cy.push_back(cOpt.value()[1]);
      }
    }

    if (!cx.empty()) {
      auto scatter = matplot::scatter(cx, cy);
      scatter->marker("o");
      scatter->marker_size(4);
      std::array<float, 4> color = {t, 0.0f, 1.0f - t, 0.5f};
      scatter->marker_face_color(color);
    }
  }

  matplot::xlim({0.0, static_cast<double>(resU)});
  matplot::ylim({static_cast<double>(resV), 0.0});  // Flip Y for image coords
  matplot::hold(matplot::off);

  // Right: reprojection error scatter
  matplot::subplot(1, 2, 1);
  matplot::hold(matplot::on);

  for (size_t viewIdx = 0; viewIdx < numViews; ++viewIdx) {
    if (!viewData[viewIdx].valid) continue;

    float t = numViews > 1 ? static_cast<float>(viewIdx) /
                                 static_cast<float>(numViews - 1)
                           : 0.5f;

    std::vector<double> ex, ey;
    for (const auto& eOpt : viewData[viewIdx].errors) {
      if (eOpt.has_value()) {
        ex.push_back(eOpt.value()[0]);
        ey.push_back(eOpt.value()[1]);
      }
    }

    if (!ex.empty()) {
      auto scatter = matplot::scatter(ex, ey);
      scatter->marker("x");
      scatter->marker_size(6);
      std::array<float, 4> color = {t, 0.0f, 1.0f - t, 0.5f};
      scatter->marker_color(color);
    }
  }

  matplot::axis(matplot::equal);
  matplot::grid(matplot::on);
  matplot::xlabel("error x (pix)");
  matplot::ylabel("error y (pix)");
  matplot::hold(matplot::off);

  if (!noShow) {
    matplot::show();
  }
}

void plotCornersAndReprojection(
    const aslam::cameras::GridCalibrationTargetObservation& obs,
    const std::vector<std::optional<Eigen::Vector2d>>& reprojections,
    int figureNumber, const std::vector<int>* cornerList, bool clearFigure,
    bool plotImage, const std::array<float, 4>& color,
    const std::string& title) {
  auto fig = getOrCreateCameraUtilsFigure(figureNumber);
  if (clearFigure) {
    fig->current_axes()->clear();
  }

  if (!title.empty()) {
    matplot::title(title);
  }

  // Plot corners first
  plotCorners(obs, figureNumber, cornerList, false, plotImage);

  // Plot reprojections
  matplot::hold(matplot::on);
  for (size_t pidx = 0; pidx < reprojections.size(); ++pidx) {
    if (cornerList != nullptr) {
      if (std::find(cornerList->begin(), cornerList->end(),
                    static_cast<int>(pidx)) == cornerList->end()) {
        continue;
      }
    }

    if (reprojections[pidx].has_value()) {
      const auto& rp = reprojections[pidx].value();
      std::vector<double> rpx = {rp[0]};
      std::vector<double> rpy = {rp[1]};
      auto scatter = matplot::scatter(rpx, rpy);
      scatter->marker("x");
      scatter->marker_size(8);
      scatter->marker_color(color);
    }
  }
  matplot::hold(matplot::off);

  matplot::xlim({0.0, static_cast<double>(obs.imCols())});
  matplot::ylim({static_cast<double>(obs.imRows()), 0.0});
}

void plotCorners(const aslam::cameras::GridCalibrationTargetObservation& obs,
                 int figureNumber, const std::vector<int>* cornerList,
                 bool clearFigure, bool plotImage,
                 const std::array<float, 4>& color, int subplot) {
  auto fig = getOrCreateCameraUtilsFigure(figureNumber);

  if (subplot > 0) {
    // matplot::subplot uses 0-indexed rows/cols
    // For simplicity, assume subplot is a linear index
  }

  if (clearFigure) {
    fig->current_axes()->clear();
  }

  // Plot image if available and requested
  if (plotImage) {
    cv::Mat img = obs.image();
    if (!img.empty() && img.rows > 0 && img.cols > 0) {
      // Convert to grayscale if needed
      cv::Mat grayImg;
      if (img.channels() == 3) {
        cv::cvtColor(img, grayImg, cv::COLOR_BGR2GRAY);
      } else {
        grayImg = img;
      }

      // Convert to matplot format
      std::vector<std::vector<double>> imgData(
          grayImg.rows, std::vector<double>(grayImg.cols));
      for (int r = 0; r < grayImg.rows; ++r) {
        for (int c = 0; c < grayImg.cols; ++c) {
          imgData[r][c] = grayImg.at<uchar>(r, c) / 255.0;
        }
      }
      matplot::imagesc(imgData);
    }
  }

  // Get corners
  std::vector<cv::Point2f> imageCorners;
  obs.getCornersImageFrame(imageCorners);

  std::vector<double> px, py;

  if (cornerList != nullptr) {
    for (int pidx : *cornerList) {
      Eigen::Vector2d pt;
      if (obs.imagePoint(pidx, pt)) {
        px.push_back(pt[0]);
        py.push_back(pt[1]);
      }
    }
  } else {
    for (const auto& p : imageCorners) {
      px.push_back(p.x);
      py.push_back(p.y);
    }
  }

  matplot::hold(matplot::on);
  if (!px.empty()) {
    auto scatter = matplot::scatter(px, py);
    scatter->marker("o");
    scatter->marker_size(6);
    scatter->marker_face_color(color);
    scatter->marker_color(color);
  }
  matplot::hold(matplot::off);

  matplot::xlim({0.0, static_cast<double>(obs.imCols())});
  matplot::ylim({static_cast<double>(obs.imRows()), 0.0});
}

void plotTrajectory(
    const std::vector<std::pair<double, sm::kinematics::Transformation>>& poses,
    int figureNumber, bool clearFigure, const std::string& title) {
  auto fig = getOrCreateCameraUtilsFigure(figureNumber);
  if (clearFigure) {
    fig->current_axes()->clear();
  }

  if (!title.empty()) {
    matplot::title(title);
  }

  if (poses.empty()) return;

  double size = 0.05;

  // Sort poses by timestamp
  auto sortedPoses = poses;
  std::sort(sortedPoses.begin(), sortedPoses.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

  // Extract positions
  std::vector<double> x, y, z;
  Eigen::Vector3d traj_min(1e9, 1e9, 1e9);
  Eigen::Vector3d traj_max(-1e9, -1e9, -1e9);

  for (const auto& [ts, T] : sortedPoses) {
    Eigen::Vector3d t = T.t();
    x.push_back(t[0]);
    y.push_back(t[1]);
    z.push_back(t[2]);

    traj_min = traj_min.cwiseMin(t);
    traj_max = traj_max.cwiseMax(t);
  }

  // Plot trajectory line
  matplot::plot3(x, y, z, "k-");
  matplot::hold(matplot::on);

  // Plot coordinate frames at each pose (simplified: just origin points)
  matplot::scatter3(x, y, z)->marker("o").marker_size(3);

  matplot::hold(matplot::off);

  // Set axis limits
  matplot::xlim({traj_min[0] - size, traj_max[0] + size});
  matplot::ylim({traj_min[1] - size, traj_max[1] + size});
  matplot::zlim({traj_min[2] - size, traj_max[2] + size});
}

void plotCameraRig(const std::vector<sm::kinematics::Transformation>& baselines,
                   int figureNumber, bool clearFigure,
                   const std::string& title) {
  auto fig = getOrCreateCameraUtilsFigure(figureNumber);
  if (clearFigure) {
    fig->current_axes()->clear();
  }

  if (!title.empty()) {
    matplot::title(title);
  }

  if (baselines.empty()) return;

  // Convert baselines to camera frames in coords of first camera
  std::vector<sm::kinematics::Transformation> cameraFrames;
  cameraFrames.push_back(
      sm::kinematics::Transformation());  // Identity for cam0

  for (size_t i = 0; i < baselines.size(); ++i) {
    cameraFrames.push_back(cameraFrames[i] * baselines[i].inverse());
  }

  // Get size (half shortest baseline)
  double minBaseline = 1e9;
  for (const auto& b : baselines) {
    double norm = b.t().norm();
    if (norm > 1e-6 && norm < minBaseline) {
      minBaseline = norm;
    }
  }
  double size = 0.5 * minBaseline;

  // Get box size
  double maxDist = 0.0;
  for (const auto& cf : cameraFrames) {
    double dist = cf.t().norm();
    if (dist > maxDist) maxDist = dist;
  }
  double boxSize = 1.25 * maxDist + size;

  matplot::hold(matplot::on);

  // Plot each camera frame
  for (size_t i = 0; i < cameraFrames.size(); ++i) {
    const auto& T = cameraFrames[i];
    Eigen::Vector3d origin = T.t();
    Eigen::Matrix3d R = T.C();

    // X axis (red)
    Eigen::Vector3d xEnd = origin + size * R.col(0);
    std::vector<double> x_line = {origin[0], xEnd[0]};
    std::vector<double> y_line = {origin[1], xEnd[1]};
    std::vector<double> z_line = {origin[2], xEnd[2]};
    matplot::plot3(x_line, y_line, z_line, "r-");

    // Y axis (green)
    Eigen::Vector3d yEnd = origin + size * R.col(1);
    std::vector<double> x_line2 = {origin[0], yEnd[0]};
    std::vector<double> y_line2 = {origin[1], yEnd[1]};
    std::vector<double> z_line2 = {origin[2], yEnd[2]};
    matplot::plot3(x_line2, y_line2, z_line2, "g-");

    // Z axis (blue)
    Eigen::Vector3d zEnd = origin + size * R.col(2);
    std::vector<double> x_line3 = {origin[0], zEnd[0]};
    std::vector<double> y_line3 = {origin[1], zEnd[1]};
    std::vector<double> z_line3 = {origin[2], zEnd[2]};
    matplot::plot3(x_line3, y_line3, z_line3, "b-");
  }

  matplot::hold(matplot::off);

  matplot::xlim({-boxSize, boxSize});
  matplot::ylim({-boxSize, boxSize});
  matplot::zlim({-boxSize, boxSize});

  matplot::xlabel("x");
  matplot::ylabel("y");
  matplot::zlabel("z");
}

void plotOutlierCorners(
    const CameraCalibration& calibrator,
    const std::vector<std::pair<int, Eigen::Vector2d>>& removedOutlierCorners,
    int figureNumber, bool clearFigure, const std::string& title) {
  auto fig = getOrCreateCameraUtilsFigure(figureNumber);
  if (clearFigure) {
    fig->current_axes()->clear();
  }

  if (!title.empty()) {
    matplot::title(title);
  }

  const auto& cameras = calibrator.getCameras();
  size_t numCams = cameras.size();

  int subplotRows = static_cast<int>(std::ceil(std::sqrt(numCams)));
  int subplotCols = static_cast<int>(std::ceil(std::sqrt(numCams)));

  for (size_t cidx = 0; cidx < numCams; ++cidx) {
    matplot::subplot(subplotRows, subplotCols, static_cast<int>(cidx));

    // Extract corners for this camera
    std::vector<double> cx, cy;
    for (const auto& [camId, corner] : removedOutlierCorners) {
      if (camId == static_cast<int>(cidx)) {
        cx.push_back(corner[0]);
        cy.push_back(corner[1]);
      }
    }

    if (!cx.empty()) {
      auto scatter = matplot::scatter(cx, cy);
      scatter->marker("x");
      scatter->marker_color("r");
    }

    // Get resolution using virtual functions
    auto geometry = cameras[cidx]->getGeometry();
    int resU = geometry->width();
    int resV = geometry->height();

    matplot::xlim({0.0, static_cast<double>(resU)});
    matplot::ylim({static_cast<double>(resV), 0.0});
    matplot::title("cam" + std::to_string(cidx));
  }
}

void generateReport(
    const CameraCalibration& calibrator, const std::string& filename,
    bool showOnScreen,
    const std::vector<std::pair<int, Eigen::Vector2d>>* removedOutlierCorners) {
  std::vector<std::shared_ptr<matplot::figure_type>> figs;
  int offset = 3010;

  std::println("Generating camera calibration report...");
  std::println("Target filename: {}", filename);

  const auto& cameras = calibrator.getCameras();
  const auto& baselines = calibrator.getBaselines();

  // =========================================================================
  // Create text pages with results
  // =========================================================================
  std::stringstream sstream;
  printParameters(calibrator, sstream);

  // Split text into lines
  std::vector<std::string> text;
  for (std::string line; std::getline(sstream, line);) {
    text.push_back(line);
  }

  size_t linesPerPage = 40;
  size_t textIdx = 0;

  // Create text pages
  while (textIdx < text.size()) {
    auto fig = getOrCreateCameraUtilsFigure(offset);
    offset += 1;

    // Get current axes
    auto ax = fig->current_axes();
    ax->xlim({0, 1});
    ax->ylim({0, 1});

    // Turn off axis
    matplot::axis(matplot::off);

    // Prepare text for this page
    size_t endIdx = std::min(textIdx + linesPerPage, text.size());
    std::string pageText;
    for (size_t i = textIdx; i < endIdx; ++i) {
      pageText += text[i];
      pageText += "\\n";
    }
    auto t = ax->text(0, 1, pageText);
    t->font_size(12);

    figs.push_back(fig);
    textIdx = endIdx;
  }

  // =========================================================================
  // Plot for each camera
  // =========================================================================
  for (size_t cidx = 0; cidx < cameras.size(); ++cidx) {
    // Polar error
    {
      int fno = offset++;
      std::string t = "cam" + std::to_string(cidx) + ": polar error";
      plotPolarError(calibrator, cidx, fno, true, nullptr, true, t);
      figs.push_back(getOrCreateCameraUtilsFigure(fno));
    }

    // Azimuthal error
    {
      int fno = offset++;
      std::string t = "cam" + std::to_string(cidx) + ": azimuthal error";
      plotAzimuthalError(calibrator, cidx, fno, true, nullptr, true, t);
      figs.push_back(getOrCreateCameraUtilsFigure(fno));
    }

    // Reprojection errors
    {
      int fno = offset++;
      std::string t = "cam" + std::to_string(cidx) + ": reprojection errors";
      plotAllReprojectionErrors(calibrator, cidx, fno, true, true, t);
      figs.push_back(getOrCreateCameraUtilsFigure(fno));
    }
  }

  // =========================================================================
  // Camera rig (if multi-camera)
  // =========================================================================
  if (cameras.size() > 1) {
    std::vector<sm::kinematics::Transformation> baselineTransforms;
    for (const auto& b : baselines) {
      baselineTransforms.push_back(sm::kinematics::Transformation(b->T()));
    }

    int fno = offset++;
    std::string t = "Camera system";
    plotCameraRig(baselineTransforms, fno, true, t);
    figs.push_back(getOrCreateCameraUtilsFigure(fno));
  }

  // =========================================================================
  // Outlier corners
  // =========================================================================
  if (removedOutlierCorners && !removedOutlierCorners->empty()) {
    int fno = offset++;
    std::string t = "Location of removed outlier corners";
    plotOutlierCorners(calibrator, *removedOutlierCorners, fno, true, t);
    figs.push_back(getOrCreateCameraUtilsFigure(fno));
  }

  // =========================================================================
  // Save to PDF using gnuplot backend
  // =========================================================================
  std::println("Report contains {} figures", figs.size());
  for (const auto& fig : figs) {
    fig->backend()->run_command("unset warnings");
  }
  if (showOnScreen) {
    std::println("\nDisplaying figures on screen...");
    sm::plot::PlotCollection::global().show();
  }

  if (!filename.empty() && !figs.empty()) {
    std::println("Saving report to PDF: {}", filename);

    try {
      // Save each figure as individual SVG using gnuplot's svg terminal
      std::vector<std::string> svgFiles;
      std::vector<std::string> pngFiles;
      for (size_t i = 0; i < figs.size(); ++i) {
        std::string figSvg = std::format(".report_fig_{:04d}.svg", i);
        std::string figPng = std::format(".report_fig_{:04d}.png", i);
        svgFiles.push_back(figSvg);
        pngFiles.push_back(figPng);
        figs[i]->size(1280, 768);
        figs[i]->title("Calibration Report - Page " + std::to_string(i + 1));
        figs[i]->save(figSvg);
        figs[i]->backend(nullptr);  // Detach backend to write to disk
        auto document = lunasvg::Document::loadFromFile(figSvg);
        if (document == nullptr) {
          throw std::runtime_error("Failed to save SVG file: " + figSvg);
        }
        auto bitmap = document->renderToBitmap(2560, 1536);
        if (bitmap.isNull()) {
          throw std::runtime_error("Failed to render SVG to bitmap: " + figSvg);
        }
        bitmap.writeToPng(figPng);
        std::println("  Generated page {}/{}", i + 1, figs.size());
      }

      // Combine PNGs using ImageMagick
      std::vector<Magick::Image> images;
      for (const auto& file : pngFiles) {
        Magick::Image image;
        image.read(file);
        images.push_back(std::move(image));
      }

      std::println("Merging PNGs...");
      try {
        Magick::writeImages(images.begin(), images.end(), filename);
        std::println("✓ Report saved successfully to: {}", filename);
        // Clean up temporary files
        for (const auto& file : pngFiles) {
          std::filesystem::remove(file);
        }
        for (const auto& file : svgFiles) {
          std::filesystem::remove(file);
        }
        std::println("  Cleaned up temporary files");
      } catch (const std::exception& e) {
        std::println(stderr, "✗ Error: Failed to merge PDFs: {}", e.what());
        std::println(stderr, "\nIndividual PNG files are available as:");
        for (const auto& file : pngFiles) {
          std::println(stderr, "  {}", file);
        }
      }
    } catch (const std::exception& e) {
      std::println(stderr, "✗ Error saving PNG: {}", e.what());
    }
  } else if (filename.empty()) {
    std::println("No filename specified, skipping PDF export");
  }

  std::println("\nReport generation completed");
  std::println("Total figures created: {}", figs.size());
}

}  // namespace kalibr
