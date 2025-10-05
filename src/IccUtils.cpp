#include <matplot/matplot.h>

#include <Eigen/Core>
#include <IccCalibrator.hpp>
#include <IccPlots.hpp>
#include <IccSensors.hpp>
#include <IccUtils.hpp>
#include <format>
#include <print>
#include <ranges>
#include "sm/plot/PlotCollection.hpp"

namespace kalibr {

void printErrorStatistics(const IccCalibrator& calibrator, std::ostream& dest) {
  std::println(dest, "Normalized Residuals\n----------------------------");

  for (auto [cidx, cam] : std::ranges::views::enumerate(
           calibrator.getCameraChain()->getCamList())) {
    if (cam->getAllReprojectionErrors().empty()) {
      std::println(dest, "Reprojection error (cam{}):     no corners", cidx);
    } else {
      // e2 = np.array([ np.sqrt(rerr.evaluateError()) for reprojectionErrors in
      // cam.allReprojectionErrors for rerr in reprojectionErrors])
      std::vector<double> e2;
      for (const auto& reprojectionErrors : cam->getAllReprojectionErrors()) {
        for (const auto& rerr : reprojectionErrors) {
          e2.push_back(std::sqrt(rerr->evaluateError()));
        }
      }
      double mean = std::accumulate(e2.begin(), e2.end(), 0.0) / e2.size();
      std::sort(e2.begin(), e2.end());
      double median = e2[e2.size() / 2];
      double std =
          std::sqrt(std::accumulate(e2.begin(), e2.end(), 0.0,
                                    [mean](double acc, double v) {
                                      return acc + (v - mean) * (v - mean);
                                    }) /
                    e2.size());
      std::println(dest,
                   "Reprojection error (cam{}):     mean {:.3f}, median "
                   "{:.3f}, std: {:.3f}",
                   cidx, mean, median, std);
    }
  }

  for (auto [iidx, imu] :
       std::ranges::views::enumerate(calibrator.getImuList())) {
    std::vector<double> e2;
    for (const auto& gyroErr : imu->getGyroErrors()) {
      e2.push_back(std::sqrt(gyroErr->evaluateError()));
    }
    double mean = std::accumulate(e2.begin(), e2.end(), 0.0) / e2.size();
    std::sort(e2.begin(), e2.end());
    double median = e2[e2.size() / 2];
    double std =
        std::sqrt(std::accumulate(e2.begin(), e2.end(), 0.0,
                                  [mean](double acc, double v) {
                                    return acc + (v - mean) * (v - mean);
                                  }) /
                  e2.size());
    std::println(dest,
                 "Gyroscope error (imu{}):        mean {:.3f}, median {:.3f}, "
                 "std: {:.3f}",
                 iidx, mean, median, std);
    e2.clear();
    for (const auto& accelErr : imu->getAccelErrors()) {
      e2.push_back(std::sqrt(accelErr->evaluateError()));
    }
    mean = std::accumulate(e2.begin(), e2.end(), 0.0) / e2.size();
    std::sort(e2.begin(), e2.end());
    median = e2[e2.size() / 2];
    std = std::sqrt(std::accumulate(e2.begin(), e2.end(), 0.0,
                                    [mean](double acc, double v) {
                                      return acc + (v - mean) * (v - mean);
                                    }) /
                    e2.size());
    std::println(dest,
                 "Accelerometer error (imu{}):    mean {:.3f}, median {:.3f}, "
                 "std: {:.3f}",
                 iidx, mean, median, std);
  }

  std::println(dest, "\nResiduals\n----------------------------");

  for (auto [cidx, cam] : std::ranges::views::enumerate(
           calibrator.getCameraChain()->getCamList())) {
    if (cam->getAllReprojectionErrors().empty()) {
      std::println(dest, "Reprojection error (cam{}) [px]:     no corners",
                   cidx);
    } else {
      std::vector<double> e2;
      for (const auto& reprojectionErrors : cam->getAllReprojectionErrors()) {
        for (const auto& rerr : reprojectionErrors) {
          e2.push_back(std::sqrt(rerr->vsError().norm()));
        }
      }
      double mean = std::accumulate(e2.begin(), e2.end(), 0.0) / e2.size();
      std::sort(e2.begin(), e2.end());
      double median = e2[e2.size() / 2];
      double std =
          std::sqrt(std::accumulate(e2.begin(), e2.end(), 0.0,
                                    [mean](double acc, double v) {
                                      return acc + (v - mean) * (v - mean);
                                    }) /
                    e2.size());
      std::println(dest,
                   "Reprojection error (cam{}) [px]:     mean {:.3f}, median "
                   "{:.3f}, std: {:.3f}",
                   cidx, mean, median, std);
    }
  }

  for (auto [iidx, imu] :
       std::ranges::views::enumerate(calibrator.getImuList())) {
    std::vector<double> e2;
    for (const auto& gyroErr : imu->getGyroErrors()) {
      e2.push_back(gyroErr->vsError().norm());
    }
    double mean = std::accumulate(e2.begin(), e2.end(), 0.0) / e2.size();
    std::sort(e2.begin(), e2.end());
    double median = e2[e2.size() / 2];
    double std =
        std::sqrt(std::accumulate(e2.begin(), e2.end(), 0.0,
                                  [mean](double acc, double v) {
                                    return acc + (v - mean) * (v - mean);
                                  }) /
                  e2.size());
    std::println(
        dest,
        "Gyroscope error (imu{}) [rad/s]:     mean {:.3f}, median {:.3f}, "
        "std: {:.3f}",
        iidx, mean, median, std);
    e2.clear();
    for (const auto& accelErr : imu->getAccelErrors()) {
      e2.push_back(accelErr->vsError().norm());
    }
    mean = std::accumulate(e2.begin(), e2.end(), 0.0) / e2.size();
    std::sort(e2.begin(), e2.end());
    median = e2[e2.size() / 2];
    std = std::sqrt(std::accumulate(e2.begin(), e2.end(), 0.0,
                                    [mean](double acc, double v) {
                                      return acc + (v - mean) * (v - mean);
                                    }) /
                    e2.size());
    std::println(
        dest,
        "Accelerometer error (imu{}) [m/s^2]: mean {:.3f}, median {:.3f}, "
        "std: {:.3f}",
        iidx, mean, median, std);
  }
}

void printGravity(const IccCalibrator& calibrator) {
  std::cout << std::endl;
  std::cout << "Gravity vector: (in target coordinates): [m/s^2]" << std::endl;

  auto gravityDv = calibrator.getGravityDv();
  if (gravityDv) {
    auto typeIdx = calibrator.getGravityDvType();
    if (typeIdx == typeid(aslam::backend::EuclideanPoint).hash_code()) {
      auto g =
          std::dynamic_pointer_cast<aslam::backend::EuclideanPoint>(gravityDv)
              ->toEuclidean();
      std::cout << g.transpose() << std::endl;
    } else if (typeIdx ==
               typeid(aslam::backend::EuclideanDirection).hash_code()) {
      auto g = std::dynamic_pointer_cast<aslam::backend::EuclideanDirection>(
                   gravityDv)
                   ->toEuclidean();
      std::cout << g.transpose() << std::endl;
    }
  }
}

void printResults(const IccCalibrator& calibrator, bool withCov) {
  // Print camera results
  auto cameraChain = calibrator.getCameraChain();
  size_t nCams = cameraChain->numCameras();

  for (size_t camNr = 0; camNr < nCams; ++camNr) {
    auto T_cam_b = cameraChain->getResultTrafoImuToCam(camNr);

    std::println("\nTransformation T_cam{0}_imu0 (imu0 to cam{0}, T_ci): ",
                 camNr);
    if (withCov && camNr == 0) {
      //             print("    quaternion: ", T_cam_b.q(), " +- ",
      //             cself.std_trafo_ic[0:3])
      //    print("    translation: ", T_cam_b.t(), " +- ",
      //    cself.std_trafo_ic[3:])
      std::println("    quaternion: {} +- {}", T_cam_b.q().transpose(),
                   calibrator.getStdTrafoIc().head(3).transpose());
      std::println("    translation: {} +- {}", T_cam_b.t().transpose(),
                   calibrator.getStdTrafoIc()
                       .tail(calibrator.getStdTrafoIc().size() - 3)
                       .transpose());
    }
    std::cout << T_cam_b.T() << std::endl;
    if (!calibrator.noTimeCalibration()) {
      //             print("")
      //    print("cam{0} to imu0 time: [s] (t_imu = t_cam +
      //    shift)".format(camNr))
      //    print(cself.CameraChain.getResultTimeShift(camNr), end=' ')
      std::println("\ncam{0} to imu0 time: [s] (t_imu = t_cam + shift)", camNr);
      std::println("{}", cameraChain->getResultTimeshift(camNr));
      if (withCov) {
        std::println(" +- {}", calibrator.getStdTimes()[camNr]);
      } else {
        std::println("");
      }
    }
  }
  std::println("");
  for (auto [imuNr, imu] :
       std::ranges::views::enumerate(calibrator.getImuList())) {
    std::println("IMU{0}:\n----------------------------", imuNr);
    imu->getImuConfig().printDetails();
  }
}

void printBaselines(const IccCalibrator& calibrator) {
  auto cameraChain = calibrator.getCameraChain();
  if (!cameraChain) return;

  size_t nCams = cameraChain->numCameras();

  if (nCams > 1) {
    for (size_t camNr = 0; camNr < nCams - 1; ++camNr) {
      auto [T, baseline] = cameraChain->getResultBaseline(camNr, camNr + 1);
      std::println("\nBaseline (cam{0} to cam{1}): [m] ", camNr, camNr + 1);
      std::cout << T.T() << std::endl;
      std::println("{}[m]", baseline);
    }
  }
}

// void generateReport(const IccCalibrator& calibrator,
//                     const std::string& filename, bool showOnScreen) {
//   std::vector<matplot::figure_handle> figs;
//   auto plotter = sm::plot::PlotCollection("Calibration Report");
//   int offset = 3010;

//   // Create text page with results
//   std::stringstream sstream;
//   printResultTxt(calibrator, sstream);

//   // text = [line for line in StringIO(sstream.getvalue())]
//   auto text = std::vector<std::string>();
//   for (std::string line; std::getline(sstream, line);) {
//     text.push_back(line);
//   }

//   size_t linesPerPage = 35;

//   while (true) {
//     auto fig = matplot::figure();

//     double left = 0.05;
//     double bottom = -0.05;
//     double width = 1;
//     double height = 1;
//     double right = left + width;
//     double top = bottom + height;

    

//     size_t startLine = (figs.size() - 1) * linesPerPage;
//     if (startLine >= text.size()) {
//       break;
//     }
//     size_t endLine = std::min(startLine + linesPerPage, text.size());
//     auto pageText = std::vector<std::string>(text.begin() + startLine,
//                                             text.begin() + endLine);

//     // matplot::text(pageText, 0.1, 0.9);
//     matplot::axis(matplot::off);
//     matplot::title("Calibration Report - Page " +
//                    std::to_string(figs.size()));
//     // matplot::draw();
//     offset += 10;
//   }

//   // Plot trajectory
//   plotTrajectory(calibrator, 1003, false, "imu0: estimated poses");

//   // Plot IMU data
//   auto& imuList = calibrator.getImuList();
//   for (size_t iidx = 0; iidx < imuList.size(); ++iidx) {
//     int fig_base = offset + static_cast<int>(iidx) * 10;

//     plotIMURates(calibrator, static_cast<int>(iidx), fig_base + 0);
//     plotGyroError(calibrator, static_cast<int>(iidx), fig_base + 1);
//     plotGyroErrorPerAxis(calibrator, static_cast<int>(iidx), fig_base + 2);
//     plotAccelError(calibrator, static_cast<int>(iidx), fig_base + 3);
//     plotAccelErrorPerAxis(calibrator, static_cast<int>(iidx), fig_base + 4);
//     plotAccelBias(calibrator, static_cast<int>(iidx), fig_base + 5);
//     plotAngularVelocityBias(calibrator, static_cast<int>(iidx), fig_base +
//     6); plotAngularVelocities(calibrator, static_cast<int>(iidx), fig_base +
//     7); plotAccelerations(calibrator, static_cast<int>(iidx), fig_base + 8);
//   }

//   // Plot camera data
//   auto cameraChain = calibrator.getCameraChain();
//   if (cameraChain) {
//     for (size_t cidx = 0; cidx < cameraChain->numCameras(); ++cidx) {
//       int fig_num = offset + 100 + static_cast<int>(cidx);
//       plotReprojectionScatter(calibrator, static_cast<int>(cidx), fig_num);
//     }
//   }

//   // Save to PDF
//   std::cout << "Report saved to: " << filename << std::endl;

//   // matplotplusplus doesn't have direct PDF export
//   // Would need to save individual figures and combine them
// }

void exportPoses(const IccCalibrator& calibrator, const std::string& filename) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    std::println(stderr, "Failed to open file: {}", filename);
    return;
  }

  std::println(file,
               "#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w "
               "[], q_RS_x [], q_RS_y [], q_RS_z []");

  auto imu = calibrator.getImuList().front();
  auto bodyspline = calibrator.getPoseDv()->spline();
  std::vector<double> times;
  for (auto im : imu->getImuData()) {
    auto t = im.stamp.toSec() + imu->getTimeOffset();
    if (t >= bodyspline.t_min() && t <= bodyspline.t_max()) {
      times.push_back(t);
    }
  }

  for (auto time : times) {
    Eigen::Vector3d position = bodyspline.position(time);
    Eigen::Vector4d orientation =
        sm::kinematics::r2quat(bodyspline.orientation(time));
    std::println(file,
                 "{:.0f}, {:.0f}, {:.0f}, {:.0f}, {:.0f}, {:.0f}, "
                 "{:.0f}, {:.0f}",
                 1e9 * time, position.x(), position.y(), position.z(),
                 orientation.w(), orientation.x(), orientation.y(),
                 orientation.z());
  }

  file.close();
}

void saveResultTxt(const IccCalibrator& calibrator,
                   const std::string& filename) {
  std::ofstream file(filename);
  if (!file.is_open()) {
    std::println(stderr, "Failed to open file: {}", filename);
    return;
  }

  printResultTxt(calibrator, file);
  file.close();
}

void printResultTxt(const IccCalibrator& calibrator, std::ostream& stream) {
  std::println(stream, "Calibration results\n===================");
  printErrorStatistics(calibrator, stream);

  auto nCams = calibrator.getCameraChain()->getCamList().size();

  for (size_t camNr = 0; camNr < nCams; ++camNr) {
    auto T = calibrator.getCameraChain()->getResultTrafoImuToCam(camNr);
    std::println(stream, "\nTransformation (cam{0}):", camNr);
    std::println(stream, "-----------------------");
    std::println(stream, "T_ci:  (imu0 to cam{0}): ", camNr);
    stream << T.T() << std::endl;
    std::println(stream, "\nT_ic:  (cam{0} to imu0): ", camNr);
    stream << T.inverse().T() << std::endl;

    std::println(stream,
                 "\ntimeshift cam{0} to imu0: [s] (t_imu = t_cam + shift)", camNr);
    std::println(stream, "{}\n", calibrator.getCameraChain()->getResultTimeshift(camNr));
  }

  if (nCams > 1) {
    std::println(stream, "Baselines:");
    std::println(stream, "----------");
    for (size_t camNr = 0; camNr < nCams - 1; ++camNr) {
      auto [T, baseline] = calibrator.getCameraChain()->getResultBaseline(camNr, camNr + 1);
      std::println(stream, "Baseline (cam{0} to cam{1}): ", camNr, camNr + 1);
      stream << T.T() << std::endl;
      std::println(stream, "baseline norm: {}[m]\n", baseline);
    }
  }

  if (calibrator.getGravityDvType() == typeid(aslam::backend::EuclideanPoint).hash_code()) {
    auto g = std::dynamic_pointer_cast<aslam::backend::EuclideanPoint>(
                 calibrator.getGravityDv())
                 ->toEuclidean();
    std::println(stream, "\nGravity vector in target coords: [m/s^2]");
    std::println(stream, "{}", g.transpose());
  } else if (calibrator.getGravityDvType() ==
             typeid(aslam::backend::EuclideanDirection).hash_code()) {
    auto g = std::dynamic_pointer_cast<aslam::backend::EuclideanDirection>(
                 calibrator.getGravityDv())
                 ->toEuclidean();
    std::println(stream, "\nGravity vector in target coords: [m/s^2]");
    std::println(stream, "{}", g.transpose());
  }

  std::println(stream, "\n\nCalibration configuration");
  std::println(stream, "=========================\n");

  for (auto [camNr, cam] :
       std::ranges::views::enumerate(calibrator.getCameraChain()->getCamList())) {
    std::println(stream, "cam{0}", camNr);
    std::println(stream, "-----");
    cam->getCamConfig().printDetails(stream);
    cam->getTargetConfig().printDetails(stream);
    std::println(stream, "");
  }

  std::println(stream, "\n\nIMU configuration");
  std::println(stream, "=================\n");

  for (auto [imuNr, imu] :
       std::ranges::views::enumerate(calibrator.getImuList())) {
    std::println(stream, "IMU{0}:", imuNr);
    std::println(stream, "----------------------------");
    imu->getImuConfig().printDetails(stream);
    std::println(stream, "");
  }
}

}  // namespace kalibr
