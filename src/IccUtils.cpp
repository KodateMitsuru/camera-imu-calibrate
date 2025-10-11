#include <matplot/matplot.h>

#include <Eigen/Core>
#include <IccCalibrator.hpp>
#include <IccPlots.hpp>
#include <IccSensors.hpp>
#include <IccUtils.hpp>
#include <filesystem>
#include <format>
#include <print>
#include <ranges>

namespace kalibr {

void plotTrajectory(const IccCalibrator& calibrator, int figureNumber,
                    bool clearFigure, const std::string& title) {
  // Get pose spline from calibrator
  auto poseDv = calibrator.getPoseDv();
  if (!poseDv) {
    std::println(stderr, "No pose design variable available");
    return;
  }

  auto& bodyspline = poseDv->spline();

  // Get IMU data to extract timestamps
  auto& imuList = calibrator.getImuList();
  if (imuList.empty()) {
    std::println(stderr, "No IMU data available");
    return;
  }

  auto& imu = imuList[0];  // Use first IMU

  // Collect timestamps within spline time range
  std::vector<double> timestamps;
  for (const auto& im : imu->getImuData()) {
    double t = im.stamp.toSec() + imu->getTimeOffset();
    if (t >= bodyspline.t_min() && t <= bodyspline.t_max()) {
      timestamps.push_back(t);
    }
  }

  if (timestamps.empty()) {
    std::println(stderr, "No valid timestamps for trajectory plotting");
    return;
  }

  // Evaluate positions along the trajectory
  std::vector<double> x_coords, y_coords, z_coords;
  for (double t : timestamps) {
    Eigen::Vector3d position = bodyspline.position(t);
    x_coords.push_back(position.x());
    y_coords.push_back(position.y());
    z_coords.push_back(position.z());
  }

  // Get or create figure
  auto fig = getOrCreateFigure(figureNumber);
  auto ax = fig->current_axes();

  if (clearFigure) {
    ax->clear();
  }

  // Plot the 3D trajectory
  auto line = matplot::plot3(x_coords, y_coords, z_coords);
  line->color("b");
  line->line_width(2.0);

  // Add start and end markers
  if (!x_coords.empty()) {
    matplot::hold(matplot::on);

    // Start point (green)
    auto start = matplot::scatter3(std::vector<double>{x_coords.front()},
                                   std::vector<double>{y_coords.front()},
                                   std::vector<double>{z_coords.front()});
    start->marker_color("g");
    start->marker_size(10);
    start->display_name("Start");

    // End point (red)
    auto end = matplot::scatter3(std::vector<double>{x_coords.back()},
                                 std::vector<double>{y_coords.back()},
                                 std::vector<double>{z_coords.back()});
    end->marker_color("r");
    end->marker_size(10);
    end->display_name("End");

    matplot::hold(matplot::off);
  }

  // Set labels and title
  matplot::xlabel("x [m]");
  matplot::ylabel("y [m]");
  matplot::zlabel("z [m]");

  if (!title.empty()) {
    matplot::title(title);
  } else {
    matplot::title("Estimated IMU Trajectory");
  }

  // Enable grid
  matplot::grid(matplot::on);

  // Set equal aspect ratio for better visualization
  matplot::axis(matplot::equal);

  // Add legend
  matplot::legend();
}

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
          e2.push_back(rerr->vsError().norm());
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

void generateReport(const IccCalibrator& calibrator,
                    const std::string& filename, bool showOnScreen) {
  std::vector<std::shared_ptr<matplot::figure_type>> figs;
  int offset = 3010;

  std::println("Generating calibration report...");
  std::println("Target filename: {} (PDF export not yet implemented)",
               filename);

  // =========================================================================
  // Create text pages with results
  // =========================================================================
  std::stringstream sstream;
  printResultTxt(calibrator, sstream);

  // Split text into lines
  std::vector<std::string> text;
  for (std::string line; std::getline(sstream, line);) {
    text.push_back(line);
  }

  size_t linesPerPage = 35;
  size_t textIdx = 0;

  // Create text pages
  while (textIdx < text.size()) {
    auto fig = getOrCreateFigure(offset);
    offset += 1;

    // Get current axes
    auto ax = fig->current_axes();
    ax->clear();
    ax->position({0.0, 1.0, 0.0, 1.0});



    // Turn off axis
    matplot::axis(matplot::off);

    // Prepare text for this page
    size_t endIdx = std::min(textIdx + linesPerPage, text.size());
    std::string pageText;
    for (size_t i = textIdx; i < endIdx; ++i) {
      pageText += text[i];
      pageText += "\\n";
    }
    auto t = matplot::text(ax, 0, 0, pageText);
    t->font_size(7);
    matplot::title("Calibration Report - Page " +
                   std::to_string(figs.size() + 1));

    figs.push_back(fig);
    textIdx = endIdx;
  }

  // =========================================================================
  // Plot trajectory
  // =========================================================================
  auto trajFig = getOrCreateFigure(1003);
  plotTrajectory(calibrator, 1003, true, "imu0: estimated poses");
  figs.push_back(trajFig);

  // =========================================================================
  // Plot IMU data
  // =========================================================================
  auto& imuList = calibrator.getImuList();
  for (size_t iidx = 0; iidx < imuList.size(); ++iidx) {
    int idx = static_cast<int>(iidx);

    // IMU rates
    auto fig1 = getOrCreateFigure(offset + idx);
    plotIMURates(calibrator, idx, offset + idx, true, true);
    figs.push_back(fig1);
    offset += static_cast<int>(imuList.size());

    // Accelerations
    auto fig2 = getOrCreateFigure(offset + idx);
    plotAccelerations(calibrator, idx, offset + idx, true, true);
    figs.push_back(fig2);
    offset += static_cast<int>(imuList.size());

    // Acceleration error per axis
    auto fig3 = getOrCreateFigure(offset + idx);
    plotAccelErrorPerAxis(calibrator, idx, offset + idx, true, true);
    figs.push_back(fig3);
    offset += static_cast<int>(imuList.size());

    // Accelerometer bias
    auto fig4 = getOrCreateFigure(offset + idx);
    plotAccelBias(calibrator, idx, offset + idx, true, true);
    figs.push_back(fig4);
    offset += static_cast<int>(imuList.size());

    // Angular velocities
    auto fig5 = getOrCreateFigure(offset + idx);
    plotAngularVelocities(calibrator, idx, offset + idx, true, true);
    figs.push_back(fig5);
    offset += static_cast<int>(imuList.size());

    // Gyroscope error per axis
    auto fig6 = getOrCreateFigure(offset + idx);
    plotGyroErrorPerAxis(calibrator, idx, offset + idx, true, true);
    figs.push_back(fig6);
    offset += static_cast<int>(imuList.size());

    // Gyroscope bias
    auto fig7 = getOrCreateFigure(offset + idx);
    plotAngularVelocityBias(calibrator, idx, offset + idx, true, true);
    figs.push_back(fig7);
    offset += static_cast<int>(imuList.size());
  }

  // =========================================================================
  // Plot camera data
  // =========================================================================
  auto cameraChain = calibrator.getCameraChain();
  if (cameraChain) {
    auto& camList = cameraChain->getCamList();
    for (size_t cidx = 0; cidx < camList.size(); ++cidx) {
      int idx = static_cast<int>(cidx);
      int fig_num = offset + idx;

      auto fig = getOrCreateFigure(fig_num);
      std::string title =
          "cam" + std::to_string(cidx) + ": reprojection errors";
      plotReprojectionScatter(calibrator, idx, fig_num, true, true, title);
      figs.push_back(fig);

      offset += static_cast<int>(camList.size());
    }
  }

  // =========================================================================
  // Save to PDF using gnuplot backend
  // =========================================================================
  std::println("Report contains {} figures", figs.size());

  if (!filename.empty() && !figs.empty()) {
    std::println("Saving report to PDF: {}", filename);

    try {
      // Save each figure as individual PDF using gnuplot's pdfcairo terminal
      std::vector<std::string> pdfFiles;

      for (size_t i = 0; i < figs.size(); ++i) {
        std::string figPdf = std::format(".report_fig_{:04d}.pdf", i);
        pdfFiles.push_back(figPdf);

        // Configure the figure to use pdfcairo terminal and save
        // Matplot++ will use gnuplot backend to generate PDF
        // Suppress gnuplot multiplot warning by unsetting warnings in the
        // backend before saving. See:
        // https://github.com/alandefreitas/matplotplusplus/issues/385 and
        // https://github.com/alandefreitas/matplotplusplus/issues/432
        try {
          if (figs[i] && figs[i]->backend()) {
            // backend()->run_command may not be available on all backends;
            // guard with try/catch to avoid crashing the report generation.
            try {
              figs[i]->backend()->run_command("unset warnings");
            } catch (...) {
              // ignore; best-effort
            }
          }
        } catch (...) {
          // ignore any error while attempting to access backend
        }

        figs[i]->save(figPdf);
        figs[i]->draw();
        std::println("  Generated page {}/{}", i + 1, figs.size());
      }

      // Combine PDFs using pdfunite (from poppler-utils)
      std::string fileList;
      for (const auto& file : pdfFiles) {
        fileList += file + " ";
      }
      // Show figures on screen if requested
      if (showOnScreen) {
        std::println("\nDisplaying figures on screen...");
        getFigureRegistry().show();
      }

      std::string command = std::format("pdfunite {} {}", fileList, filename);
      std::println("Merging PDFs...");
      int ret = std::system(command.c_str());

      if (ret == 0) {
        std::println("✓ Report saved successfully to: {}", filename);

        // Clean up temporary files
        for (const auto& file : pdfFiles) {
          std::filesystem::remove(file);
        }
        std::println("  Cleaned up temporary files");
      } else {
        std::println(stderr,
                     "✗ Error: Failed to merge PDFs. "
                     "Make sure 'pdfunite' is installed:");
        std::println(stderr,
                     "  Ubuntu/Debian: sudo apt-get install poppler-utils");
        std::println(stderr, "  macOS: brew install poppler");
        std::println(stderr, "\nIndividual PDF files are available as:");
        for (const auto& file : pdfFiles) {
          std::println(stderr, "  {}", file);
        }
      }
    } catch (const std::exception& e) {
      std::println(stderr, "✗ Error saving PDF: {}", e.what());
    }
  } else if (filename.empty()) {
    std::println("No filename specified, skipping PDF export");
  }

  std::println("\nReport generation completed");
  std::println("Total figures created: {}", figs.size());
}

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
                 "\ntimeshift cam{0} to imu0: [s] (t_imu = t_cam + shift)",
                 camNr);
    std::println(stream, "{}\n",
                 calibrator.getCameraChain()->getResultTimeshift(camNr));
  }

  if (nCams > 1) {
    std::println(stream, "Baselines:");
    std::println(stream, "----------");
    for (size_t camNr = 0; camNr < nCams - 1; ++camNr) {
      auto [T, baseline] =
          calibrator.getCameraChain()->getResultBaseline(camNr, camNr + 1);
      std::println(stream, "Baseline (cam{0} to cam{1}): ", camNr, camNr + 1);
      stream << T.T() << std::endl;
      std::println(stream, "baseline norm: {}[m]\n", baseline);
    }
  }

  if (calibrator.getGravityDvType() ==
      typeid(aslam::backend::EuclideanPoint).hash_code()) {
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

  for (auto [camNr, cam] : std::ranges::views::enumerate(
           calibrator.getCameraChain()->getCamList())) {
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
