// #include <matplot/matplot.h>

// #include <Eigen/Core>
// #include <IccCalibrator.hpp>
// #include <IccPlots.hpp>
// #include <IccSensors.hpp>
// #include <IccUtils.hpp>
// #include <cmath>
// #include <fstream>
// #include <iomanip>
// #include <iostream>
// #include <sstream>
// #include <vector>

// namespace kalibr {

// void plotTrajectory(const IccCalibrator& calibrator, int figureNumber,
//                     bool clearFigure, const std::string& title) {
//   auto fig = matplot::figure(figureNumber);
//   if (clearFigure) {
//     fig->clear();
//   }

//   if (!title.empty()) {
//     matplot::title(title);
//   }

//   double size = 0.05;

//   // Get times to evaluate
//   auto& imuList = calibrator.getImuList();
//   if (imuList.empty()) {
//     std::cerr << "No IMUs available" << std::endl;
//     return;
//   }

//   auto& imu = imuList[0];
//   auto bodyspline = calibrator.getPoseDv()->spline();

//   std::vector<double> timestamps;
//   for (const auto& im : imu->getImuData()) {
//     double t = im.timestamp.toSec() + imu->getTimeOffset();
//     if (t > bodyspline.t_min() && t < bodyspline.t_max()) {
//       timestamps.push_back(t);
//     }
//   }

//   if (timestamps.empty()) return;

//   double t_min = *std::min_element(timestamps.begin(), timestamps.end());
//   double t_max = *std::max_element(timestamps.begin(), timestamps.end());

//   // Sample at 10 Hz
//   std::vector<double> times;
//   for (double t = t_min; t <= t_max; t += 0.1) {
//     times.push_back(t);
//   }

//   // Extract trajectory
//   std::vector<double> x_vals, y_vals, z_vals;
//   Eigen::Vector3d traj_min(9999.0, 9999.0, 9999.0);
//   Eigen::Vector3d traj_max(-9999.0, -9999.0, -9999.0);

//   for (double t : times) {
//     // Evaluate spline at time t
//     // Eigen::Matrix4d T = bodyspline.transformation(t);
//     // Eigen::Vector3d pos = T.block<3,1>(0,3);

//     // For now, placeholder
//     Eigen::Vector3d pos(0, 0, 0);

//     x_vals.push_back(pos.x());
//     y_vals.push_back(pos.y());
//     z_vals.push_back(pos.z());

//     traj_min = traj_min.cwiseMin(pos);
//     traj_max = traj_max.cwiseMax(pos);
//   }

//   // Create 3D plot
//   auto ax = matplot::gca();
//   matplot::plot3(x_vals, y_vals, z_vals);
//   matplot::xlabel("X (m)");
//   matplot::ylabel("Y (m)");
//   matplot::zlabel("Z (m)");
//   matplot::grid(matplot::on);

//   matplot::show();
// }

// void printErrorStatistics(const IccCalibrator& calibrator, std::ostream& dest) {
//   dest << "Normalized Residuals\n----------------------------" << std::endl;

//   // Camera reprojection errors
//   auto cameraChain = calibrator.getCameraChain();
//   if (cameraChain) {
//     for (size_t cidx = 0; cidx < cameraChain->numCameras(); ++cidx) {
//       // Calculate and print normalized residuals
//       dest << "  cam" << cidx << ": " << std::endl;
//       // Extract errors and compute statistics
//     }
//   }

//   // IMU errors
//   auto& imuList = calibrator.getImuList();
//   for (size_t iidx = 0; iidx < imuList.size(); ++iidx) {
//     dest << "  imu" << iidx << ":" << std::endl;
//     // Gyro errors
//     dest << "    gyro: " << std::endl;
//     // Accel errors
//     dest << "    accel: " << std::endl;
//   }

//   dest << std::endl;
//   dest << "Residuals\n----------------------------" << std::endl;

//   // Similar structure for non-normalized residuals
// }

// void printGravity(const IccCalibrator& calibrator) {
//   std::cout << std::endl;
//   std::cout << "Gravity vector: (in target coordinates): [m/s^2]" << std::endl;

//   auto gravityDv = calibrator.getGravityDv();
//   if (gravityDv) {
//     Eigen::Vector3d g = gravityDv->toEuclidean();
//     std::cout << g.transpose() << std::endl;
//     std::cout << "Magnitude: " << g.norm() << " m/s^2" << std::endl;
//   }
// }

// void printResults(const IccCalibrator& calibrator, bool withCov) {
//   std::cout
//       << "\n=============================================================\n";
//   std::cout << "                  Calibration Results\n";
//   std::cout
//       << "=============================================================\n\n";

//   // Print camera results
//   auto cameraChain = calibrator.getCameraChain();
//   if (cameraChain) {
//     size_t nCams = cameraChain->numCameras();

//     for (size_t camNr = 0; camNr < nCams; ++camNr) {
//       std::cout << "Camera " << camNr << " Results:\n";
//       std::cout << "----------------------------\n";

//       // Print intrinsics
//       // Print extrinsics relative to IMU
//       // Print time offset

//       if (withCov) {
//         // Print covariances
//       }

//       std::cout << std::endl;
//     }
//   }

//   // Print IMU results
//   auto& imuList = calibrator.getImuList();
//   for (size_t imuNr = 0; imuNr < imuList.size(); ++imuNr) {
//     std::cout << "IMU " << imuNr << " Results:\n";
//     std::cout << "----------------------------\n";

//     auto& imu = imuList[imuNr];
//     auto T_dv = imu->getTransformationDv();

//     if (T_dv) {
//       // Print transformation
//       std::cout << "T_imu_body:" << std::endl;
//       // Print matrix
//     }

//     std::cout << "Time offset: " << imu->getTimeOffset() << " s" << std::endl;

//     if (withCov) {
//       // Print covariances
//     }

//     std::cout << std::endl;
//   }

//   // Print gravity
//   printGravity(calibrator);
// }

// void printBaselines(const IccCalibrator& calibrator) {
//   auto cameraChain = calibrator.getCameraChain();
//   if (!cameraChain) return;

//   size_t nCams = cameraChain->numCameras();

//   if (nCams > 1) {
//     std::cout << "\nCamera Chain Baselines:\n";
//     std::cout << "----------------------------\n";

//     for (size_t i = 1; i < nCams; ++i) {
//       std::cout << "Baseline cam" << (i - 1) << " to cam" << i << ":"
//                 << std::endl;
//       // Print transformation
//     }
//   }
// }

// void generateReport(const IccCalibrator& calibrator,
//                     const std::string& filename, bool showOnScreen) {
//   std::cout << "Generating calibration report..." << std::endl;

//   std::vector<matplot::figure_handle> figs;
//   int offset = 3010;

//   // Create text page with results
//   std::ostringstream sstream;
//   printResultTxt(calibrator, sstream);

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
//     plotAngularVelocityBias(calibrator, static_cast<int>(iidx), fig_base + 6);
//     plotAngularVelocities(calibrator, static_cast<int>(iidx), fig_base + 7);
//     plotAccelerations(calibrator, static_cast<int>(iidx), fig_base + 8);
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

// void exportPoses(const IccCalibrator& calibrator, const std::string& filename) {
//   std::ofstream file(filename);
//   if (!file.is_open()) {
//     std::cerr << "Failed to open file: " << filename << std::endl;
//     return;
//   }

//   file << "timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z" << std::endl;

//   auto poseDv = calibrator.getPoseDv();
//   if (!poseDv) return;

//   auto& bodyspline = poseDv->spline();

//   // Sample at fixed frequency
//   double t_min = bodyspline.t_min();
//   double t_max = bodyspline.t_max();
//   double dt = 0.01;  // 100 Hz

//   for (double t = t_min; t <= t_max; t += dt) {
//     // Evaluate spline
//     // Eigen::Matrix4d T = bodyspline.transformation(t);

//     // Extract position and quaternion
//     // Eigen::Vector3d pos = T.block<3,1>(0,3);
//     // Eigen::Quaterniond q(T.block<3,3>(0,0));

//     // Placeholder
//     Eigen::Vector3d pos(0, 0, 0);
//     Eigen::Quaterniond q(1, 0, 0, 0);

//     file << std::fixed << std::setprecision(9) << t << "," << pos.x() << ","
//          << pos.y() << "," << pos.z() << "," << q.w() << "," << q.x() << ","
//          << q.y() << "," << q.z() << std::endl;
//   }

//   file.close();
//   std::cout << "Poses exported to: " << filename << std::endl;
// }

// void saveResultTxt(const IccCalibrator& calibrator,
//                    const std::string& filename) {
//   std::ofstream file(filename);
//   if (!file.is_open()) {
//     std::cerr << "Failed to open file: " << filename << std::endl;
//     return;
//   }

//   printResultTxt(calibrator, file);
//   file.close();

//   std::cout << "Results saved to: " << filename << std::endl;
// }

// void printResultTxt(const IccCalibrator& calibrator, std::ostream& stream) {
//   stream << "=============================================================\n";
//   stream << "                  Calibration Results\n";
//   stream << "=============================================================\n\n";

//   printResults(calibrator, false);
//   stream << std::endl;

//   printErrorStatistics(calibrator, stream);
//   stream << std::endl;

//   printGravity(calibrator);
// }

// }  // namespace kalibr
