#include "sm/kinematics/quaternion_algebra.hpp"
#include <matplot/matplot.h>

#include <Eigen/Core>
#include <IccCalibrator.hpp>
#include <IccPlots.hpp>
#include <IccSensors.hpp>
#include <IccUtils.hpp>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <tuple>
#include <vector>

namespace kalibr {

// ============================================================================
// PlotManager 类实现
// ============================================================================

PlotManager::PlotManager(int rows, int cols) { configureGrid(rows, cols); }

void PlotManager::configureGrid(int rows, int cols) {
  grid_config_.rows = rows;
  grid_config_.cols = cols;
  grid_config_.max_subplots = rows * cols;
}

void PlotManager::clearAllSubplots() { used_subplots_.clear(); }

std::tuple<int, int, int> PlotManager::getGridConfig() const {
  return {grid_config_.rows, grid_config_.cols, grid_config_.max_subplots};
}

size_t PlotManager::getUsedSubplotCount() const {
  return used_subplots_.size();
}

void PlotManager::activateSubplot(int figureNumber) {
  // 将 figureNumber 映射到 subplot 索引 (1-based)
  int subplot_idx = (figureNumber % grid_config_.max_subplots) + 1;

  // 标记为已使用
  used_subplots_[subplot_idx] = true;

  // 激活对应的 subplot
  matplot::subplot(grid_config_.rows, grid_config_.cols, subplot_idx);
}

void PlotManager::plotCoordinateFrame(const Eigen::Vector3d& position,
                                      const Eigen::Matrix3d& orientation,
                                      double size) {
  // X 轴（红色）
  Eigen::Vector3d x_end = position + orientation.col(0) * size;
  std::vector<double> x_axis_x = {position.x(), x_end.x()};
  std::vector<double> x_axis_y = {position.y(), x_end.y()};
  std::vector<double> x_axis_z = {position.z(), x_end.z()};
  auto l1 = matplot::plot3(x_axis_x, x_axis_y, x_axis_z, "r-");
  l1->line_width(2);

  // Y 轴（绿色）
  Eigen::Vector3d y_end = position + orientation.col(1) * size;
  std::vector<double> y_axis_x = {position.x(), y_end.x()};
  std::vector<double> y_axis_y = {position.y(), y_end.y()};
  std::vector<double> y_axis_z = {position.z(), y_end.z()};
  auto l2 = matplot::plot3(y_axis_x, y_axis_y, y_axis_z, "g-");
  l2->line_width(2);

  // Z 轴（蓝色）
  Eigen::Vector3d z_end = position + orientation.col(2) * size;
  std::vector<double> z_axis_x = {position.x(), z_end.x()};
  std::vector<double> z_axis_y = {position.y(), z_end.y()};
  std::vector<double> z_axis_z = {position.z(), z_end.z()};
  auto l3 = matplot::plot3(z_axis_x, z_axis_y, z_axis_z, "b-");
  l3->line_width(2);
}

void PlotManager::plotTrajectoryLine(const Eigen::Vector3d& from,
                                     const Eigen::Vector3d& to) {
  std::vector<double> line_x = {from.x(), to.x()};
  std::vector<double> line_y = {from.y(), to.y()};
  std::vector<double> line_z = {from.z(), to.z()};
  auto line = matplot::plot3(line_x, line_y, line_z, "k-");
  line->line_width(1);
}

void PlotManager::plotTrajectory(const IccCalibrator& calibrator,
                                 int figureNumber, bool clearFigure,
                                 const std::string& title) {
  // 激活对应的 subplot
  activateSubplot(figureNumber);

  if (clearFigure) {
    matplot::cla();  // 清除当前坐标轴
  }

  if (!title.empty()) {
    matplot::title(title);
  }

  constexpr double size = 0.05;  // 坐标系框架的大小

  // 获取 IMU 和样条数据
  auto& imuList = calibrator.getImuList();
  if (imuList.empty()) {
    std::cerr << "PlotManager: No IMUs available" << std::endl;
    return;
  }

  auto& imu = imuList[0];
  auto poseDv = calibrator.getPoseDv();
  if (!poseDv) {
    std::cerr << "PlotManager: No pose design variable available" << std::endl;
    return;
  }

  auto bodyspline = poseDv->spline();

  // 获取有效时间范围内的 IMU 时间戳
  std::vector<double> times_imu;
  for (const auto& im : imu->getImuData()) {
    double t = im.stamp.toSec() + imu->getTimeOffset();
    if (t > bodyspline.t_min() && t < bodyspline.t_max()) {
      times_imu.push_back(t);
    }
  }

  if (times_imu.empty()) {
    std::cerr << "PlotManager: No valid timestamps in spline range"
              << std::endl;
    return;
  }

  double t_min = *std::min_element(times_imu.begin(), times_imu.end());
  double t_max = *std::max_element(times_imu.begin(), times_imu.end());

  // Sample at 10 Hz (每 0.1 秒一个采样点)
  std::vector<double> times;
  for (double t = t_min; t <= t_max; t += 0.1) {
    times.push_back(t);
  }

  // 用于记录边界
  Eigen::Vector3d traj_min = Eigen::Vector3d::Constant(9999.0);
  Eigen::Vector3d traj_max = Eigen::Vector3d::Constant(-9999.0);

  Eigen::Vector3d last_position;
  bool has_last = false;

  // Plot each pose as coordinate frame + trajectory line
  for (double t : times) {
    try {
      // 评估样条在时刻 t 的位置和姿态
      Eigen::Vector3d position = bodyspline.position(t);
      auto orientation = sm::kinematics::r2quat(bodyspline.orientation(t));
      auto T = sm::kinematics::Transformation(orientation, position);
      PlotManager::plotCoordinateFrame(position, orientation, size);

      // 更新边界
      traj_min = traj_min.cwiseMin(position);
      traj_max = traj_max.cwiseMax(position);

      // 绘制坐标系框架
      if (has_last) {
        matplot::hold(matplot::on);
      }
      plotCoordinateFrame(position, orientation, size);
      matplot::hold(matplot::on);

      // 绘制从上一个位置到当前位置的连接线
      if (has_last) {
        plotTrajectoryLine(last_position, position);
      }

      last_position = position;
      has_last = true;

    } catch (const std::exception& e) {
      std::cerr << "PlotManager: Error evaluating spline at t=" << t << ": "
                << e.what() << std::endl;
      continue;
    }
  }

  matplot::hold(matplot::off);

  // 设置坐标轴标签
  matplot::xlabel("X (m)");
  matplot::ylabel("Y (m)");
  matplot::zlabel("Z (m)");
  matplot::grid(matplot::on);

  // 设置坐标轴范围（在轨迹边界基础上增加一些边距）
  if (has_last) {
    matplot::xlim({traj_min.x() - size, traj_max.x() + size});
    matplot::ylim({traj_min.y() - size, traj_max.y() + size});
    matplot::zlim({traj_min.z() - size, traj_max.z() + size});
  }
}

void PlotManager::show() { matplot::show(); }

// ============================================================================
// 兼容旧接口的全局函数实现（使用单例 PlotManager）
// ============================================================================

namespace {
// 全局单例 PlotManager
PlotManager& getGlobalPlotManager() {
  static PlotManager instance;
  return instance;
}
}  // anonymous namespace

void plotTrajectory(const IccCalibrator& calibrator, int figureNumber,
                    bool clearFigure, const std::string& title) {
  getGlobalPlotManager().plotTrajectory(calibrator, figureNumber, clearFigure,
                                        title);
  getGlobalPlotManager().show();
}

void configureSubplotGrid(int rows, int cols) {
  getGlobalPlotManager().configureGrid(rows, cols);
}

void clearAllSubplots() { getGlobalPlotManager().clearAllSubplots(); }

// ============================================================================
// 其他工具函数
// ============================================================================

// void printErrorStatistics(const IccCalibrator& calibrator, std::ostream&
// dest) {
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
//   std::cout << "Gravity vector: (in target coordinates): [m/s^2]" <<
//   std::endl;

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

//     std::cout << "Time offset: " << imu->getTimeOffset() << " s" <<
//     std::endl;

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

// void exportPoses(const IccCalibrator& calibrator, const std::string&
// filename) {
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
//   stream <<
//   "=============================================================\n"; stream
//   << "                  Calibration Results\n"; stream <<
//   "=============================================================\n\n";

//   printResults(calibrator, false);
//   stream << std::endl;

//   printErrorStatistics(calibrator, stream);
//   stream << std::endl;

//   printGravity(calibrator);
// }

}  // namespace kalibr
