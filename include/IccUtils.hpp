#ifndef ICC_UTILS_HPP
#define ICC_UTILS_HPP

#include <Eigen/Core>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "IccCalibrator.hpp"

namespace kalibr {

/**
 * @brief 面向对象的绘图管理器
 *
 * 负责管理 subplot 网格和绘图操作。
 * 使用示例：
 * @code
 *   PlotManager plotter(3, 3);  // 创建 3x3 网格
 *   plotter.plotTrajectory(calibrator, 1, true, "Trajectory");
 *   plotter.show();
 * @endcode
 */
class PlotManager {
 public:
  /**
   * @brief 构造函数
   * @param rows Subplot 网格的行数（默认 5）
   * @param cols Subplot 网格的列数（默认 5）
   */
  explicit PlotManager(int rows = 5, int cols = 5);

  /**
   * @brief 析构函数
   */
  ~PlotManager() = default;

  // 禁止拷贝，允许移动
  PlotManager(const PlotManager&) = delete;
  PlotManager& operator=(const PlotManager&) = delete;
  PlotManager(PlotManager&&) = default;
  PlotManager& operator=(PlotManager&&) = default;

  /**
   * @brief 配置 subplot 网格布局
   * @param rows 行数
   * @param cols 列数
   */
  void configureGrid(int rows, int cols);

  /**
   * @brief 清除所有 subplot 跟踪状态
   */
  void clearAllSubplots();

  /**
   * @brief 绘制 3D 轨迹
   * @param calibrator 标定器实例
   * @param figureNumber 子图编号（自动映射到网格位置）
   * @param clearFigure 是否清除子图内容
   * @param title 图形标题
   */
  void plotTrajectory(const IccCalibrator& calibrator, int figureNumber = 1,
                      bool clearFigure = true, const std::string& title = "");

  /**
   * @brief 显示所有绘图窗口
   */
  void show();

  /**
   * @brief 获取当前网格配置
   * @return {rows, cols, max_subplots}
   */
  std::tuple<int, int, int> getGridConfig() const;

  /**
   * @brief 获取已使用的 subplot 数量
   */
  size_t getUsedSubplotCount() const;

 private:
  /**
   * @brief 激活指定编号的 subplot
   * @param figureNumber 子图编号（1-based）
   */
  void activateSubplot(int figureNumber);

  /**
   * @brief 绘制坐标系框架
   * @param position 原点位置
   * @param orientation 方向矩阵
   * @param size 坐标轴长度
   */
  void plotCoordinateFrame(const Eigen::Vector3d& position,
                           const Eigen::Matrix3d& orientation, double size);

  /**
   * @brief 绘制轨迹线段
   * @param from 起点
   * @param to 终点
   */
  void plotTrajectoryLine(const Eigen::Vector3d& from,
                          const Eigen::Vector3d& to);

  // Subplot 配置
  struct GridConfig {
    int rows = 5;
    int cols = 5;
    int max_subplots = 25;
  };

  GridConfig grid_config_;
  std::map<int, bool> used_subplots_;
};

// ============================================================================
// 兼容旧接口的自由函数（内部使用全局 PlotManager 实例）
// ============================================================================

/**
 * @brief Plot trajectory in 3D (legacy interface)
 * @param calibrator The calibrator instance
 * @param figureNumber Figure number (mapped to subplot position)
 * @param clearFigure Whether to clear the subplot before plotting
 * @param title Plot title
 */
void plotTrajectory(const IccCalibrator& calibrator, int figureNumber = 1,
                    bool clearFigure = true, const std::string& title = "");

/**
 * @brief Configure subplot grid layout (legacy interface)
 * @param rows Number of rows in the subplot grid
 * @param cols Number of columns in the subplot grid
 *
 * Call this before creating plots to set the layout.
 * Default is 5x5 grid (25 subplots).
 */
void configureSubplotGrid(int rows, int cols);

/**
 * @brief Clear all subplots (legacy interface)
 *
 * Clears the tracking of used subplots.
 * Note: This doesn't clear the actual plot window in matplotplusplus.
 */
void clearAllSubplots();

/**
 * @brief Print error statistics
 */
void printErrorStatistics(const IccCalibrator& calibrator,
                          std::ostream& dest = std::cout);

/**
 * @brief Print gravity vector
 */
void printGravity(const IccCalibrator& calibrator);

/**
 * @brief Print calibration results
 */
void printResults(const IccCalibrator& calibrator, bool withCov = false);

/**
 * @brief Print baselines between cameras
 */
void printBaselines(const IccCalibrator& calibrator);

/**
 * @brief Generate PDF report with plots and results
 */
void generateReport(const IccCalibrator& calibrator,
                    const std::string& filename = "report.pdf",
                    bool showOnScreen = true);

/**
 * @brief Export poses to CSV file
 */
void exportPoses(const IccCalibrator& calibrator,
                 const std::string& filename = "poses_imu0.csv");

/**
 * @brief Save results to text file
 */
void saveResultTxt(const IccCalibrator& calibrator,
                   const std::string& filename = "cam_imu_result.txt");

/**
 * @brief Print results to stream
 */
void printResultTxt(const IccCalibrator& calibrator,
                    std::ostream& stream = std::cout);

}  // namespace kalibr

#endif  // ICC_UTILS_HPP
