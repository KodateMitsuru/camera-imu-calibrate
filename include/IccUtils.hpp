#ifndef ICC_UTILS_HPP
#define ICC_UTILS_HPP

#include <Eigen/Core>
#include <iostream>
#include <string>
#include <vector>

namespace kalibr {

class IccCalibrator;

/**
 * @brief Plot trajectory in 3D
 */
void plotTrajectory(const IccCalibrator& calibrator, int figureNumber = 1,
                    bool clearFigure = true, const std::string& title = "");

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
