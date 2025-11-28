#ifndef ICC_UTILS_HPP
#define ICC_UTILS_HPP

#include <Eigen/Core>
#include <string>

#include "IccCalibrator.hpp"

namespace kalibr {

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
