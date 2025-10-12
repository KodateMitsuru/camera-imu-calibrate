#include <matplot/matplot.h>

#include <Eigen/Core>
#include <sm/kinematics/RotationVector.hpp>
#include <sm/kinematics/transformations.hpp>
#include <string>

namespace sm {
namespace plot {
/**
 * @brief Plot a coordinate frame on a 3d axis. In the resulting plot,
 *        x = red, y = green, z = blue.
 *
 * @param ax Pointer to a matplot axes object with 3D projection
 * @param T_0f The 4x4 transformation matrix that takes points from the frame
 *             of interest to the plotting frame
 * @param size The length of each line in the coordinate frame (default: 1.0)
 * @param linewidth The width of each line in the coordinate frame
 * (default: 3.0)
 * @param name Optional name label for the coordinate frame (default: nullptr)
 *
 * Usage example:
 *   auto fig = matplot::figure();
 *   auto ax = fig->add_subplot(1, 1, 1, true);  // true for 3D
 *   Eigen::Matrix4d T_0f = Eigen::Matrix4d::Identity();
 *   // ... set up your transformation matrix
 *   sm::plot::plotCoordinateFrame(ax, T_0f);
 */
inline void plotCoordinateFrame(matplot::axes_handle ax,
                                const Eigen::Matrix4d& T_0f, double size = 1.0,
                                double linewidth = 3.0,
                                const std::string /*name*/ = "") {
  // Create points in the frame: origin and three axis endpoints
  // Columns: origin, x-axis, y-axis, z-axis
  Eigen::Matrix4d p_f;
  p_f << 0, 0, 0, 1, size, 0, 0, 1, 0, size, 0, 1, 0, 0, size, 1;
  p_f.transposeInPlace();

  // Transform points to the plotting frame
  Eigen::Matrix4d p_0 = T_0f * p_f;

  // Extract coordinates for plotting
  // X axis: from origin (column 0) to x-endpoint (column 1)
  std::vector<double> X_x = {p_0(0, 0), p_0(0, 1)};
  std::vector<double> X_y = {p_0(1, 0), p_0(1, 1)};
  std::vector<double> X_z = {p_0(2, 0), p_0(2, 1)};

  // Y axis: from origin (column 0) to y-endpoint (column 2)
  std::vector<double> Y_x = {p_0(0, 0), p_0(0, 2)};
  std::vector<double> Y_y = {p_0(1, 0), p_0(1, 2)};
  std::vector<double> Y_z = {p_0(2, 0), p_0(2, 2)};

  // Z axis: from origin (column 0) to z-endpoint (column 3)
  std::vector<double> Z_x = {p_0(0, 0), p_0(0, 3)};
  std::vector<double> Z_y = {p_0(1, 0), p_0(1, 3)};
  std::vector<double> Z_z = {p_0(2, 0), p_0(2, 3)};

  // Plot the three axes
  auto line_x = ax->plot3(X_x, X_y, X_z, "r-");
  line_x->line_width(linewidth);

  auto line_y = ax->plot3(Y_x, Y_y, Y_z, "g-");
  line_y->line_width(linewidth);

  auto line_z = ax->plot3(Z_x, Z_y, Z_z, "b-");
  line_z->line_width(linewidth);

  // Add text label if name is provided
  // if (!name.empty()) {
  //   // Note: matplotplusplus's text support for 3D may be limited
  //   // Using text at the origin position as a workaround
  //   // ax->text(p_0(0, 0), p_0(1, 0), name);
  // }
}
}  // namespace plot
}  // namespace sm