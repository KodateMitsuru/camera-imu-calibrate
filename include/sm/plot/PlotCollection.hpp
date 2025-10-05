#pragma once

#include <matplot/matplot.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace sm {
namespace plot {

/**
 * @brief C++ port of Python PlotCollection for matplotplusplus
 *
 * Usage example:
 *   PlotCollection plotter("My Calibration Results");
 *
 *   auto fig1 = matplot::figure();
 *   matplot::plot({1, 2, 3}, {2, 3, 4});
 *   plotter.add_figure("Plot 1", fig1);
 *
 *   auto fig2 = matplot::figure();
 *   matplot::scatter({1, 2, 3}, {4, 5, 6});
 *   plotter.add_figure("Plot 2", fig2);
 *
 *   plotter.show();  // Shows all figures in separate windows
 *   // or
 *   plotter.save_all("output_dir");  // Saves all to PNG files
 */
class PlotCollection {
 public:
  /**
   * @brief Construct a new Plot Collection
   * @param window_name Name prefix for plot windows
   */
  explicit PlotCollection(const std::string& window_name = "Plot Collection")
      : window_name_(window_name) {}

  /**
   * @brief Add a figure to the collection
   * @param tab_name Name for this plot (used as window title or filename)
   * @param fig Shared pointer to matplot figure
   */
  void add_figure(const std::string& tab_name,
                  std::shared_ptr<matplot::figure_type> fig) {
    figures_[tab_name] = fig;
  }

  /**
   * @brief Delete a figure from the collection
   * @param name Name of the figure to delete
   */
  void delete_figure(const std::string& name) { figures_.erase(name); }

  /**
   * @brief Show all figures (opens separate windows for each)
   * Note: matplotplusplus gnuplot backend doesn't support embedded tabs,
   * so each figure opens in its own window
   */
  void show() {
    if (figures_.empty()) {
      return;
    }

    for (const auto& [name, fig] : figures_) {
      if (fig) {
        fig->title(window_name_ + " - " + name);
        fig->show();
      }
    }

    // Keep windows open until user closes them
    matplot::show();
  }

  /**
   * @brief Get number of figures in collection
   */
  size_t size() const { return figures_.size(); }

  /**
   * @brief Clear all figures
   */
  void clear() { figures_.clear(); }

  std::string get_window_name() const { return window_name_; }

  std::map<std::string, std::shared_ptr<matplot::figure_type>> get_figures()
      const {
    return figures_;
  }

 private:
  std::string window_name_;
  std::map<std::string, std::shared_ptr<matplot::figure_type>> figures_;
};

}  // namespace plot
}  // namespace sm
