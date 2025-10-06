#pragma once

#include <matplot/matplot.h>

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>

namespace sm {
namespace plot {

/**
 * @brief Enhanced PlotCollection with dual indexing support
 *
 * This class manages a collection of matplot++ figures with support for:
 * - String-based naming (semantic access)
 * - Integer-based ID (numeric access for compatibility)
 * - Global singleton pattern for cross-compilation-unit access
 *
 * Usage example 1 (String-based):
 *   PlotCollection plotter("My Calibration Results");
 *   auto fig1 = matplot::figure();
 *   matplot::plot({1, 2, 3}, {2, 3, 4});
 *   plotter.add_figure("Plot 1", fig1);
 *   plotter.show();
 *
 * Usage example 2 (Integer-based, compatible with existing code):
 *   auto& registry = PlotCollection::global();
 *   auto fig = registry.get_or_create_figure(1001);
 *   matplot::gca()->plot({1, 2, 3}, {2, 3, 4});
 *
 * Usage example 3 (Mixed):
 *   auto& registry = PlotCollection::global();
 *   auto fig = registry.get_or_create_figure(1001, "IMU Rates");
 *   // Now accessible by ID or name
 *   auto same_fig = registry.get_figure_by_name("IMU Rates");
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
   * @brief Get the global singleton instance
   * @return Reference to the global PlotCollection
   */
  static PlotCollection& global() {
    static PlotCollection instance("Global Plot Collection");
    return instance;
  }

  /**
   * @brief Add a figure with a string name
   * @param tab_name Name for this plot (used as window title or filename)
   * @param fig Shared pointer to matplot figure
   */
  void add_figure(const std::string& tab_name,
                  std::shared_ptr<matplot::figure_type> fig) {
    figures_by_name_[tab_name] = fig;
  }

  /**
   * @brief Add a figure with an integer ID
   * @param figure_id Numeric ID for the figure
   * @param fig Shared pointer to matplot figure
   */
  void add_figure(int figure_id, std::shared_ptr<matplot::figure_type> fig) {
    figures_by_id_[figure_id] = fig;
  }

  /**
   * @brief Add a figure with both ID and name
   * @param figure_id Numeric ID for the figure
   * @param tab_name String name for the figure
   * @param fig Shared pointer to matplot figure
   */
  void add_figure(int figure_id, const std::string& tab_name,
                  std::shared_ptr<matplot::figure_type> fig) {
    figures_by_id_[figure_id] = fig;
    figures_by_name_[tab_name] = fig;
    id_to_name_[figure_id] = tab_name;
  }

  /**
   * @brief Get or create a figure by ID
   * @param figure_id Numeric ID for the figure
   * @param name Optional name to associate with the figure
   * @return Shared pointer to the figure
   */
  std::shared_ptr<matplot::figure_type> get_or_create_figure(
      int figure_id, const std::optional<std::string>& name = std::nullopt) {
    auto it = figures_by_id_.find(figure_id);
    if (it != figures_by_id_.end()) {
      return it->second;
    }

    // Create new figure
    auto fig = matplot::figure(true);  // quiet mode
    figures_by_id_[figure_id] = fig;

    // Associate with name if provided
    if (name.has_value()) {
      figures_by_name_[name.value()] = fig;
      id_to_name_[figure_id] = name.value();
    }

    return fig;
  }

  /**
   * @brief Get a figure by numeric ID
   * @param figure_id Numeric ID
   * @return Shared pointer to figure, or nullptr if not found
   */
  std::shared_ptr<matplot::figure_type> get_figure_by_id(int figure_id) const {
    auto it = figures_by_id_.find(figure_id);
    return (it != figures_by_id_.end()) ? it->second : nullptr;
  }

  /**
   * @brief Get a figure by string name
   * @param name Figure name
   * @return Shared pointer to figure, or nullptr if not found
   */
  std::shared_ptr<matplot::figure_type> get_figure_by_name(
      const std::string& name) const {
    auto it = figures_by_name_.find(name);
    return (it != figures_by_name_.end()) ? it->second : nullptr;
  }

  /**
   * @brief Get the name associated with a figure ID
   * @param figure_id Numeric ID
   * @return Optional string name
   */
  std::optional<std::string> get_name_by_id(int figure_id) const {
    auto it = id_to_name_.find(figure_id);
    if (it != id_to_name_.end()) {
      return it->second;
    }
    return std::nullopt;
  }

  /**
   * @brief Delete a figure by name
   * @param name Name of the figure to delete
   */
  void delete_figure(const std::string& name) {
    auto it = figures_by_name_.find(name);
    if (it != figures_by_name_.end()) {
      // Remove from ID map if it has an ID
      for (auto id_it = figures_by_id_.begin();
           id_it != figures_by_id_.end();) {
        if (id_it->second == it->second) {
          id_to_name_.erase(id_it->first);
          id_it = figures_by_id_.erase(id_it);
        } else {
          ++id_it;
        }
      }
      figures_by_name_.erase(it);
    }
  }

  /**
   * @brief Delete a figure by ID
   * @param figure_id Numeric ID of the figure to delete
   */
  void delete_figure(int figure_id) {
    auto it = figures_by_id_.find(figure_id);
    if (it != figures_by_id_.end()) {
      // Remove from name map if it has a name
      auto name_it = id_to_name_.find(figure_id);
      if (name_it != id_to_name_.end()) {
        figures_by_name_.erase(name_it->second);
        id_to_name_.erase(name_it);
      }
      figures_by_id_.erase(it);
    }
  }

  /**
   * @brief Show all figures (opens separate windows for each)
   * Note: matplotplusplus gnuplot backend doesn't support embedded tabs,
   * so each figure opens in its own window
   */
  void show() {
    if (figures_by_name_.empty() && figures_by_id_.empty()) {
      return;
    }

    // Show figures with names
    for (const auto& [name, fig] : figures_by_name_) {
      if (fig) {
        fig->title(window_name_ + " - " + name);
        fig->show();
      }
    }

    // Show figures with only IDs (not already shown via name map)
    for (const auto& [id, fig] : figures_by_id_) {
      if (fig && id_to_name_.find(id) == id_to_name_.end()) {
        fig->title(window_name_ + " - Figure " + std::to_string(id));
        fig->show();
      }
    }

    // Keep windows open until user closes them
    matplot::show();
  }

  /**
   * @brief Get total number of unique figures in collection
   */
  size_t size() const {
    // Count unique figures (some may be in both maps)
    std::set<std::shared_ptr<matplot::figure_type>> unique_figs;
    for (const auto& [_, fig] : figures_by_name_) {
      unique_figs.insert(fig);
    }
    for (const auto& [_, fig] : figures_by_id_) {
      unique_figs.insert(fig);
    }
    return unique_figs.size();
  }

  /**
   * @brief Clear all figures
   */
  void clear() {
    figures_by_name_.clear();
    figures_by_id_.clear();
    id_to_name_.clear();
  }

  std::string get_window_name() const { return window_name_; }

  /**
   * @brief Get all figures indexed by name
   */
  std::map<std::string, std::shared_ptr<matplot::figure_type>>
  get_figures_by_name() const {
    return figures_by_name_;
  }

  /**
   * @brief Get all figures indexed by ID
   */
  std::map<int, std::shared_ptr<matplot::figure_type>> get_figures_by_id()
      const {
    return figures_by_id_;
  }

 private:
  std::string window_name_;
  std::map<std::string, std::shared_ptr<matplot::figure_type>> figures_by_name_;
  std::map<int, std::shared_ptr<matplot::figure_type>> figures_by_id_;
  std::map<int, std::string> id_to_name_;  // Bidirectional mapping
};

}  // namespace plot
}  // namespace sm
