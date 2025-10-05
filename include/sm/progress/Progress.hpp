#ifndef SM_PROGRESS_HPP
#define SM_PROGRESS_HPP

#include <chrono>
#include <format>
#include <print>

namespace sm {
namespace progress {
/**
 * @brief Simple progress tracker that displays iteration progress and time
 * estimates
 *
 * This class tracks the progress of iterative processes and displays
 * elapsed time and estimated total time.
 */
class Progress {
 public:
  /**
   * @brief Constructor
   * @param numIterations Total number of iterations expected
   */
  explicit Progress(int numIterations)
      : started_(false),
        elapsed_(0.0),
        startTime_(),
        numIterations_(numIterations),
        iteration_(0) {}

  /**
   * @brief Sample the progress. Call this at each iteration.
   *
   * On first call, starts the timer. On subsequent calls, prints
   * progress information including elapsed and estimated total time.
   */
  void sample() {
    if (started_) {
      iteration_++;
      auto now = std::chrono::steady_clock::now();
      auto elapsed = now - startTime_;

      auto timePerRun = elapsed / iteration_;
      auto totalTime = timePerRun * numIterations_;

      std::println("Progress {} / {}", iteration_, numIterations_);
      std::println("Time {} / {}  ({} * {})", elapsed, totalTime, timePerRun,
                   numIterations_);

      elapsed_ = std::chrono::duration<double>(elapsed).count();
    } else {
      startTime_ = std::chrono::steady_clock::now();
      iteration_ = 0;
      started_ = true;
    }
  }

 private:
  bool started_;
  double elapsed_;
  std::chrono::steady_clock::time_point startTime_;
  int numIterations_;
  int iteration_;
};

/**
 * @brief Enhanced progress tracker with single-line updating display
 *
 * This version provides a cleaner output with in-place updating of the
 * progress line and formatted time remaining display.
 *
 * Example usage:
 * @code
 *   #include <sm/progress/Progress.hpp>
 *   #include <thread>
 *   #include <chrono>
 *
 *   int numIter = 10;
 *   sm::Progress2 progress(numIter);
 *   for (int iter = 0; iter < numIter; ++iter) {
 *       progress.sample();
 *       std::this_thread::sleep_for(std::chrono::seconds(1));
 *   }
 *   std::cout << std::endl;  // Add newline after progress completes
 * @endcode
 */
class Progress2 {
 public:
  /**
   * @brief Constructor
   * @param numIterations Total number of iterations expected
   */
  explicit Progress2(int numIterations)
      : started_(false),
        elapsed_(0.0),
        startTime_(),
        numIterations_(numIterations),
        iteration_(0) {}

  /**
   * @brief Sample the progress. Call this at each iteration.
   *
   * Prints remaining steps and time on a single line that updates in place.
   *
   * @param steps Number of steps to increment (default: 1)
   */
  void sample(int steps = 1) {
    if (started_) {
      iteration_ += steps;
      auto now = std::chrono::steady_clock::now();
      auto elapsed = now - startTime_;
      elapsed_ = std::chrono::duration<double>(elapsed).count();

      auto timePerRun = elapsed / iteration_;
      auto totalTime = timePerRun * numIterations_;
      auto remaining = totalTime - elapsed;

      // Format remaining time using chrono format specifiers
      using namespace std::chrono;
      auto remainingSecs = duration_cast<seconds>(remaining);

      // Create human-readable format based on magnitude
      std::string tRemainingStr;
      if (remainingSecs >= 1h) {
        // Format: "2h 30m 45s"
        tRemainingStr = std::format("{:%Hh %Mm %Ss}", remainingSecs);
      } else if (remainingSecs >= 1min) {
        // Format: "30m 45s"
        tRemainingStr = std::format("{:%Mm %Ss}", remainingSecs);
      } else {
        // Format: "45s"
        tRemainingStr = std::format("{:%Ss}", remainingSecs);
      }

      // Print with carriage return to overwrite previous line
      std::print("\r  Progress {} / {} \t Time remaining: {}                 ",
                 iteration_, numIterations_, tRemainingStr);
    } else {
      startTime_ = std::chrono::steady_clock::now();
      iteration_ = 0;
      started_ = true;
    }
  }

  /**
   * @brief Reset the progress tracker
   * @param numIterations New total number of iterations (optional, -1 keeps
   * current value)
   */
  void reset(int numIterations = -1) {
    started_ = false;
    elapsed_ = 0.0;
    startTime_ = std::chrono::steady_clock::time_point();
    iteration_ = 0;

    if (numIterations != -1) {
      numIterations_ = numIterations;
    }
  }

  /**
   * @brief Get current iteration number
   */
  int getIteration() const { return iteration_; }

  /**
   * @brief Get total number of iterations
   */
  int getNumIterations() const { return numIterations_; }

  /**
   * @brief Get elapsed time in seconds
   */
  double getElapsed() const { return elapsed_; }

 private:
  bool started_;
  double elapsed_;
  std::chrono::steady_clock::time_point startTime_;
  int numIterations_;
  int iteration_;
};
}  // namespace progress
}  // namespace sm

#endif  // SM_PROGRESS_HPP
