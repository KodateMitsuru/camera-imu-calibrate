#include <signal.h>

#include <argparse/argparse.hpp>
#include <format_utils.hpp>
#include <print>

#include "camera_cmd.hpp"
#include "imu_camera_cmd.hpp"

// Signal handler for graceful shutdown
void signalHandler([[maybe_unused]] int signal) {
  std::println("");
  std::println("Shutting down! (CTRL+C)");
  std::exit(1);
}

int main(int argc, char* argv[]) {
  // Setup signal handler
  signal(SIGINT, signalHandler);

  // Create main program with subcommands
  argparse::ArgumentParser program("calibrate", "1.0.0",
                                   argparse::default_arguments::help);
  program.add_description(
      "Camera and IMU calibration toolkit.\n\n"
      "Available subcommands:\n"
      "  imu_camera  - Calibrate IMU-camera spatial and temporal parameters\n"
      "  camera      - Calibrate camera intrinsics and extrinsics");

  // Create subcommands
  argparse::ArgumentParser imu_camera_cmd("imu_camera", "1.0.0",
                                          argparse::default_arguments::help);
  setupImuCameraArgs(imu_camera_cmd);

  argparse::ArgumentParser camera_cmd("camera", "1.0.0",
                                      argparse::default_arguments::help);
  setupCameraArgs(camera_cmd);

  // Add subcommands to main program
  program.add_subparser(imu_camera_cmd);
  program.add_subparser(camera_cmd);

  // Parse arguments
  try {
    program.parse_args(argc, argv);
  } catch (const std::exception& err) {
    std::println(stderr, "Error: {}", err.what());
    std::println(stderr, "");
    std::println(stderr, "{}", program.help().str());
    return 1;
  }

  // Dispatch to appropriate subcommand
  if (program.is_subcommand_used("imu_camera")) {
    return runImuCameraCalibration(imu_camera_cmd);
  } else if (program.is_subcommand_used("camera")) {
    return runCameraCalibration(camera_cmd);
  } else {
    // No subcommand specified
    std::println("{}", program.help().str());
    return 0;
  }
}