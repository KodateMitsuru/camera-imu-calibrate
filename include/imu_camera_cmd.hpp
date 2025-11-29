#pragma once

#include <argparse/argparse.hpp>

void setupImuCameraArgs(argparse::ArgumentParser& cmd);
int runImuCameraCalibration(argparse::ArgumentParser& cmd);
