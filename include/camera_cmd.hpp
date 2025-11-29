#pragma once

#include <argparse/argparse.hpp>

void setupCameraArgs(argparse::ArgumentParser& cmd);
int runCameraCalibration(argparse::ArgumentParser& cmd);
