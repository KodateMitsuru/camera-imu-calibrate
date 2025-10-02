#include <iostream>
#include <argparse/argparse.hpp>
#include "IccCalibrator.hpp"

int main(int argc,char* argv[])
{
    argparse::ArgumentParser program("camera_imu_calibrate");

    program.add_argument("-f","--files")
        .help("Path to the files")
        .required();
    program.add_argument("-c","--config")
        .help("Path to the configuration file")
        .required();
    program.add_argument("-o","--output")
        .help("Path to the output file")
        .default_value(std::string("./output.yaml"));

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << "Error: " << err.what() << std::endl;
        return 1;
    }

    IccCalibrator calibrator;
    calibrator.loadConfig(program.get<std::string>("--config"));
    calibrator.loadData(program.get<std::string>("--files"));
    calibrator.calibrate();
    calibrator.saveResults(program.get<std::string>("--output"));

    return 0;
}