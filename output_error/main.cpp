#include "OutputError_DragPolar.h"
#include "LogDataExtractor.h"
#include "Inertia.h"
#include "FixedWingDimensions.h"

#include <string>

int main(void)
{
    constexpr double m = 10.0;
    constexpr double Ix = 1.0;
    constexpr double Iy = 1.0;
    constexpr double Iz = 1.0;
    constexpr double Ixy = 1.0;
    constexpr double Iyz = 1.0;
    constexpr double Ixz = 1.0;
    Inertia inertia(m, Ix, Iy, Iz, Ixy, Iyz, Ixz);

    constexpr double S = 0.6;
    constexpr double b = 2.05;
    constexpr double c = 0.3;
    FixedWingDimensions dimensions(S, b, c);

    constexpr double start_time = 300;
    constexpr double duration = 10;
    LogDataExtractor_IMU *log_data_extractor_imu = new LogDataExtractor_IMU("../imu_data_support.csv", start_time, duration);
    LogDataExtractor_ARSP *log_data_extractor_arsp = new LogDataExtractor_ARSP("../arsp_data_support.csv", start_time, duration);

    OutputError_DragPolar output_error(inertia, dimensions, log_data_extractor_imu, log_data_extractor_arsp);

    delete log_data_extractor_imu;
    delete log_data_extractor_arsp;

    return 0;
}