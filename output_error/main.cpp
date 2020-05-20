#include "OutputError_DragPolar.h"
#include "LogDataExtractor.h"
#include "Inertia.h"
#include "FixedWingDimensions.h"

#include <string>

int main(void)
{
    constexpr double m = 4500;
    constexpr double Ix = 22370;
    constexpr double Iy = 36060;
    constexpr double Iz = 53960;
    constexpr double Ixy = 0.0;
    constexpr double Iyz = 0.0;
    constexpr double Ixz = 1630;
    Inertia inertia(m, Ix, Iy, Iz, Ixy, Iyz, Ixz);

    constexpr double S = 39;
    constexpr double b = 19.8;
    constexpr double c = 0.0;
    FixedWingDimensions dimensions(S, b, c);

    constexpr double start_time = 0;
    constexpr double duration = 40;
    LogDataExtractor_IMU *log_data_extractor_imu = new LogDataExtractor_IMU("../imu_data_test.csv", start_time, duration);
    LogDataExtractor_ARSP *log_data_extractor_arsp = new LogDataExtractor_ARSP("../arsp_data_test.csv", start_time, duration);

    OutputError_DragPolar output_error(inertia, dimensions, log_data_extractor_imu, log_data_extractor_arsp);

    delete log_data_extractor_imu;
    delete log_data_extractor_arsp;

    return 0;
}