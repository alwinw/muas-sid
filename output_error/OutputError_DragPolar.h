#pragma once 

#include "OutputError.h"
#include "Inertia.h"
#include "FixedWingDimensions.h"
#include "LogDataExtractor.h"

#include <cstring>

#include <string>

#include <Eigen/Dense>



class OutputError_DragPolar : public OutputError<8,6,4,31>
{
public:
    OutputError_DragPolar(const Inertia &inertia, const FixedWingDimensions &dimensions, 
                          const LogDataExtractor_IMU *log_data_imu, const LogDataExtractor_ARSP *log_data_arsp);

    static constexpr unsigned int num_state_variables = 8;
    static constexpr unsigned int num_output_variables = 6;
    static constexpr unsigned int num_control_variables = 4;
    static constexpr unsigned int num_parameters = 31;


private:
    // f(x(t,p), u(t,p), t)
    void state_equation(const DynamicState &x, const ControlState &u, const Parameters &p, DynamicState &dxdt, const double t) override; 
    // g(x(t,p), u(t,p), t)
    void output_equation(const DynamicState &x, const ControlState &u, const Parameters &p, OutputState &y, const double t) override; 

    // Log data
    void extract_imu_data(const LogDataExtractor_IMU *log_data_imu);
    void extract_arsp_data(const LogDataExtractor_ARSP *log_data_arsp);

    // Constants
    Inertia _inertia;
    FixedWingDimensions _dimensions;
    static constexpr double rho = 1.225;
    static constexpr double g = 9.81;
};