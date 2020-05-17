#pragma once 

#include "OutputError.h"
#include "Inertia.h"
#include "LogDataExtractor.h"

#include <cstring>

#include <string>

#include <Eigen/Dense>



class OutputError_DragPolar : public OutputError<8,6,4,31>
{
public:
    OutputError_DragPolar(const Inertia &inertia, const LogDataExtractor_IMU *log_data_imu, const LogDataExtractor_ARSP *log_data_arsp);

    static constexpr unsigned int num_state_variables = 8;
    static constexpr unsigned int num_output_variables = 6;
    static constexpr unsigned int num_control_variables = 4;
    static constexpr unsigned int num_parameters = 31;


private:
    void state_equation(const DynamicState &x, const ControlState &u, DynamicState &dxdt, const double t) override; // f(x(t), u(t))
    void output_equation(const DynamicState &x, const ControlState &u, OutputState &y, const double t) override; // g(x(t), u(t))

    Inertia _inertia;
};