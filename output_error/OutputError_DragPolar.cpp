#include "OutputError_DragPolar.h"

#include <string>
#include <iostream>
#include <fstream>
#include <exception>

#include <boost/numeric/odeint/integrate/integrate.hpp>

#include <Eigen/Dense>

OutputError_DragPolar::OutputError_DragPolar(const Inertia &inertia, const LogDataExtractor_IMU *log_data_imu, const LogDataExtractor_ARSP *log_data_arsp) : 
    OutputError<num_state_variables, num_output_variables, num_control_variables, num_parameters>(),
    _inertia(inertia)
{
    // TODO: Transfer log data into internal data storage
}


void OutputError_DragPolar::state_equation(const DynamicState &x, const ControlState &u, DynamicState &dxdt, const double t)
{

}


void OutputError_DragPolar::output_equation(const DynamicState &x, const ControlState &u, OutputState &y, const double t)
{
    y = OutputState();
    y[0] = x[0];
    y[1] = x[1];
    y[2] = x[2];
    // y[3] = 
}