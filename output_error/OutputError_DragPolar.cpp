#include "OutputError_DragPolar.h"

#include <string>
#include <iostream>
#include <fstream>
#include <exception>
#include <utility>

#include <boost/numeric/odeint/integrate/integrate.hpp>
#include <boost/math/interpolators/barycentric_rational.hpp>

#include <Eigen/Dense>

OutputError_DragPolar::OutputError_DragPolar(const Inertia &inertia, const FixedWingDimensions &dimensions, 
                                             const LogDataExtractor_IMU *log_data_imu, const LogDataExtractor_ARSP *log_data_arsp) : 
    OutputError<num_state_variables, num_output_variables, num_control_variables, num_parameters>(),
    _inertia(inertia),
    _dimensions(dimensions)
{
    extract_imu_data(log_data_imu);
    extract_arsp_data(log_data_arsp);
}


void OutputError_DragPolar::state_equation(const DynamicState &x, const ControlState &u, const Parameters &p, DynamicState &dxdt, const double t)
{
    // TODO: 
}


void OutputError_DragPolar::output_equation(const DynamicState &x, const ControlState &u, const Parameters &p, OutputState &y, const double t)
{
    // TODO:
    y = OutputState();
    y[0] = x[0];
    y[1] = x[1];
    y[2] = x[2];
    // y[3] = 
}


// Transfer IMU data into _measured_state and _time
void OutputError_DragPolar::extract_imu_data(const LogDataExtractor_IMU *log_data_imu)
{
    const std::vector<std::array<double, 7>> &imu_data = log_data_imu->get_data();
    _measured_state.reserve(imu_data.size());
    for (size_t i = 0; i < imu_data.size(); ++i) {
        _time[i] = imu_data[i][0];
        for (size_t j = 1; j < imu_data[i].size(); ++j) {
            _measured_state[i][j] = imu_data[i][j];
        }
    }
}


// Transfer ARSP data into _airspeed
// Since ARSP data is sampled at a lower rate than IMU data, interpolation is used to approximate the missing data
void OutputError_DragPolar::extract_arsp_data(const LogDataExtractor_ARSP *log_data_arsp)
{
    // extract the raw airspeed data
    const std::vector<std::array<double, 2>> &arsp_data = log_data_arsp->get_data();
    std::vector<double> t(arsp_data.size());
    std::vector<double> V(arsp_data.size());
    for (size_t i = 0; i < arsp_data.size(); ++i) {
        t[i] = arsp_data[i][0];
        V[i] = arsp_data[i][1];
    }

    // create the interpolant using the raw data
    boost::math::barycentric_rational<double> interpolant(t.data(), V.data(), V.size());
    
    // perform the interpolation
    _airspeed.reserve(_measured_state.size());
    for (size_t i = 0; i < _time.size(); ++i) {
        _airspeed[i] = interpolant(_time[i]);
    }
}