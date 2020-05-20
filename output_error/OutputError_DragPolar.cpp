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
    OutputError<oe_dragpolar::num_state_variables, oe_dragpolar::num_output_variables, oe_dragpolar::num_control_variables, oe_dragpolar::num_parameters>(),
    airspeed_interpolation_type(AirspeedInterpolationType::MEAN),
    _inertia(inertia),
    _dimensions(dimensions)
{
    extract_imu_data(log_data_imu);
    extract_arsp_data(log_data_arsp);
}


void OutputError_DragPolar::state_equation(const DynamicState &state, const ControlState &control, const Parameters &param, 
    DynamicState &state_derivative, const double time)
{
    // x = [beta, p, r, phi]
    // (4x1)
    // u = [delta_a, delta_r, 1]
    // (5x1)
    // theta = [C_Y_beta, C_Y_r, C_Y_delta_r, b_betadot,
    //          C_l_beta, C_l_p, C_l_r, C_l_delta_a, C_l_delta_r, b_pdot,
    //          C_n_beta, C_n_p, C_n_r, C_n_delta_a, C_n_delta_r, b_rdot, b_phidot, b_a_y]
    // (18x1)

    // TODO: Decide how these steady-state variables can be imported
    const double alpha_0 = 1.0 * M_PI / 180.0;
    const double theta_0 = 2.0 * M_PI / 180.0;
    const double V_0 = 62.0;

    const double S = _dimensions.S;
    const double b = _dimensions.b;

    const double m = _inertia.m;
    const double Ix = _inertia.Ix;
    const double Iy = _inertia.Iy;
    const double Iz = _inertia.Iz;
    const double Ixz = _inertia.Ixz;
    const double Gamma = _inertia.c[0];
    
    const double C_Y_beta = param[0];
    const double C_Y_r = param[1];
    const double C_Y_delta_r = param[2];
    const double b_betadot = param[3];
    const double C_l_beta = param[4];
    const double C_l_p = param[5];
    const double C_l_r = param[6];
    const double C_l_delta_a = param[7];
    const double C_l_delta_r = param[8];
    const double b_pdot = param[9];
    const double C_n_beta = param[10];
    const double C_n_p = param[11];
    const double C_n_r = param[12];
    const double C_n_delta_a = param[13];
    const double C_n_delta_r = param[14];
    const double b_rdot = param[15];
    const double b_phidot = param[16];
    const double b_a_y = param[17];

    const double q0 = 0.5 * rho * V_0 * V_0;
    const double k1 = q0 * S / m / V_0;
    const double k2 = b / 2.0 / V_0;
    const double k3 = q0 * S * b;
    const double k4 = q0 * S / m / g;
    const double c3 = _inertia.c[3];
    const double c4 = _inertia.c[4];
    const double c9 = _inertia.c[9];

    Eigen::Matrix<double, 4, 4> A;
    A << k1 * C_Y_beta,  sin(alpha_0),  k1 * C_Y_r * k2 - cos(alpha_0),  g * cos(theta_0) / V_0,
         k3 * (c3 * C_l_beta + c4 * C_n_beta),  k3 * b / 2 / V_0 * (c3 * C_l_p + c4 * C_n_p),  k3 * b / 2 / V_0 * (c3 * C_l_r + c4 * C_n_r),  0,
         k3 * (c4 * C_l_beta + c9 * C_n_beta),  k3 * b / 2 / V_0 * (c4 * C_l_p + c9 * C_n_p),  k3 * b / 2 / V_0 * (c4 * C_l_r + c9 * C_n_r),  0,
         0,  1,  tan(theta_0),  0;

    Eigen::Matrix<double, 4, 3> B;
    B << 0,  k1 * C_Y_delta_r,  b_betadot,
         k3 * (c3 * C_l_delta_a + c4 * C_n_delta_a),  k3 * (c3 * C_l_delta_r + c4 * C_n_delta_r),  b_pdot,
         k3 * (c4 * C_l_delta_a + c9 * C_n_delta_a),  k3 * (c4 * C_l_delta_r + c9 * C_n_delta_r),  b_rdot,
         0,  0,  b_phidot;

    state_derivative = A * state + B * control;
}


void OutputError_DragPolar::output_equation(const DynamicState &state, const ControlState &control, const Parameters &param, OutputState &output, 
                                            const double time)
{
    const double alpha_0 = 1.0 * M_PI / 180.0;
    const double theta_0 = 2.0 * M_PI / 180.0;
    const double V_0 = 62.0;

    const double S = _dimensions.S;
    const double b = _dimensions.b;

    const double m = _inertia.m;
    const double Ix = _inertia.Ix;
    const double Iy = _inertia.Iy;
    const double Iz = _inertia.Iz;
    const double Ixz = _inertia.Ixz;
    const double Gamma = _inertia.c[0];
    
    const double C_Y_beta = param[0];
    const double C_Y_r = param[1];
    const double C_Y_delta_r = param[2];
    const double b_betadot = param[3];
    const double C_l_beta = param[4];
    const double C_l_p = param[5];
    const double C_l_r = param[6];
    const double C_l_delta_a = param[7];
    const double C_l_delta_r = param[8];
    const double b_pdot = param[9];
    const double C_n_beta = param[10];
    const double C_n_p = param[11];
    const double C_n_r = param[12];
    const double C_n_delta_a = param[13];
    const double C_n_delta_r = param[14];
    const double b_rdot = param[15];
    const double b_phidot = param[16];
    const double b_a_y = param[17];

    const double q0 = 0.5 * rho * V_0 * V_0;
    const double k1 = q0 * S / m / V_0;
    const double k2 = b / 2.0 / V_0;
    const double k3 = q0 * S * b;
    const double k4 = q0 * S / m / g;
    const double c3 = _inertia.c[3];
    const double c4 = _inertia.c[4];
    const double c9 = _inertia.c[9];

    Eigen::Matrix<double, 5, 4> C;
    C << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1,
         k4 * C_Y_beta, 0, k4 * C_Y_r * k2, 0;

    Eigen::Matrix<double, 5, 3> D;
    D << 0, 0, 0,
         0, 0, 0,
         0, 0, 0,
         0, 0, 0,
         0, k4 * C_Y_delta_r, b_a_y;

    output = C * state + D * control;
}


// Transfer IMU data into _measured_state and _time
void OutputError_DragPolar::extract_imu_data(const LogDataExtractor_IMU *log_data_imu)
{
    const std::vector<std::array<double, 7>> &imu_data = log_data_imu->get_data();
    _time.resize(imu_data.size());
    _measured_state.resize(imu_data.size());
    for (size_t i = 0; i < imu_data.size(); ++i) {
        _time[i] = imu_data[i][0];
        for (size_t j = 1; j < imu_data[i].size(); ++j) {
            _measured_state[i][j-1] = imu_data[i][j];
        }
    }
}


// Transfer ARSP data into _airspeed
// Since ARSP data is sampled at a lower rate than IMU data, interpolation may be used to approximate the missing data
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
    
    // airspeed data can be handled in different ways
    _airspeed.resize(_measured_state.size()); 
    switch (airspeed_interpolation_type) {
        case AirspeedInterpolationType::MEAN:
        {
            double mean_airspeed = 0;
            for (size_t i = 0; i < V.size(); ++i) {
                mean_airspeed += V[i];
            }
            mean_airspeed /= V.size();

            for (size_t i = 0; i < _time.size(); ++i) {
                _airspeed[i] = mean_airspeed;
            }
        }
        case AirspeedInterpolationType::BARYCENTRIC_RATIONAL:
        {
            // create the interpolant using the raw data
            boost::math::barycentric_rational<double> interpolant(t.data(), V.data(), V.size(), 1);

            // perform the interpolation
            for (size_t i = 0; i < _time.size(); ++i) {
                _airspeed[i] = interpolant(_time[i]);
            }
        }
    }
}