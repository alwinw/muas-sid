#pragma once 

#include "OutputError.h"
#include "Inertia.h"
#include "FixedWingDimensions.h"
#include "LogDataExtractor.h"

#include <cstring>

#include <string>

#include <Eigen/Dense>


namespace oe_dragpolar
{
    constexpr unsigned int num_state_variables = 4;
    constexpr unsigned int num_output_variables = 5;
    constexpr unsigned int num_control_variables = 3;
    constexpr unsigned int num_parameters = 18; 
};


class OutputError_DragPolar : public OutputError<oe_dragpolar::num_state_variables, oe_dragpolar::num_output_variables, 
                                                 oe_dragpolar::num_control_variables, oe_dragpolar::num_parameters>
{
public:
    OutputError_DragPolar(const Inertia &inertia, const FixedWingDimensions &dimensions, 
                          const LogDataExtractor_IMU *log_data_imu, const LogDataExtractor_ARSP *log_data_arsp);

    // Options
    enum class AirspeedInterpolationType
    {
        MEAN,
        BARYCENTRIC_RATIONAL,
    } airspeed_interpolation_type;


private:
    // f(x(t,p), u(t,p), t)
    void state_equation(const DynamicState &state, const ControlState &control, const Parameters &param, DynamicState &sate_derivative, const double time) override; 
    // g(x(t,p), u(t,p), t)
    void output_equation(const DynamicState &state, const ControlState &control, const Parameters &param, OutputState &state_derivative, const double time) override; 

    // Log data
    void extract_imu_data(const LogDataExtractor_IMU *log_data_imu);
    void extract_arsp_data(const LogDataExtractor_ARSP *log_data_arsp);

    // Constants
    Inertia _inertia;
    FixedWingDimensions _dimensions;
    static constexpr double rho = 1.225;
    static constexpr double g = 9.81;
};