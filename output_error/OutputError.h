#pragma once

#include <vector>
#include <functional>

#include <Eigen/Dense>
#include <boost/numeric/odeint/integrate/integrate.hpp>

template <unsigned int Ns, unsigned int No, unsigned int Nc, unsigned int Np> // Number of states, outputs, controls, and parameters 
class OutputError
{
public:
    typedef Eigen::Matrix<double, Ns, 1> DynamicState; 
    typedef Eigen::Matrix<double, No, 1> OutputState; 
    typedef Eigen::Matrix<double, Nc, 1> ControlState; 
    typedef Eigen::Matrix<double, Np, 1> Parameters; 

protected:
    std::vector<double> _time;                   // t(i); common to all states
    std::vector<OutputState> _measured_state;    // z(i)
    std::vector<ControlState> _measured_control; // u(i)
    std::vector<DynamicState> _model_state;      // x(i)
    std::vector<OutputState> _model_output;      // y(i)
    Parameters _parameters;                      // theta
    std::vector<double> _airspeed;               // V(i)

    Eigen::Matrix<double, No, No> _measurement_noise_covariance_matrix; // R

    // Specified by derived classes 
    virtual void state_equation(const DynamicState &x, const ControlState &u, const Parameters &p, DynamicState &dxdt, const double t) = 0;
    virtual void output_equation(const DynamicState &x, const ControlState &u, const Parameters &p, OutputState &y, const double t) = 0;

    // Uses an initial state to populate _model_state and _model_control given the current set of parameters
    void compute_model_output(const DynamicState &initial_state)
    {
        using namespace std::placeholders; // for std::bind placeholder arguments

        // Required to get outputs from the integrator
        struct Observer
        {
            DynamicState x;
            double t;

            Observer(DynamicState &state, double &time) :
                x(state),
                t(time)
            {}

            void operator()(const DynamicState &state, double time)
            {
                x = state;
                t = time;
            }
        };

        // Initial dynamic state
        DynamicState x0 = initial_state;

        // Compute the initial output state
        output_equation(x0, _measured_control[0], _parameters, _model_output[0], _time[0]);

        for (size_t i = 0; i < _measured_state.size() - 1; ++i) {
            // integrate the dynamic state forward in time
            const double t0 = _time[i];
            const double t1 = _time[i+1];
            const double dt = (t1 - t0) / 10;
            const OutputState &u = _measured_control[i];
            const Parameters &p = _parameters;
            const auto f = std::bind(state_equation, _1, u, p, _2, _3, _4);
            double x, t;
            boost::numeric::odeint::integrate(f, x0, t0, t1, dt, Observer(x, t));
            _model_state[i+1] = x;

            // compute the corresponding output state
            output_equation(_model_state[i+1], _measured_control[i+1], _parameters, _model_output[i+1], t1);

            // set the next initial condition
            x0 = x;
        }
    }
};
