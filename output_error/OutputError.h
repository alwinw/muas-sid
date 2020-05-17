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
    std::vector<double> _times;               // t(i); common to all states
    std::vector<OutputState> _measured;       // z(i)
    std::vector<DynamicState> _model_state;   // x(i)
    std::vector<ControlState> _model_control; // u(i)
    std::vector<OutputState> _model_output;   // y(i)
    std::vector<Parameters> _parameters;      // theta(i)

    Eigen::Matrix<double, No, No> _measurement_noise_covariance_matrix; // R

    // Specified by derived classes 
    virtual void state_equation(const DynamicState &x, const ControlState &u, DynamicState &dxdt, const double t) = 0;
    virtual void output_equation(const DynamicState &x, const ControlState &u, OutputState &y, const double t) = 0;

    // Uses an initial state to compute values for _model_output
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

        DynamicState x0 = initial_state;

        for (size_t i = 0; i < _measured.size() - 1; ++i) {
            const double t0 = _times[i];
            const double t1 = _times[i+1];
            const double dt = (t1 - t0) / 10;
            const OutputState &u = _model_output[i];
            const auto f = std::bind(state_equation, _1, u, _2, _3, _4);
            double x, t;
            boost::numeric::odeint::integrate(f, x0, t0, t1, dt, Observer(x, t));
            _model_output[i+1] = x;
            x0 = x;
        }
    }
};
