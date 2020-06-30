function dxdt = compute_numerical_derivatives(t, x)
% t : Times (N,1)
% x : Values (N,1)

% fit piecewise polynomials, then evaluate the exact derivatives of these
% polynomials

assert(length(t)==length(x), 't,x must be the same length');

% higher values are more accurate
N = 10;

% compute the numerical derivatives
dxdt = numerical_derivative(t, x, N);

end

function df_dt = numerical_derivative(t, f, N)

    dT = (t(2)-t(1)) / N;
    T = (t(1) : dT : t(end) + 2*dT)';
    f_interp = interp1(t, f, T, 'spline');
    df_dt = diff(f_interp, 1) / dT;
    df_dt = df_dt(1:N:end);
    
end