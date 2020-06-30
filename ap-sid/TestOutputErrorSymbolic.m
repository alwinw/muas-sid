% define the state and output equations
syms s_t s_x1 s_x2 s_u1 s_u2 s_theta1 s_theta2 s_theta3 s_theta4 s_theta5 s_theta6
s_x = [s_x1; s_x2];
s_u = [s_u1; s_u2];
s_theta = [s_theta1; s_theta2; s_theta3; s_theta4; s_theta5; s_theta6];
f = @(x,u,theta,t) [-theta(1) theta(4); 0 -theta(2)]*x + [5 theta(3); 0 5]*u + theta(5:end);
h = @(x,u,theta,t) [1 0; 0 1]*x + [0 0; 0 0]*u ;

% define the Jacobians
% symbolic computation of Jacobians
Jfx = jacobian(f(s_x, s_u, s_theta), s_x);
Jft = jacobian(f(s_x, s_u, s_theta), s_theta);
Jhx = jacobian(h(s_x, s_u, s_theta), s_x);
Jht = jacobian(h(s_x, s_u, s_theta), s_theta);
% create functions 
Jfx = matlabFunction(Jfx, 'vars', {s_x, s_u, s_theta, s_t});
Jft = matlabFunction(Jft, 'vars', {s_x, s_u, s_theta, s_t});
Jhx = matlabFunction(Jhx, 'vars', {s_x, s_u, s_theta, s_t});
Jht = matlabFunction(Jht, 'vars', {s_x, s_u, s_theta, s_t});
%%
% setup
bias = [0.02; -0.06];
theta_true = [1.4;3.8;2;1; bias];
x0 = [1;2];
dT = 1/50;
T = (0:dT:10)';
N = length(T);

% add some variance to the sampling time
for i=1:length(T)
    T(i) = T(i) + 0.1*dT*randn();
end

% create the input signal
f0 = 0.1; f1 = 5.0; dur = T(end)/3; A = 1.0;
Tf = 0.2 * dur;
u = zeros(2, N);
fade_fn = @(t) tanh(3*t/Tf).*tanh(-3*(t-dur)/Tf);
exp_chirp = @(ti,w0,w1) sin(w0*ti + 0.0187*(w1-w0)*(dur/4.0*exp(4.0/dur*ti) - ti));
for i=1:N
    w0 = f0 * 2*pi;
    w1 = f1 * 2*pi;

    if (T(i) <= dur)
        ti = T(i);
        u(1,i) = A*exp_chirp(ti, w0, w1);
        u(1,i) = u(1,i) * fade_fn(ti);
    elseif (T(i) >= dur && T(i) <= 2*dur)
        ti = T(i) - dur;
        u(2,i) = A*exp_chirp(ti, w0, w1);
        u(2,i) = u(2,i) * fade_fn(ti);
    end
end


% create the sequence of "measured" states
q0 = x0;
z0 = h(q0, u(:,1), theta_true, T(1));
z = zeros(size(z0,1), length(T));
z(:,1) = z0;
for i=1:length(T)-1
    u_i = 0.5*(u(:,i) + u(:,i+1));
    % first create the dynamic states
    [~,q] = ode45(@(t,x) f(x, u_i, theta_true, t), [T(i) T(i+1)], q0);
    q = q';
    q0 = q(:,end);
    % then apply the output equation
    z(:,i+1) = h(q(:,end), u_i, theta_true, T(i));
end
% add some noise
z = z + 0.01*[max(z(1,:)), max(z(2,:))]*randn(2, N);
% add some bias
z = z + bias;

%%
% create the output error instance
output_error = OutputError(f, h, x0, T, u, z);
output_error = output_error.set_output_to_state_matrix([1 0; 0 1]);
output_error = output_error.set_jacobians(Jfx, Jhx, Jft, Jht);
param_estimates = [0.0  1.e3;
                   0.0  1.e3;
                   0.0  1.e3;
                   0.0  1.e3;
                   0.0  1.e3;
                   0.0  1.e3];
output_error = output_error.set_known_parameter_estimates(param_estimates(:,1), param_estimates(:,2));
[theta,M,R] = output_error.estimate_parameters(param_estimates(:,1));

% print results
fprintf('%9s  %5s  %10s  %5s\n', 'Parameter', 'Value', 'Covariance', 'True');
cov = diag(inv(M));
for i=1:length(theta_true)
    fprintf('%9i  %5.2f  %10.2f  %5.2f\n', i, theta(i), cov(i), theta_true(i));
end
