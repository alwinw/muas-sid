% define the state and output equations
f = @(x,u,theta,t) [-theta(1) theta(4); 0 -theta(2)]*x + [5 theta(3); 0 5]*u + theta(5:end);
h = @(x,u,theta,t) [1 0; 0 1]*x + [0 0; 0 0]*u ;

% setup
bias = [0.02; -0.06];
theta_true = [5;10;2;10; bias];
theta0 = ones(length(theta_true), 1);
theta0(5:end) = [0;0];
x0 = [1;2];
dT = 1/10;
T = (0:dT:10)';
N = length(T);

% add some variance to the sampling time
for i=1:length(T)
    T(i) = T(i) + 0.1*dT*randn();
end

% create the input signal
f0 = 0.1; f1 = 5.0; dur = T(end)/3; A = 0.1;
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

%% create the output error instance
output_error = OutputError(f, h, x0, T, u, z);
[theta,M,R] = output_error.estimate_parameters(theta0);

% n = length(theta0);
% Aeq = zeros(n,n);
% beq = zeros(n,1);
% %      CL0, CL_alpha, CD0, CD_alpha, Cm0, Cm_V, Cm_alpha, Cm_delta_e
% %      T0, bias_dot_d_V, bias_dot_d_alpha, bias_dot_d_q, bias_dot_d_theta  
% lb = [-100 -100 -100 -100];
% lb = [lb -Inf*ones(1,length(bias))];
% ub = [+100 +100 +100 +100];
% ub = [ub +Inf*ones(1,length(bias))];
% options = optimoptions('fmincon', 'Display', 'iter');
% p = fmincon(@output_error.compute_cost_function_direct, theta0, [], [], ...
%         Aeq, beq, lb, ub, [], options);
    
%% print results
fprintf('%9s  %5s  %10s\n', 'Parameter', 'Value', 'Covariance');
if exist('M','var')
    cov = diag(inv(M));
else
    cov = zeros(length(theta0),1);
end
for i=1:length(theta_true)
    fprintf('%9i  %5.2f  %10.2e\n', i, p(i), cov(i));
end