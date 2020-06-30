% APSID
% ArduPilot System Identification 
%

%% User parameters
% [Hz]; should be significantly larger than any vehicle dynamic mode
params.filter_cutoff_freq = 0.5; 
% [Hz]; higher frequency data decimated, lower frequency data interpolated
params.resample_freq = 50;

% params.time_start = 181; % sitl
params.time_start = 494.7; % support
% 494.7, 713
% params.time_start = 382.96; % mar2
params.max_duration = 50;
params.min_duration = 50;
params.duration_step = 10;


%% Load log data from CSV files
% These files can be generated using the mavlogdump tool from pymavlink
% e.g., 
% mavlogdump.py --format csv --types IMU support.bin > support_IMU.csv
%
log_name = 'support';
fprintf('Importing log files: logs/%s_*.csv\n', log_name);
fprintf('Importing log data...'); tic;
log_files = cell(7,1);
log_types = {'IMU', 'CTUN', 'NKF1', 'NKF2', 'AOA', 'RCOU', 'BAT'};
for i=1:length(log_files)
    log_files{i} = sprintf('logs/%s_%s.csv', log_name, log_types{i});
end
log_data = import_log_data(log_files{1}, log_files{2}, log_files{3}, ...
                           log_files{4}, log_files{5}, log_files{6}, ...
                           log_files{7});
fprintf('(%.2f s)\n', toc);
                       
%% Resample to a common frequency and filter non-physical frequencies
fprintf('Filtering and resampling...'); tic;
fnames_1 = fieldnames(log_data);
for i=1:length(fnames_1)
    fnames_2 = fieldnames(log_data.(fnames_1{i}));
    for j=1:length(fnames_2) 
        if strcmp(fnames_2{j}, 'time')
            % don't filter the time data
            continue
        end
        
        % resample to a common frequency
        [log_data.(fnames_1{i}).(fnames_2{j}), T] = ...
            resample(log_data.(fnames_1{i}).(fnames_2{j}), ...
            log_data.(fnames_1{i}).time, params.resample_freq);
        
        % sampling frequency
        Fs = params.resample_freq;
        
        % filter everything except the low frequency dynamics, and the
        % short period mode
        log_data.(fnames_1{i}).(fnames_2{j}) = lowpass(log_data.(fnames_1{i}).(fnames_2{j}), ...
                                               params.filter_cutoff_freq, ...
                                               Fs);
    end
    log_data.(fnames_1{i}).time = T;
end
all_resampled_data = resample_log_data(log_data, params.resample_freq);
fprintf('(%.2f s)\n', toc);

%% Only examine a limited time series for speed
% modelling should only be applied during cruise, not during takeoff and
% landing
all_resampled_data = filter_struct_data_by_time(all_resampled_data, ...
    params.time_start, params.time_start + params.max_duration);

resampled_data = all_resampled_data;

%% Smoothing and numerical derivatives
fprintf('Smoothing and computing numerical derivatives...'); tic;
% Smooth
resampled_data = smooth_resampled_data(resampled_data);
% Numerical derivatives of smoothed data
t = resampled_data.time;
x = {resampled_data.angular_velocity_x, resampled_data.angular_velocity_y, ...
     resampled_data.angular_velocity_z, resampled_data.angle_of_attack, ...
     resampled_data.sideslip};
dxdt = cell(length(x), 1);
parfor i=1:length(x)
    dxdt{i} = compute_numerical_derivatives(t, x{i});
end
resampled_data.angular_acceleration_x = dxdt{1};
resampled_data.angular_acceleration_y = dxdt{2};
resampled_data.angular_acceleration_z = dxdt{3};
resampled_data.angle_of_attack_rate = dxdt{4};
resampled_data.sideslip_rate = dxdt{5};
fprintf('(%.2f s)\n', toc);

%% Prepare the model

%%% symbolic variables
% state
syms V alpha q theta 
syms V_eq alpha_eq q_eq theta_eq
% output
syms ax az
% control
syms delta_e delta_a delta_r Pb
% auxiliary
syms p r phi beta
syms dot_alpha dot_beta
syms a_y
% model parameters
syms CL0 CL_dot_alpha CL_q CL_V CL_dot_beta CL_r CL_delta_r CL_beta CL_p CL_delta_e
syms CD0 CD_alpha CD_V CD_dot_alpha CD_q CD_delta_e CD_dot_beta CD_r CD_delta_r CD_beta
syms Cm0 Cm_dot_alpha Cm_delta_e Cm_p Cm_r

% shared constants
rho = 1.225;
g = 9.80665;
%%% quadplane
% aircraft properties
S = 0.602;
b = 2.195;
cbar = 0.274;
m = 10.5;
Ix = 0.7879;
Iy = 0.633;
Iz = 1.3837;
Ixz = 0.0291;
% thrust model (R^2=1 for Adam's model)
% expanded about V=25 m/s, Pb=550 W
T0 = 12.71;
T_Pb = 0.02786;
T_V = -0.8139;
T_V2 = 0.000189;
T_VPb = -0.0005075;
T_Pb2 = -5.013e-6;
T = T0 + T_Pb*(Pb-550) + T_V*(V-25) + T_V2*(V-25).^2 + T_VPb*(V-25).*(Pb-550) + T_Pb2*(Pb-550).^2;
fprintf('Selected model: <strong>QUADPLANE</strong>\n');
%%% rascal
% % aircraft properties
% S = 0.982;
% b = 2.795;
% cbar = 0.3505;
% m = 5.8984;
% Ix = 2.6438;
% Iy = 2.1015;
% Iz = 2.5896;
% Ixz = 0;
% % thrust model
% T = 10.0; % value doesn't matter, since it cancels with CD0
% fprintf('Selected model: <strong>RASCAL</strong>\n');

fprintf('Preparing the model...'); tic;
%%% Equilibrium states
% Taylor series expansions about this point 
% x_eq_vals = mean([resampled_data.airspeed, ...
%                   resampled_data.angle_of_attack, ...
%                   resampled_data.angular_velocity_y, ...
%                   resampled_data.pitch], 1)';
x_eq_vals = mean([resampled_data.airspeed, ...
                  resampled_data.angular_velocity_y, ...
                  resampled_data.pitch], 1)';
% equilbrium values that are not state variables
Pb_eq = mean(resampled_data.battery_power);
delta_e_eq = mean(resampled_data.elevator);
delta_a_eq = mean(resampled_data.aileron);
delta_r_eq = mean(resampled_data.rudder);

% define the state, output, control, and auxiliary measurement vectors
x = [V; alpha; theta];%[V; alpha; q; theta];
x_eq = [V_eq; alpha_eq; theta_eq];%[V_eq; alpha_eq; q_eq; theta_eq];
u = [delta_e; delta_a; delta_r; Pb];
y = [V; alpha; theta; ax; az];%[V; alpha; q; theta; ax; az];
w = [p; r; phi; beta; dot_alpha; dot_beta; a_y; q];

%%% Shorthand symbols
% state
d_V = V - V_eq;
d_alpha = alpha - alpha_eq;
d_q = q - q_eq;
d_theta = theta - theta_eq;
% control
d_delta_e = delta_e - delta_e_eq;
d_delta_a = delta_a - delta_a_eq;
d_delta_r = delta_r - delta_r_eq;

%%% Derived quantities
qbar = 0.5*rho*V^2; % NB: This does not use the mean airspeed
c5 = (Iz - Ix) / Iy;
c6 = Ixz / Iy;
c7 = 1 / Iy;

%%% Biases
% dynamically create state biases
biases_x = [];
for i=1:length(x)
    biases_x = [biases_x; sym(['bias_dot_' char(x(i))])];
    syms(['bias_dot_' char(x(i))])
end
% dynamically create output biases
biases_y = [];
for i=1:length(y)
    if any(y(i) == x)
        biases_y = [biases_y; 0];
    else
        biases_y = [biases_y; sym(['bias_' char(y(i))])];
    end
    syms(['bias_' char(y(i))])
end

%%% Model parameters vector
zeta = [CL0; CL_dot_alpha; CL_q; CL_V; CL_dot_beta; CL_r; CL_delta_r; CL_beta; CL_p; CL_delta_e; 
        CD0; CD_alpha; CD_V; CD_dot_alpha; CD_q; CD_delta_e; CD_dot_beta; CD_r; CD_delta_r; CD_beta;
%         Cm0; Cm_dot_alpha; Cm_delta_e; Cm_p; Cm_r;
        biases_x; biases_y(biases_y~=0)];

%%% Aerodynamic model
CL = CL0 + CL_dot_alpha*dot_alpha + CL_q*q + CL_V*V + CL_dot_beta*dot_beta + ...
     CL_r*r + CL_delta_r*delta_r + CL_beta*beta + CL_p*p + CL_delta_e*delta_e;
CD = CD0 + CD_alpha*alpha + CD_V*V + CD_dot_alpha*dot_alpha + CD_q*q + ...
     CD_delta_e*delta_e + CD_dot_beta*dot_beta + CD_r*r + CD_delta_r*delta_r + ...
     CD_beta*beta;
Cm = Cm0 + Cm_dot_alpha*dot_alpha + Cm_delta_e*delta_e + Cm_p*p + Cm_r*r;
CX = CL*sin(alpha) - CD*cos(alpha);
CY = m/qbar/S * a_y;
CZ = -CL*cos(alpha) - CD*sin(alpha);
CDw = CD*cos(beta) - CY*sin(beta);
CYw = CY*cos(beta) + CD*sin(beta);

%%% State equation
% [V; alpha; q; theta]
f = [-qbar*S/m*CDw + T/m*cos(alpha)*cos(beta) + g*(cos(phi)*cos(theta)*sin(alpha)*cos(beta) + ...
        sin(phi)*cos(theta)*sin(beta) - sin(theta)*cos(alpha)*cos(beta));
        
     -qbar*S/m/V/cos(beta)*CL + q - tan(beta)*(p*cos(alpha) + r*sin(alpha)) - ...
        T/m/V*sin(alpha)/cos(beta) + g/V/cos(beta)*(cos(phi)*cos(theta)*cos(alpha) + sin(theta)*sin(alpha));
        
%      c5*p*r - c6*(p^2 - r^2) + c7*qbar*S*cbar*Cm;
     
     q*cos(phi) - r*sin(phi)] + ... 
     biases_x;
% try to reduce complexity after substituting equilibrium values
f = simplify(subs(f, x_eq, x_eq_vals));

%%% Output equation
h = [V;
     alpha;
%      q;
     theta;
     qbar*S/m*CX + T/m;
     qbar*S/m*CZ] + ...
     biases_y;
% try to reduce complexity after substituting equilibrium values
h = simplify(subs(h, x_eq, x_eq_vals));

%%% Jacobians
% symbolic math allows Jacobians to be readily computed
% they can be used to improve the initial parameter estimate
df_dx = jacobian(f,x);
dh_dx = jacobian(h,x);
df_dzeta = jacobian(f,zeta);
dh_dzeta = jacobian(h,zeta);

%%% Convert symbolic functions to numeric
f = matlabFunction(f, 'vars', {x, u, zeta, w});
h = matlabFunction(h, 'vars', {x, u, zeta, w});
df_dx = matlabFunction(df_dx, 'vars', {x, u, zeta, w});
dh_dx = matlabFunction(dh_dx, 'vars', {x, u, zeta, w});
df_dzeta = matlabFunction(df_dzeta, 'vars', {x, u, zeta, w});
dh_dzeta = matlabFunction(dh_dzeta, 'vars', {x, u, zeta, w});

fprintf('(%.2f s)\n', toc);

%% Run the Output Error method
%%% Parameter estimates
% covariance inverses are used as weights for the cost function, so be
% conservative!
%

syms CL0 CL_dot_alpha CL_q CL_V CL_dot_beta CL_r CL_delta_r CL_beta CL_p CL_delta_e
syms CD0 CD_alpha CD_V CD_dot_alpha CD_q CD_delta_e CD_dot_beta CD_r CD_delta_r CD_beta

%                  Variable         Value       Covariance
param_estimates = [CL0              1.74        5.1e-4;
                   CL_dot_alpha    -1.22        1.3e-4;
                   CL_q             1.41        3.8e-4;
                   CL_V            -5.35e-2     1.0e-6;
                   CL_dot_beta     -1.38        1.5e-3;
                   CL_r            -1.36        1.5e-3;
                   CL_delta_r      -8.84        2.2e-1;
                   CL_beta         -1.88e-1     1.4e-4;
                   CL_p             6.48e-02    2.0e-05;
                   CL_delta_e      -3.48e-01    8.2e-03;
                   CD0              1.98e-01    3.7e-05;
                   CD_alpha         6.47e-01    2.7e-05;
                   CD_V            -5.53e-03    7.8e-08;
                   CD_dot_alpha     2.93e-02    8.5e-06;
                   CD_q            -6.92e-02    2.5e-05;
                   CD_delta_e       2.88e-01    5.7e-04;
                   CD_dot_beta      1.27e-01    9.0e-05;
                   CD_r             1.12e-01    8.8e-05;
                   CD_delta_r       1.06e+00    1.4e-02;
                   CD_beta          2.41e-02    1.1e-05];
% increase covariance estimates, since MSD covariance estimates are very
% optimistic
param_estimates(:,3) = 1e4 * param_estimates(:,3);
% add biases to the matrix of parameter estimates
param_estimates = [param_estimates; 
                   biases_x, zeros(length(biases_x),1), 1.e3*ones(length(biases_x),1)];
param_estimates = [param_estimates; 
                   biases_y(biases_y~=0), zeros(length(biases_y(biases_y~=0)),1), 1.e3*ones(length(biases_y(biases_y~=0)),1)];

% check all the model parameters are present and in the correct order
assert(isequaln(param_estimates(:,1), zeta), 'Check the order of parameter estimates')

%%% Parameter estimation
% iteratively increases the time history range to improve robustness
% against model instability (unstable models can cause ODE45 to fail)
iter = 0;
for duration = params.min_duration : params.duration_step : params.max_duration
    
    % reduce the data time range
    rd_curr = filter_struct_data_by_time(resampled_data, ...
        params.time_start, params.time_start + duration);
    
    % prepare the inputs using the measured data
    % this must match what was previous specified!
    t = rd_curr.time;
    x0 = [rd_curr.airspeed(1);
%           rd_curr.angle_of_attack(1);
          rd_curr.angular_velocity_y(1);
          rd_curr.pitch(1)];
    u = [rd_curr.elevator, ...
         rd_curr.aileron, ...
         rd_curr.rudder, ...
         rd_curr.battery_power]';
%     z = [rd_curr.airspeed, ...
%          rd_curr.angle_of_attack, ...
%          rd_curr.angular_velocity_y, ...
%          rd_curr.pitch, ...
%          rd_curr.body_acceleration_x, ...
%          rd_curr.body_acceleration_z]';
    z = [rd_curr.airspeed, ...
         rd_curr.angular_velocity_y, ...
         rd_curr.pitch, ...
         rd_curr.body_acceleration_x, ...
         rd_curr.body_acceleration_z]';
    w = [rd_curr.angular_velocity_x, ...
         rd_curr.angular_velocity_z, ...
         rd_curr.roll, ...
         rd_curr.sideslip, ...
         rd_curr.angle_of_attack_rate, ...
         rd_curr.sideslip_rate, ...
         rd_curr.body_acceleration_y, ...
         rd_curr.angle_of_attack]';
    zeta0 = double(param_estimates(:,2));
    
    % instantiate the Output Error solver
    oe = OutputError(f, h, x0, t, u, z, w);
    
    % Jacobians improve initial parameter estimates, and hence reduce the
    % chance of converging to a local optimum
    oe = oe.set_jacobians(df_dx, dh_dx, df_dzeta, dh_dzeta);
    if duration == params.min_duration
        % only pass the output->state matrix once so that initial parameter
        % guesses only occur on the first iteration
%         oe = oe.set_output_to_state_matrix([eye(3) zeros(3,2)]);
    end
    % give the solver some names for more informative plots
    oe = oe.set_parameter_names(arrayfun(@char, zeta, 'uniform', 0));
    oe = oe.set_output_names(arrayfun(@char, y, 'uniform', 0));
    
    % use initial guesses, or results from the previous parameter
    % estimation run
    oe = oe.set_known_parameter_estimates(...
        double(param_estimates(:,2)), double(param_estimates(:,3)));

    % this is where the magic happens
    [p,M,R] = oe.estimate_parameters(zeta0);
    
    % use the resulting estimates to update the information given to the
    % solver
    param_estimates(:,2) = p;
    % covariance estimates are asymptotic, so increase them to avoid
    % biasing the solver
    cov = diag(inv(M));
%     param_estimates(:,3) = 100 * cov;
    
    % display the parameter estimates after each iteration
    vpa([param_estimates(:,1:2) cov], 3)
    
    iter = iter + 1;
end

%% Print results
fprintf('<strong>%20s  %8s  %10s</strong>\n', 'Parameter', 'Value', 'Covariance');
if exist('M','var')
    cov = diag(inv(M));
else
    cov = zeros(1, length(zeta));
end
for i=1:length(zeta)
    fprintf('%20s  %8.1e  %10.2e\n', char(zeta(i)), p(i), cov(i));
end

%% Compare model to a different set of data
%{
idx = find(all_resampled_data.time >= params.time_start & ...
           all_resampled_data.time <= params.time_start + 5);%params.max_duration + 10);
t = all_resampled_data.time(idx);
u = [all_resampled_data.elevator(idx), ...
     all_resampled_data.aileron(idx), ...
     all_resampled_data.rudder(idx), ...
     all_resampled_data.battery_power(idx)]';
z = [all_resampled_data.airspeed(idx), ...
     all_resampled_data.angle_of_attack(idx), ...
     all_resampled_data.angular_velocity_y(idx), ...
     all_resampled_data.pitch(idx), ...
     all_resampled_data.body_acceleration_x(idx), ...
     all_resampled_data.body_acceleration_z(idx)]';
w = [all_resampled_data.angular_velocity_x(idx), ...
     all_resampled_data.angular_velocity_z(idx), ...
     all_resampled_data.roll(idx), ...
     all_resampled_data.sideslip(idx)]';
 
zeta_est = p;

Y = zeros(size(z,1), length(t));
X0 = z(1:length(x),1);
for i=1:length(t)-1
    % assume constant controls over the local domain
    fn = @(t,x) f(x, u(:,i), zeta_est, w(:,i));
    [~,x] = ode45(fn, [t(i), t(i+1)], X0);
    % store the final state to be used as the next initial
    % condition
    x = x';
    X0 = x(:,end);
    % apply the output equation to get the output state
    Y(:,i) = h(x(:,1), u(:,i), zeta_est, w(:,i));
    Y(:,i+1) = h(x(:,end), u(:,i+1), zeta_est, w(:,i+1));
end

figure(4); clf;
for i=1:length(y)
    subplot(3,3,i);
    plot(t, z(i,:), 'k.-');
    hold on; grid on;
    plot(t, Y(i,:), 'LineWidth', 2)
    title(char(y(i)));
end

figure(5); clf;
for i=1:length(y)
    subplot(3,3,i);
    plot(t, z(i,:)-Y(i,:), 'm-');
    hold on; grid on;
    title(sprintf(['\\nu_%i;' char(y(i))], i));
end
%}