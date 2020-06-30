%% Import data and process
ds = DataStore('support', 'Directory', 'logs/');

%%% Constants
% Start time and duration
% t0 = 181; % sitl
t0 = 494.7; % support
% t0 = 382.96; % mar2
T = 200;
t1 = t0 + T;
% Resampling and lowpass cutoff
Fs = 50;
Fcutoff = 0.5;
% IMU position w.r.t. CG in body axes
x_imu = -0.17; % Guess; CG @ 0.15, Pixhawk slightly forward of TE
y_imu = 0.0;
z_imu = 0.0;

ds = ds.process_log_data(Fs, Fcutoff);
ds = ds.generate_processed_data_subdomain(t0, t1);
ds = ds.smooth_processed_data();
ds = ds.compute_numerical_derivatives();
ds = ds.correct_accel_data_for_sensor_position(x_imu, y_imu, z_imu);
spd = ds.spd;


%% Define the model
%%% Constants
g = 9.77;%9.80665;

%%% State
syms u v w h phi theta psi
state = [u; v; w; h; phi; theta; psi];

%%% Control (sensor measurements)
syms ax ay az p q r Pb
control = [ax; ay; az; p; q; r; Pb];

%%% Output
syms V alpha beta h phi theta psi
output = [V; alpha; beta; h; phi; theta; psi];

%%% Parameters
% output state biases
% NB : psi, h biases omitted since they are arbitrary
syms bias_V bias_alpha bias_beta_f bias_phi bias_theta
% sensor biases
syms bias_ax bias_ay bias_az bias_p bias_q bias_r
% output state scaling factors
% NB : beta_f - this is the flank angle, not purely sideslip
syms scale_V scale_alpha scale_beta_f scale_phi scale_theta scale_psi scale_h
% parameter vector
zeta = [bias_V; bias_alpha; bias_beta_f; bias_phi; bias_theta; 
        bias_ax; bias_ay; bias_az; bias_p; bias_q; bias_r;
        scale_V; scale_alpha; scale_beta_f; scale_phi; scale_theta; scale_psi; scale_h];
    
%%% Auxiliary Measured States
aux = sym([]);

%%% State equation function, f(x, u, zeta)
% x = [u; v; w; h; phi; theta; psi]
f_fn = [(r - bias_r)*v - (q - bias_q)*w - g*sin(theta) + ax - bias_ax;
     -(r - bias_r)*u + (p - bias_p)*w + g*cos(theta)*sin(phi) + ay - bias_ay;
     (q - bias_q)*u - (p - bias_p)*v + g*cos(theta)*cos(phi) + az - bias_az;
     sin(theta)*u - cos(theta)*sin(phi)*v - cos(theta)*cos(phi)*w;
     (p - bias_p) + tan(theta)*sin(phi)*(q - bias_q) + tan(theta)*cos(phi)*(r - bias_r);
     cos(phi)*(q - bias_q) - sin(phi)*(r - bias_r);
     sin(phi)/cos(theta)*(q - bias_q) + cos(phi)/cos(theta)*(r - bias_r)];

%%% Output equation function, h(x)
% y = [V; alpha; beta; h; phi; theta; psi]
% NB: 3rd equation converts from flank angle to sideslip
h_fn = [(1 + scale_V)*sqrt(u^2 + v^2 + w^2) + bias_V;
     (1 + scale_alpha)*atan2(w,u) + bias_alpha;
     tan(atan(((1 + scale_beta_f)*atan2(v,u) + bias_beta_f) * cos((1 + scale_alpha)*atan2(w,u) + bias_alpha)));
     (1 + scale_h)*h;
     (1 + scale_phi)*phi + bias_phi;
     (1 + scale_theta)*theta + bias_theta;
     mod((1 + scale_psi)*psi, 2*pi)];
    
%%% Jacobians
df_dx = jacobian(f_fn, state);
dh_dx = jacobian(h_fn, state);
df_dzeta = jacobian(f_fn, zeta);
dh_dzeta = jacobian(h_fn, zeta);

%%% Convert to MATLAB functions
f_fn = matlabFunction(f_fn, 'vars', {state, control, zeta, aux});
h_fn = matlabFunction(h_fn, 'vars', {state, control, zeta, aux});
df_dx = matlabFunction(df_dx, 'vars', {state, control, zeta, aux});
dh_dx = matlabFunction(dh_dx, 'vars', {state, control, zeta, aux});
df_dzeta = matlabFunction(df_dzeta, 'vars', {state, control, zeta, aux});
dh_dzeta = matlabFunction(dh_dzeta, 'vars', {state, control, zeta, aux});


%% Estimate the sensor bias and scaling factors
% Initial condition
x0 = [spd.velocity_x(1);
      spd.velocity_y(1);
      spd.velocity_z(1);
      spd.height(1);
      spd.roll(1);
      spd.pitch(1);
      spd.yaw(1)];

% Control inputs (sensor measurements)
u = [spd.acceleration_x, ...
     spd.acceleration_y, ...
     spd.acceleration_z, ...
     spd.angular_velocity_x, ...
     spd.angular_velocity_y, ...
     spd.angular_velocity_z]';
 
% Measured data
z = [spd.airspeed, ...
     spd.angle_of_attack, ...
     spd.sideslip, ...
     spd.height, ...
     spd.roll, ...
     spd.pitch, ...
     spd.yaw]';
 
% Initial parameter estimates
zeta0 = zeros(length(zeta), 1);
% use gyro bias estimates provided by the EKF
zeta0(find(zeta==bias_p)) = mean(ds.log.nkf1.gyro_bias_x);
zeta0(find(zeta==bias_q)) = mean(ds.log.nkf1.gyro_bias_y);
zeta0(find(zeta==bias_r)) = mean(ds.log.nkf1.gyro_bias_z);
zeta0 = param; % CHECK

% Auxiliary data
w = zeros(1, length(spd.time));
  
oe = OutputError(f_fn, h_fn, x0, spd.time, u, z, w);
% oe = oe.set_jacobians(df_dx, dh_dx, df_dzeta, dh_dzeta);
% oe = oe.set_output_to_state_matrix([zeros(3,3) zeros(3,4);
%                                     zeros(4,3) eye(4,4)]);
oe = oe.set_parameter_names(arrayfun(@char, zeta, 'uniform', 0));
oe = oe.set_output_names(arrayfun(@char, output, 'uniform', 0));

[param,M,R] = oe.estimate_parameters(zeta0);


%% Print results
fprintf('<strong>%20s  %8s  %10s</strong>\n', 'Parameter', 'Value', 'Covariance');
if exist('M','var')
    cov = diag(inv(M));
else
    cov = zeros(1, length(zeta));
end
for i=1:length(zeta)
    fprintf('%20s  %8.1e  %10.2e\n', char(zeta(i)), param(i), cov(i));
end

%% Airspeed MSD

% Y = oe.compute_output_sequence(param);
% 
% plot(spd.time, spd.airspeed, 'k.');
% hold on;
% plot(spd.time, Y(1,:), 'LineWidth', 2);
% 
% t = spd.time;
% Vhat = Y(1,:)';
% V = spd.airspeed;
% z = V - Vhat;
% 
% syms Pb Ib
% Kb = [Pb; Ib];
% Xb = [spd.battery_power, ...
%       spd.battery_current];
% 
% ms = ModelStructure(z, Kb, Xb);
% [A,zeta,cov,corr,sigma,R2,ms] = ms.stepwise_regression();
% z_fn = matlabFunction(transpose(A)*zeta, 'vars', {Kb});
% Vmsd = Vhat + z_fn(Xb')';
% 
% plot(spd.time, spd.airspeed, 'k-');
% hold on;
% plot(spd.time, Vhat);
% plot(spd.time, Vmsd);
% legend('Measured', 'Kinematic', 'With Extra Terms');