%
% Generates a set of CSV files with the same structure as ArduPilot logs
% using a set model with noise applied to both the 'measurements' and
% sampling frequency
%

%% Setup
% sample times
dt = 0.1;
t = (0:dt:40)';
for i=1:length(t)
    t(i) = t(i) + 0.1*dt*(2*rand() - 1);
end

% constants
g = 9.81;
S = 39;
b = 19.8;
m = 4500;
Ix = 22370;
Iy = 36060;
Iz = 53960;
Ixz = 1630;
gamma = Ix * Iz - Ixz^2;

% cruise conditions
V0 = 62;
alpha0 = 1 * pi/180;
theta0 = 2 * pi/180;

% derived constants
q0bar = 0.5*1.225*V0^2;
k1 = q0bar * S / m / V0;
k2 = b / 2 / V0;
k3 = q0bar * S * b;
k4 = q0bar * S / m / g;
c3 = Iz / gamma;
c4 = Ixz / gamma;
c9 = Ix / gamma;

% parameters: theta
C_Y_beta = -8.66e-1;
C_Y_r = 9.31e-1;
C_Y_dr = 3.75e-1;
b_betadot = 1e-2;
C_l_beta = -1.19e-1;
C_l_p = -5.84e-1;
C_l_r = 1.88e-1;
C_l_da = -2.28e-1;
C_l_dr = 3.84e-2;
b_pdot = 2e-2; % prime
C_n_beta = 8.65e-2;
C_n_p = -6.39e-2;
C_n_r = -1.92e-1;
C_n_da = -2.73e-3;
C_n_dr = -1.36e-1;
b_rdot = 3e-2; % prime
b_phidot = 4e-2; 
b_a_y = 5e-2;

% state equation: dx/dt = f(x(t), u(t), theta, t)
% x : [beta p r phi]'
% u : [da dr 1]'
% q = [x; u]
A = [k1 * C_Y_beta, sin(alpha0), k1 * C_Y_r * k2 - cos(alpha0), g * cos(theta0) / V0;
     k3 * (c3 * C_l_beta + c4 * C_n_beta), k3 * b / 2 / V0 * (c3 * C_l_p + c4 * C_n_p), ...
        k3 * b / 2 / V0 * (c3 * C_l_r + c4 * C_n_r), 0;
     k3 * (c4 * C_l_beta + c9 * C_n_beta), k3 * b / 2 / V0 * (c4 * C_l_p + c9 * C_n_p), ...
        k3 * b / 2 / V0 * (c4 * C_l_r + c9 * C_n_r), 0;
     0, 1, tan(theta0), 0];
B = [0, k1 * C_Y_dr, b_betadot;
     k3 * (c3 * C_l_da + c4 * C_n_da), k3 * (c3 * C_l_dr + c4 * C_n_dr), b_pdot;
     k3 * (c4 * C_l_da + c9 * C_n_da), k3 * (c4 * C_l_dr + c9 * C_n_dr), b_rdot;
     0, 0, b_phidot];
f = @(t,q) [A, B;
            zeros(3,7)] * q;

 % output equation: y = h(x(t), u(t), theta, t)
 C = [1, 0, 0, 0;
      0, 1, 0, 0;
      0, 0, 1, 0;
      0, 0, 0, 1;
      k4 * C_Y_beta, 0, k4 * C_Y_r * k2, 0];
D = [0, 0, 0;
     0, 0, 0;
     0, 0, 0;
     0, 0, 0;
     0, k4 * C_Y_dr, b_a_y];
 h = @(t,q) [C, D;
             zeros(5,7)] * q;
  
% control input sequence
% doublet on aileron and rudder
inp_mag = 5 * pi/180;
inp_dur_count = 40;
inp_delay_count = 5;
u = ones(3, length(t));
for i=1:length(t)
    % aileron doublet
    if t(i) > inp_delay_count * dt && t(i) < (inp_delay_count + inp_dur_count) * dt
        u(1,i) = inp_mag;
    elseif t(i) >= (inp_delay_count + inp_dur_count) * dt && t(i) < (inp_delay_count + 2*inp_dur_count) * dt
        u(1,i) = -inp_mag;
    else
        u(1,i) = 0;
    end
    
    % aileron doublet
    if t(i) > (inp_delay_count + 2*inp_dur_count) * dt && t(i) < (inp_delay_count + 3*inp_dur_count) * dt
        u(2,i) = inp_mag;
    elseif t(i) >= (inp_delay_count + 3*inp_dur_count) * dt && t(i) < (inp_delay_count + 4*inp_dur_count) * dt
        u(2,i) = -inp_mag;
    else
        u(2,i) = 0;
    end
end

% initial condition 
% x : [beta p r phi]'
x = zeros(4, length(t));
x(:,1) = [0; 0; 0; 0];
% y : [beta p r phi a_y]'
y = zeros(5, length(t));
q0 = h(0, [x(:,1); u(:,1)]);
y(:,1) = q0(1:5);

%% Propagate
for i=1:length(t)-1
    q = [x(:,i); 
         u(:,i)];
    [tout,qout] = ode45(f, [t(i) t(i+1)], q);
    qout = qout';
    x(:,i+1) = qout(1:4,end);
    q = h(t(i), [x(:,i+1); u(:,i+1)]);
    y(:,i+1) = q(1:5);
end

%% Create the CSV files
z = zeros(length(t), 1);

% IMU - gyro and accel measurements
T = array2table([t, z, y(2,:)', z, y(3,:)', z, y(4,:)', zeros(length(t), 8)]);
T.Properties.VariableNames(1:15) = {'timestamp', 'TimeUS', 'GyrX', 'GyrY', ...
    'GyrZ', 'AccX', 'AccY', 'AccZ', 'EG', 'EA', 'T', 'GH', 'AH', 'GHz', 'AHz'};
writetable(T, 'log_data_imu_test.csv');

% ARSP - airspeed measurements
T = array2table([t, z, V0*ones(length(t),1), z, z, z, z, z, z, z]);
T.Properties.VariableNames(1:10) = {'timestamp', 'TimeUS', 'Airspeed', ...
    'DiffPress', 'Temp', 'RawPress', 'Offset', 'U', 'Health', 'Primary'};
writetable(T, 'log_data_arsp_test.csv');