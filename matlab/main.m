% Dependencies: pymavlink
% Installation instructions: https://github.com/ArduPilot/pymavlink
% 
% Move a .BIN file into this directory, then run
% generateCsvFilesFromBin(<YOURFILENAME>.bin)
% This will create all the .CSV files you need for the section below

%% Read in data
binName = 'support'; % select a name to identify this CSV
IMU = getLogData(sprintf('imu_data_%s.csv', binName));
NKF1 = getLogData(sprintf('nkf1_data_%s.csv', binName));
NKF2 = getLogData(sprintf('nkf2_data_%s.csv', binName));
AOA = getLogData(sprintf('aoa_data_%s.csv', binName));
ARSP = getLogData(sprintf('arsp_data_%s.csv', binName));
BAT = getLogData(sprintf('bat_data_%s.csv', binName));
RCOU = getLogData(sprintf('rcou_data_%s.csv', binName));

%% Initialise the motor data interpolator
motor_interp = MotorInterpolator(0.020, 1.5, 320, 20*0.0254, 15/20);

%% Setup
t0 = 1537931333; % set this to the global time you want to start at
t1 = t0 + 200;   % use this to select a set duration
dt = 1/400;      % higher rates (lower values) require more time
interpMethod = 'cubicinterp';

% Fit over prescribed time range
V =      fit(ARSP.timestamp, ARSP.Airspeed, interpMethod);
phi =    fit(NKF1.timestamp, NKF1.Roll * pi/180, interpMethod);
theta =  fit(NKF1.timestamp, NKF1.Pitch * pi/180, interpMethod);
psi =    fit(NKF1.timestamp, NKF1.Yaw * pi/180, interpMethod);
alpha =  fit(AOA.timestamp, AOA.AOA * pi/180, interpMethod);
beta =   fit(AOA.timestamp, AOA.SSA * pi/180, interpMethod);
p =      fit(IMU.timestamp, IMU.GyrX, interpMethod);
q =      fit(IMU.timestamp, IMU.GyrY, interpMethod);
r =      fit(IMU.timestamp, IMU.GyrZ, interpMethod);
ax =     fit(IMU.timestamp, IMU.AccX, interpMethod);
az =     fit(IMU.timestamp, IMU.AccZ, interpMethod);
thr =    fit(RCOU.timestamp, (RCOU.C3-1000)/1000, 'linearinterp');
V_batt = fit(BAT.timestamp, BAT.Volt, interpMethod);
I_batt = fit(BAT.timestamp, BAT.Curr, interpMethod);

m = 10;
S = 0.6078;

%% Computation
i = 1;
imax = length(t0:dt:t1);
CL = zeros(imax, 1);
CD = zeros(imax, 1);

for t=t0:dt:t1
    if (mod(i, min(1e3, floor(0.10*imax))) == 0)
        fprintf('%.1f%%\n', 100*i/imax);
    end
    
    V_t = V(t);
    
    phi_t = phi(t);
    theta_t = theta(t);
    psi_t = psi(t);
    
    alpha_t = alpha(t);
    beta_t = beta(t);
    
    u_t = V_t * cos(alpha_t) * cos(beta_t); 
    v_t = V_t * sin(beta_t);
    w_t = V_t * sin(alpha_t) * cos(beta_t);
    
    p_t = p(t);
    q_t = q(t);
    r_t = r(t);
    
    ax_t = ax(t);
    az_t = az(t);
    
    thr_t = thr(t);
    V_batt_t = V_batt(t);
    I_batt_t = I_batt(t);
    T = motor_interp.getThrust(V_t, 1.225, thr_t * V_batt_t, I_batt_t);
    
    CX = computeCX(m, V_t, v_t, w_t, S, theta_t, q_t, r_t, T, ax_t);
    CZ = computeCZ(m, V_t, u_t, v_t, S, theta_t, phi_t, p_t, q_t, az_t);
    [CL(i), CD(i)] = computeCLCD(CX, CZ, alpha_t);
    
    i = i + 1;
end

figure(1); clf; hold on; grid on;
scatter(CL, CD, 10);
xlabel('C_L'); 
ylabel('C_D');

%% Misc. plots
figure(2); clf;

ax1 = subplot(231);
plot(NKF1.timestamp - NKF1.timestamp(1), NKF1.Roll, '-o', 'MarkerSize', 2);
grid on; xlabel('Time [s]'); ylabel('Roll [deg]');

ax2 = subplot(232);
plot(NKF1.timestamp - NKF1.timestamp(1), NKF1.Pitch, '-o', 'MarkerSize', 2);
grid on; xlabel('Time [s]'); ylabel('Pitch [deg]');

ax3 = subplot(233);
plot(AOA.timestamp - AOA.timestamp(1), AOA.SSA, '-o', 'MarkerSize', 2);
grid on; xlabel('Time [s]'); ylabel('Sideslip [deg]');

ax4 = subplot(234); grid on;
plot(RCOU.timestamp - RCOU.timestamp(1), (RCOU.C1-1500)/1000, '-o', 'MarkerSize', 2);
grid on; xlabel('Time [s]'); ylabel('Aileron, \delta_a \in [-1,1]');

ax5 = subplot(235); grid on;
plot(RCOU.timestamp - RCOU.timestamp(1), (RCOU.C2-1500)/1000, '-o', 'MarkerSize', 2);
grid on; xlabel('Time [s]'); ylabel('Elevator, \delta_e \in [0,1]');

ax6 = subplot(236); grid on;
plot(RCOU.timestamp - RCOU.timestamp(1), thr(RCOU.timestamp), '-o', 'MarkerSize', 2);
grid on; xlabel('Time [s]'); ylabel('Throttle, \delta_t \in [0,1]');
linkaxes([ax1 ax2 ax3 ax4 ax5 ax6], 'x')