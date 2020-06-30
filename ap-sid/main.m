% APSID
% ArduPilot System Identification 
%

%% User parameters
% [Hz]; should be significantly larger than any vehicle dynamic mode
params.filter_cutoff_freq = 20; % <- maybe ignore short-period mode and let OE handle it?
% [Hz]; higher frequency data decimated, lower frequency data interpolated
params.resample_freq = 50;

% params.time_start = 181; % sitl
params.time_start = 494.7; % support
% 494.7, 713
% params.time_start = 382.96; % mar2
params.duration = 50;%30;


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
%         log_data.(fnames_1{i}).(fnames_2{j}) = bandstop(log_data.(fnames_1{i}).(fnames_2{j}), ...
%                                                [0.5, 14.5], ...
%                                                Fs);
        log_data.(fnames_1{i}).(fnames_2{j}) = lowpass(log_data.(fnames_1{i}).(fnames_2{j}), ...
                                               0.5, ...
                                               Fs);
    end
    log_data.(fnames_1{i}).time = T;
end
all_resampled_data = resample_log_data(log_data, params.resample_freq);
fprintf('(%.2f s)\n', toc);

%% Only examine a limited time series for speed
% modelling should only be applied during cruise, not during takeoff and
% landing
resampled_data = filter_struct_data_by_time(all_resampled_data, ...
    params.time_start, params.time_start + params.duration);

%% Smoothing and numerical derivatives
fprintf('Smoothing and computing numerical derivatives...'); tic;
% Smooth
resampled_data = smooth_resampled_data(resampled_data);
% Numerical derivatives of smoothed data
[dot_p, dot_q, dot_r, dot_alpha, dot_beta] = compute_numerical_derivatives(...
                            resampled_data.time, ...
                            resampled_data.angular_velocity_x, ...
                            resampled_data.angular_velocity_y, ...
                            resampled_data.angular_velocity_z, ...
                            resampled_data.angle_of_attack, ...
                            resampled_data.sideslip);
resampled_data.angular_acceleration_x = dot_p;
resampled_data.angular_acceleration_y = dot_q;
resampled_data.angular_acceleration_z = dot_r;
resampled_data.angle_of_attack_rate = dot_alpha;
resampled_data.sideslip_rate = dot_beta;
fprintf('(%.2f s)\n', toc);

%% Compute nondimensional aerodynamic force and moment coefficients
fprintf('Preparing model...'); tic;
%%% symbolic variables
syms V
syms p q r
syms dot_p dot_q dot_r
syms ax ay az
syms alpha
syms Pb

% define a state vector of symbols and measured values for concise
% substitution code
x = [V;
     p; q; r; 
     dot_p; dot_q; dot_r; 
     ax; ay; az; 
     alpha;
     Pb];
x_vals = [resampled_data.airspeed, ...
          resampled_data.angular_velocity_x, ...
          resampled_data.angular_velocity_y, ...
          resampled_data.angular_velocity_z, ...
          resampled_data.angular_acceleration_x, ...
          resampled_data.angular_acceleration_y, ...
          resampled_data.angular_acceleration_z, ...
          resampled_data.body_acceleration_x, ...
          resampled_data.body_acceleration_y, ...
          resampled_data.body_acceleration_z, ...
          resampled_data.angle_of_attack, ...
          resampled_data.battery_power]';

%%% shared constants
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

%%% Derived quantities
qbar = 0.5*rho*V^2; % NB: This does not use the mean airspeed

%%% Nondimensional coefficients
% first define symbolically
CX = 1/qbar/S * (m*ax - T);
CY = 1/qbar/S * m*ay;
CZ = 1/qbar/S* m*az;
CL = CX*sin(alpha) - CZ*cos(alpha);
CD = -CX*cos(alpha) - CZ*sin(alpha);
Cl = 1/qbar/S/b * (Ix*dot_p - Ixz*(p*q + dot_r) + (Iz - Iy)*q*r);
Cm = 1/qbar/S/cbar * (Iy*dot_q + (Ix - Iz)*p*r + Ixz*(p^2 - r^2));
Cn = 1/qbar/S/b * (Iz*dot_r - Ixz*(dot_p - q*r) + (Iy - Ix)*p*q);
% then convert to MATLAB functions
CX = matlabFunction(CX, 'vars', {x});
CY = matlabFunction(CY, 'vars', {x});
CZ = matlabFunction(CZ, 'vars', {x});
CL = matlabFunction(CL, 'vars', {x});
CD = matlabFunction(CD, 'vars', {x});
Cl = matlabFunction(Cl, 'vars', {x});
Cm = matlabFunction(Cm, 'vars', {x});
Cn = matlabFunction(Cn, 'vars', {x});
% then substitute measured data
z = [CL(x_vals);
     CD(x_vals);
     Cm(x_vals)]';
t = resampled_data.time;
z_names = {'C_L', 'C_D', 'C_m'};

% define the measured base regressors
syms V alpha beta
syms p q r
syms delta_a delta_e delta_r
syms dot_alpha dot_beta
syms Pb
% These variables must be first-order only
Kb = [V;
      alpha;
      beta;
      p;
      q;
      r;
      delta_a;
      delta_e;
      delta_r;
      dot_alpha;
      dot_beta];
  
% define the measurements for the base regressors
Xb = [resampled_data.airspeed, ...
      resampled_data.angle_of_attack, ...
      resampled_data.sideslip, ...
      resampled_data.angular_velocity_x, ...
      resampled_data.angular_velocity_y, ...
      resampled_data.angular_velocity_z, ...
      resampled_data.aileron, ...
      resampled_data.elevator, ...
      resampled_data.rudder, ...
      resampled_data.angle_of_attack_rate, ...
      resampled_data.sideslip_rate];
assert(size(Kb,1) == size(Xb,2), 'Kb,Xb inconsistent');
  
fprintf('(%.2f s)\n', toc);

%% Model structure determination

model = sym(zeros(size(z,2), 1));
variance = cell(size(z,2), 1);
correlation = cell(size(z,2), 1);
s = cell(size(z,2), 1);
coeff_of_det = cell(size(z,2), 1);
param_est = cell(size(z,2), 1);
regressors = cell(size(z,2), 1);
for i=1:size(z,2)
    fprintf('<strong>Model %i</strong>\n', i);
    ms = ModelStructure(z(:,i), Kb, 0, Xb, 0.99, 0.85);
    [A, zeta, cov, corr, sigma, R2, ms] = ms.stepwise_regression();
    model(i) = transpose(A)*zeta;
    regressors{i} = A;
    param_est{i} = zeta;
    variance{i} = cov;
    correlation{i} = corr;
    s{i} = sigma;
    coeff_of_det{i} = R2;
    
    fprintf('R^2 = %.3f\n', R2);
    fprintf('%s\n', char(vpa(model(i), 3)));
end

% Plot the results
figure(1); clf;
axH = zeros(1, size(z,2));
for i=1:size(z,2)
    yhat_fn = matlabFunction(model(i), 'vars', {Kb});
    yhat = yhat_fn(Xb')';
    nu = z(:,i) - yhat;
    
    axH(i) = subplot(3,3,i);
    plot(t, z(:,i), 'k.-');
    hold on; grid on;
    plot(t, yhat, 'LineWidth', 2);
    title(sprintf('%s (R^2=%.3f)', z_names{i}, coeff_of_det{i}));
    
    axH(i+size(z,2)) = subplot(3,3,i+size(z,2));
    plot(t, nu, 'k-');
    hold on; grid on;
    plot([t(1) t(end)], 2*s{i}*[1 1], 'r-.');
    plot([t(1) t(end)], 2*s{i}*[-1 -1], 'r-.');
    title(sprintf('Residual: \\nu_{%s} (2\\sigma=%.1e)', z_names{i}, 2*s{i}));
    
    subplot(3,3,i+2*size(z,2));
    plot(yhat, nu, 'k.');
    hold on; grid on;
    plot([min(yhat) max(yhat)], 2*s{i}*[1 1], 'r-.');
    plot([min(yhat) max(yhat)], 2*s{i}*[-1 -1], 'r-.');
    ylabel(['\nu_{' z_names{i} '}']);
    xlabel(z_names{i});
end
linkaxes(axH, 'x');


for i=1:size(z,2)
    yhat_fn = matlabFunction(model(i), 'vars', {Kb});
    yhat = yhat_fn(Xb')';
    nu = z(:,i) - yhat;
    
    figure(1 + i); clf;
    % residual autocorrelation
    Ns = size(z,1);
    Rvv_fn = @(k) 1/Ns * (nu(1:Ns-k)' * nu(1+k:Ns));
    Rvv = arrayfun(Rvv_fn, 1:Ns);
    
    % estimate of the residual autocorrelation standard deviation
    s_Rvv = Rvv_fn(0) / sqrt(Ns);
    
    sgtitle(sprintf('%s Residual Analysis', char(z_names{i})))
    subplot(121);
    plot(-(Ns-1):Ns-1,  [fliplr(Rvv(1:end-1)) Rvv]);
    grid on; hold on;
    plot([-(Ns-1) Ns-1], 2*s_Rvv*[1 1], 'r-.');
    plot([-(Ns-1) Ns-1], 2*s_Rvv*[-1 -1], 'r-.');
    title('Residual Autocorrelation');
    ylabel('R_{\nu\nu}');
    xlabel('Lag Index');
    
    % compute the normally distributed function to check whether the
    % residuals are Gaussian ~ N(0,1)
    phi = @(z) icdf('norm', z, 0, 1);
    P = @(i) (i-0.5)/Ns;
    phi_inv = (phi(P(1:Ns)));
    
    subplot(122);
    plot(sort(nu), phi_inv);
    grid on;
    xlabel(['\nu_{' z_names{i} '}']);
    ylabel('\Phi^{-1}(P)');
    title('Relationship to Gaussian Residuals');
end

%% Print the parameters
for i=1:size(z,2)
    N = max([max(strlength(arrayfun( @char, regressors{i}, 'UniformOutput', false))), ...
             strlength('Variable')]);
    
    fprintf('<strong>%s Model</strong>\n', z_names{i});
    fprintf(sprintf('%s%is  %s  %s  %s\\n', '%', N, '%9s', '%8s', '%8s'), ...
        'Variable', 'Value', 'Variance', '|t_0|');
    for j=1:length(regressors{i})
        fprintf(sprintf('%s%is  %s  %s  %s\\n', '%', N, '%9.2e', '%8.1e', '%8.1e'), ...
            regressors{i}(j), param_est{i}(j), ...
            variance{i}(j), abs(param_est{i}(j) / variance{i}(j)));
    end
end

