%%% True model
% states
syms alpha V q delta_e
syms alpha_eq V_eq q_eq delta_e_eq
d_alpha = alpha - alpha_eq;
d_V = V - V_eq;
d_q = q - q_eq;
d_delta_e = delta_e - delta_e_eq;
% model parameters
CL0 = 0.40;
CL_alpha = 6.2;
CL_V = -1.2e-1;
CL_q = -3.0e-2;
%
CD0 = 0.05;
CD_alpha = 0.1; % <- too small to be identified
CD_V = 0.7;
CD_q = 0.7;
%
Cm0 = -0.05;
Cm_alpha = -0.1;
Cm_V = 0.2;
Cm_q = -0.06;
Cm_delta_e = 2.9;
% model definitions
CL = CL0 + CL_alpha*alpha + CL_V*V + CL_q*q;
CD = CD0 + CD_alpha*alpha + CD_V*V + CD_q*q;
Cm = Cm0 + Cm_alpha*alpha + Cm_V*V + Cm_q*q + Cm_delta_e*delta_e;

%%% Generate fake measured data
% z: alpha; V; q; delta_e; qdot (approximated)
% time
Fs = 50;
T = 200;
t = 0:1/Fs:T;
% fake measured states
z_vals = [5*pi/180*sin(2*pi*0.1*t) + 2*pi/180*sin(2*pi*0.8*t);
          20 + (sin(2*pi*0.08*t) + sin(2*pi*0.5*t));
          50*pi/180*(sin(2*pi*0.08*t) + sin(2*pi*0.14*t) + 0.1*sin(2*pi*8.8*t));
          0.1*(0.5*sin(2*pi*4.1*t) + sin(2*pi*0.07*t) + 0.1*sin(2*pi*11.2*t))];
z_eq_vals = mean(z_vals, 2);
z_rms_vals = rms(z_vals, 2);
% add noisepl
noise_frac = 0.50;
z_vals = z_vals + noise_frac*(z_rms_vals.*randn(size(z_vals))-0.5*z_rms_vals);
% filter
Fcutoff = 15;
for i=1:size(z_vals,1)
    z_vals(i,:) = lowpass(z_vals(i,:), Fcutoff, Fs);
end
% smoothing
for i=1:size(z_vals,1)
    z_vals(i,:) = smooth(t, z_vals(i,:), 10, 'loess');
end
% numerical differentiation
[~, dot_q, ~, ~, ~] = compute_numerical_derivatives(...
                            t, ...
                            z_vals(1,:), ...
                            z_vals(3,:), ...
                            z_vals(1,:), ...
                            z_vals(1,:), ...
                            z_vals(1,:));
z_vals = [z_vals; 
          dot_q'];
% constants
qbar = 0.5*1.225*20^2;
S = 0.5;
b = 2.0;
cbar = 0.3;
m = 10;
T = 12.0;
% calculate the dimensionless coefficients
syms alpha V q delta_e dot_q
z = [alpha; V; q; delta_e; dot_q];
CL_fn = matlabFunction(subs(CL, [alpha_eq; V_eq; q_eq; delta_e_eq], z_eq_vals(1:4,:)), 'vars', {z});
CD_fn = matlabFunction(subs(CD, [alpha_eq; V_eq; q_eq; delta_e_eq], z_eq_vals(1:4,:)), 'vars', {z});
Cm_fn = matlabFunction(subs(Cm, [alpha_eq; V_eq; q_eq; delta_e_eq], z_eq_vals(1:4,:)), 'vars', {z});
CL = CL_fn(z_vals);
CD = CD_fn(z_vals);
Cm = Cm_fn(z_vals);
CX = CL.*sin(z_vals(1,:)) - CD.*cos(z_vals(1,:));
CZ = -CL.*cos(z_vals(1,:)) - CD.*sin(z_vals(1,:));
% calculate the IMU measurements
ax = 1/m*(qbar*S*CX + T);
az = qbar*S/m*CZ;

%%
%%% MSD
z = [CL; CD; Cm]';
Kb = [alpha; V; q; delta_e; dot_q];
N = 0;
Xb = z_vals';
conf_in = 0.99;
conf_out = 0.95;

model = sym(zeros(size(z,2), 1));
variance = cell(size(z,2), 1);
correlation = cell(size(z,2), 1);
s = cell(size(z,2), 1);
coeff_of_det = cell(size(z,2), 1);
param_est = cell(size(z,2), 1);
regressors = cell(size(z,2), 1);
for i=1:size(z,2)
    fprintf('<strong>Model %i</strong>\n', i);
    ms = ModelStructure(z(:,i), Kb, N, Xb, conf_in, conf_out);
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

%%
% Plot the results
figure(1); clf;
axH = zeros(1, size(z,2));
for i=1:size(z,2)
    yhat_fn = matlabFunction(model(i), 'vars', {Kb});
    yhat = yhat_fn(Xb')';
    nu = z(:,i) - yhat;
    
    axH(i) = subplot(3,size(z,2),i);
    plot(t, z(:,i), 'k.-');
    hold on; grid on;
    plot(t, yhat, 'LineWidth', 2);
    title(sprintf('%s (R^2=%.3f)', z_names{i}, coeff_of_det{i}));
    
    axH(i+size(z,2)) = subplot(3,size(z,2),i+size(z,2));
    plot(t, nu, 'k-');
    hold on; grid on;
    plot([t(1) t(end)], 2*s{i}*[1 1], 'r-.');
    plot([t(1) t(end)], 2*s{i}*[-1 -1], 'r-.');
    title(sprintf('Residual: \\nu_{%s} (2\\sigma=%.1e)', z_names{i}, 2*s{i}));
    
    subplot(3,size(z,2),i+2*size(z,2));
    plot(yhat, nu, 'k.');
    hold on; grid on;
    plot([min(yhat) max(yhat)], 2*s{i}*[1 1], 'r-.');
    plot([min(yhat) max(yhat)], 2*s{i}*[-1 -1], 'r-.');
    ylabel(['\nu_{' z_names{i} '}']);
    xlabel(z_names{i});
end
linkaxes(axH, 'x');

% Print the parameters
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