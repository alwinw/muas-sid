

CL_fn = subs(CL, [param_estimates(1:5,1); x_eq], [param_estimates(1:5,2); x_eq_vals]);
CL_fn = matlabFunction(CL_fn, 'vars', {V, alpha, delta_e, q, delta_a});
CD_fn = subs(CD, [param_estimates(1:11,1); x_eq], [param_estimates(1:10,2); 0; x_eq_vals]);
CD_fn = matlabFunction(CD_fn, 'vars', {V, alpha, delta_e, q});


fprintf('L=%.2f kg  D=%.2f N\n', 0.5*rho*S*25^2*CL_fn(25, 0.05, -0.055, 0, 0)/9.81, ...
                          0.5*rho*S*25^2*CD_fn(25, 0.05, -0.055, 0));

% V_dp = 25;
% delta_e_dp = 0;
% q_dp = 0;
% alpha_dp = linspace(-5, 10, 100) * pi/180;
% plot(CL_fn(V_dp,alpha_dp,delta_e_dp,q_dp), CD_fn(V_dp,alpha_dp,delta_e_dp,q_dp));


V0 = 25;
alpha_dp = 0.05;
delta_e_dp = -0.055;
delta_a_dp = 0.0;
q_dp = 0;
V_dp = linspace(V0-5,V0+5,100);
figure(4); clf;
subplot(211);
plot(CL_fn(V_dp,alpha_dp,delta_e_dp,q_dp,delta_a_dp), CD_fn(V_dp,alpha_dp,delta_e_dp,q_dp));
hold on;
plot(CL_fn(V0,alpha_dp,delta_e_dp,q_dp,delta_a_dp), CD_fn(V0,alpha_dp,delta_e_dp,q_dp), 'ko');
legend('Drag polar', sprintf('V=%.1f m/s', V0));
xlabel('C_L');
ylabel('C_D');
grid on
subplot(212);
plot(V_dp, CL_fn(V_dp,alpha_dp,delta_e_dp,q_dp,delta_a_dp)./CD_fn(V_dp,alpha_dp,delta_e_dp,q_dp));
hold on;
plot(V0, CL_fn(V0,alpha_dp,delta_e_dp,q_dp,delta_a_dp)./CD_fn(V0,alpha_dp,delta_e_dp,q_dp), 'ko');
xlabel('V');
ylabel('L/D');
grid on