% step_compare_all.m

clear; clc; close all;

% Common data
amp     = 1; 
t_final = 30;      % simulation time
t = 0:0.01:t_final;

% LQR closed-loop model
A = [0 1 0; 0 0 1; -6.0476 -5.2856 -0.238];
B = [0; 0; 2.4767];
C = [1 0 0];
D = 0;

% LQR gain from paper
K_ise = [0.0204 1.5362 1.4038];
K_itse = [0.1968 4.4657 3.5938];
K_itae = [0.0898 1.5708 1.4131];
K_mse = [7.3238 6.9351 3.8548];

% Closed-loop matrix
Acl_ise = A - B*K_ise;
Acl_itse = A - B*K_itse;
Acl_itae = A - B*K_itae;
Acl_mse = A - B*K_mse;

% Nbar for reference tracking
Nbar_ise = -1 / (C * inv(A - B*K_ise) * B);
Nbar_itse = -1 / (C * inv(A - B*K_itse) * B);
Nbar_itae = -1 / (C * inv(A - B*K_itae) * B);
Nbar_mse = -1 / (C * inv(A - B*K_mse) * B);

% Closed-loop state-space
sys_lqr_ise = ss(Acl_ise, B*Nbar_ise, C, D);
sys_lqr_itse = ss(Acl_itse, B*Nbar_itse, C, D);
sys_lqr_itae = ss(Acl_itae, B*Nbar_itae, C, D);
sys_lqr_mse = ss(Acl_mse, B*Nbar_mse, C, D);

% Simulate step responses on the same time vector
r = amp * ones(size(t));    % reference = 16.67 m/s

y_lqr_ise   = lsim(sys_lqr_ise, r, t);
y_lqr_itse   = lsim(sys_lqr_itse, r, t);
y_lqr_itae   = lsim(sys_lqr_itae, r, t);
y_lqr_mse   = lsim(sys_lqr_mse, r, t);

% Plot comparison
figure;
plot(t, r, 'k--', 'LineWidth', 2); hold on;
plot(t, y_lqr_ise,   'LineWidth', 2.0); hold on;
plot(t, y_lqr_itse,   'LineWidth', 2.0); hold on;
plot(t, y_lqr_itae,   'LineWidth', 2.0); hold on;
plot(t, y_lqr_mse,   'LineWidth', 2.0); hold on;
grid on;

title('LQR response for all fitness functions');
xlabel('Time (s)');
ylabel('Speed (m/s)');
ylim([0 1.5]);

legend('Reference', 'ISE', 'ITSE','ITAE', 'MSE', 'Location', 'best');

% time response specs
disp("LQR-MSE Step Response Metrics:");
stepinfo(y_lqr_mse, t)

disp("LQR-ISE Step Response Metrics:");
stepinfo(y_lqr_ise, t)

disp("LQR-ITSE Step Response Metrics:");
stepinfo(y_lqr_itse, t)

disp("LQR-ITAE Step Response Metrics:");
stepinfo(y_lqr_itae, t)

% Steady-state error (Ess)
r_final = 1; % desired reference
Ess_ise = r_final - y_lqr_ise(end);
Ess_itse = r_final - y_lqr_itse(end);
Ess_itae = r_final - y_lqr_itae(end);
Ess_mse = r_final - y_lqr_mse(end);

fprintf("Final output r_final = %.6f m/s\n", r_final);
fprintf("Steady-state error Ess-ISE = %.6f m/s\n", Ess_ise);
fprintf("Steady-state error Ess-ITSE = %.6f m/s\n", Ess_itse);
fprintf("Steady-state error Ess-ITAE = %.6f m/s\n", Ess_itae);
fprintf("Steady-state error Ess-MSE = %.6f m/s\n", Ess_mse);