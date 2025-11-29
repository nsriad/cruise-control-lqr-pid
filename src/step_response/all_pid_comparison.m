% step_compare_all.m

clear; clc; close all;

% Common data
amp     = 1;   % step input
t_final = 30;      % simulation time
t = 0:0.01:t_final;

% plant model
num = 2.4767;
den = [1 6.0476 5.2856 0.238];
G   = tf(num, den);

% PID controller
Kp_mse = 2.8; Ki_mse = 0.437; Kd_mse = 2; % MSE as fitness function
Kp_ise = 2.8; Ki_ise = 1.247; Kd_ise = 2; % ISE as fitness function
Kp_itse = 2.8; Ki_itse = 0.4; Kd_itse = 2; % ITSE as fitness function
Kp_itae = 2.8; Ki_itae = 0.493; Kd_itae = 2; % ITSE as fitness function

PID_mse = pid(Kp_mse, Ki_mse, Kd_mse);
PID_ise = pid(Kp_ise, Ki_ise, Kd_ise);
PID_itse = pid(Kp_itse, Ki_itse, Kd_itse);
PID_itae = pid(Kp_itae, Ki_itae, Kd_itae);

% Closed-loop PID TF
T_pid_mse = feedback(PID_mse * G, 1);
T_pid_ise = feedback(PID_ise * G, 1);
T_pid_itse = feedback(PID_itse * G, 1);
T_pid_itae = feedback(PID_itae * G, 1);

% Simulate step responses on the same time vector
r = amp * ones(size(t));    % reference = 1

y_pid_mse   = lsim(T_pid_mse,  r, t);
y_pid_ise   = lsim(T_pid_ise,  r, t);
y_pid_itse   = lsim(T_pid_itse,  r, t);
y_pid_itae   = lsim(T_pid_itae,  r, t);


% Plot comparison
figure;
plot(t, r, 'k--', 'LineWidth', 2); hold on;
plot(t, y_pid_mse,   'LineWidth', 2.0); hold on;
plot(t, y_pid_ise,   'LineWidth', 2.0); hold on;
plot(t, y_pid_itse,   'LineWidth', 2.0); hold on;
plot(t, y_pid_itae,   'LineWidth', 2.0); hold on;
grid on;

title('PID response for all fitness functions');
xlabel('Time (s)');
ylabel('Speed (m/s)');
ylim([0 1.5]);

legend('Reference', 'MSE', 'ISE', 'ITSE', 'ITAE', 'Location', 'best');

% time response specs
disp("PID-MSE Step Response Metrics:");
stepinfo(y_pid_mse, t)

disp("PID-ISE Step Response Metrics:");
stepinfo(y_pid_ise, t)

disp("PID-ITSE Step Response Metrics:");
stepinfo(y_pid_itse, t)

disp("PID-ITAE Step Response Metrics:");
stepinfo(y_pid_itae, t)

% Steady-state error (Ess)
r_final = 1; % desired reference
Ess_ise = r_final - y_pid_ise(end);
Ess_itse = r_final - y_pid_itse(end);
Ess_itae = r_final - y_pid_itae(end);
Ess_mse = r_final - y_pid_mse(end);

fprintf("Final output r_final = %.6f m/s\n", r_final);
fprintf("Steady-state error Ess-ISE = %.6f m/s\n", Ess_ise);
fprintf("Steady-state error Ess-ITSE = %.6f m/s\n", Ess_itse);
fprintf("Steady-state error Ess-ITAE = %.6f m/s\n", Ess_itae);
fprintf("Steady-state error Ess-MSE = %.6f m/s\n", Ess_mse);