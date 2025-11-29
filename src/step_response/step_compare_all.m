% step_compare_all.m

clear; clc; close all;

% Common data
amp     = 16.67;   % 60 km/h in m/s
t_final = 25;      % simulation time
t = 0:0.01:t_final;

% plant model
num = 2.4767;
den = [1 6.0476 5.2856 0.238];
G   = tf(num, den);

% PID controller
Kp = 2.8; Ki = 0.4; Kd = 2;
PID = pid(Kp, Ki, Kd);

% Closed-loop PID TF
T_pid = feedback(PID * G, 1);

% LQR closed-loop model
A = [0 1 0; 0 0 1; -6.0476 -5.2856 -0.238];
B = [0; 0; 2.4767];
C = [1 0 0];
D = 0;

% LQR gain from paper
K = [0.0898 1.5708 1.4131];

% Closed-loop matrix
Acl = A - B*K;

% Nbar for reference tracking
Nbar = -1 / (C * inv(A - B*K) * B);

% Closed-loop state-space
sys_lqr = ss(Acl, B*Nbar, C, D);

% Simulate step responses on the same time vector
r = amp * ones(size(t));    % reference = 16.67 m/s

y_plant = lsim(G, r, t);   % open-loop plant
y_pid   = lsim(T_pid,  r, t);   % PID closed-loop
y_lqr   = lsim(sys_lqr, r, t);  % LQR closed-loop

% Plot comparison
figure;
plot(t, r, 'k--', 'LineWidth', 2); hold on;
plot(t, y_pid,   'LineWidth', 2.0); hold on;
plot(t, y_lqr,   'LineWidth', 2.0);
grid on;

title('Step Response Comparison (PID vs LQR)');
xlabel('Time (s)');
ylabel('Speed (m/s)');
ylim([0 20]);

legend('Reference', 'PID', 'LQR', 'Location', 'best');
