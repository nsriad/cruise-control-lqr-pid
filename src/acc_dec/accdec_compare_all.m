clear; clc; close all;

t = 0:0.01:60;
r = zeros(size(t));

r(t >= 0  & t < 20) = 16.67;
r(t >= 20 & t < 40) = 25.0;
r(t >= 40)          = 16.67;

% Plant & PID
num = 2.4767;
den = [1 6.0476 5.2856 0.238];
G   = tf(num, den);

Kp  = 2.8; Ki  = 0.4; Kd  = 2;
PID = pid(Kp, Ki, Kd);

T_pid = feedback(PID * G, 1);

% LQR closed-loop
A = [0 1 0;
     0 0 1;
    -6.0476 -5.2856 -0.238];

B = [0;
     0;
     2.4767];

C = [1 0 0];
D = 0;

K = [0.0898 1.5708 1.4131];

Acl  = A - B*K;
Nbar = -1 / (C * inv(A - B*K) * B);
sys_lqr = ss(Acl, B*Nbar, C, D);

% Simulate
y_pid = lsim(T_pid,  r, t);
y_lqr = lsim(sys_lqr, r, t);

% Plot comparison
figure;
plot(t, r, 'k--', 'LineWidth', 2); hold on;
plot(t, y_pid, 'LineWidth', 2.0);
plot(t, y_lqr, 'LineWidth', 2.0);
grid on;

title('Acceleration/Deceleration Reference vs PID vs LQR');
xlabel('Time (s)');
ylabel('Speed (m/s)');
ylim([0 30]);

legend('Reference', 'PID', 'LQR', 'Location', 'best');
