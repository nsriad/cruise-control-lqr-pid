clear all; close all; clc;

% State-space model
A = [0 1 0; 0 0 1; -6.0476 -5.2856 -0.238];
B = [0; 0; 2.4767];
C = [1 0 0];
D = 0;

% Compute LQR gain
% K = lqr(A, B, Q, R);
% Q and R determined using GA in the paper, not mentioned

K = [0.0898 1.5708 1.4131];   % from Table 4 and 5, optimal values
fprintf("K: ");
disp(K)

% Closed-loop system: A-BK
Acl = A - B*K;

% Nbar
Nbar = -1 / (C * inv(A - B*K) * B);
fprintf("Nbar: ");
disp(Nbar);

%closed loop state space
sys_cl = ss(Acl, B*Nbar, C, D);

% simulation duration
t = 0:0.01:60;

% reference profile
r = zeros(size(t));

% 0–20 sec: 16.67 m/s (60 km/h)
r(t >= 0 & t < 20) = 16.67;

% 20–40 sec: 25 m/s (90 km/h)
r(t >= 20 & t < 40) = 25.0;

% 40–60 sec: back to 16.67 m/s
r(t >= 40) = 16.67;

% Simulate closed-loop system
y = lsim(sys_cl, r, t);

% plot
figure;
plot(t, y, 'LineWidth', 2);
grid on;
title("LQR acc/dec Response");
xlabel("Time (s)");
ylabel("Speed (m/s)");
ylim([0 30]);


% % specs on the 1st segment only (0 to 60)
% idx = t <= 20;     
% disp("Step info for first segment (first 20 sec):")
% stepinfo(y(idx), t(idx))
