clear all; close all; clc;

% syms s
% expand( (s + 0.0476)*(s + 1)*(s + 5) )
% s^3 + (15119*s^2)/2500 + (6607*s)/1250 + 119/500 = s^3 + 6.0476*s^2 + 5.2856*s + 0.238

% using pid controller
% Plant model
num = 2.4767;
den = [1 6.0476 5.2856 0.238];
G = tf(num, den);

% PID controller (from paper, ITSE optimized)
Kp = 2.8;
Ki = 0.4;
Kd = 2;
PID = pid(Kp, Ki, Kd);

% Closed-loop transfer function
T = feedback(PID * G, 1);

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
y = lsim(T, r, t);


% plot Step Response
figure;
plot(t, y, 'LineWidth', 2);
grid on;
title('PID acc/dec Response');
xlabel('Time (s)');
ylabel('Speed (m/s)');
ylim([0 30]);

% time response specs
disp("PID Step Response Metrics:");
stepinfo(y, t)