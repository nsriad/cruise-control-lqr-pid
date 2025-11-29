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

amp = 16.67; 
t_final = 25;

% Step input of 16.67 m/s (60 km/h)
[y, t] = step(amp * T, t_final);

% Steady-State Error (Ess)
y_final = y(end); % final value from simulation
r_final = amp; % desired reference
Ess = r_final - y_final;

fprintf("Final output y_final = %.6f m/s\n", y_final);
fprintf("Steady-state error Ess = %.10f m/s\n", Ess);

% plot Step Response
figure;
plot(t, y, 'LineWidth', 2);
grid on;
title('PID Step Response');
xlabel('Time (s)');
ylabel('Speed (m/s)');
ylim([0 30]);

% time response specs
disp("PID Step Response Metrics:");
stepinfo(y, t)