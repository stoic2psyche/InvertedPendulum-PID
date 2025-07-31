% Physical parameters
M = 0.5;      % Mass of cart (kg)
m = 0.2;      % Mass of pendulum (kg)
b = 0.1;      % Coefficient of friction (N/m/sec)
I = 0.006;    % Inertia of the pendulum (kg.m^2)
g = 9.8;      % Acceleration due to gravity (m/s^2)
l = 0.3;      % Length to pendulum center of mass (m)

% Denominator helper
p = I*(M + m) + M*m*l^2;
% Transfer function from force (input) to cart position (output)
num = [p, 0];
den = [p, b*(I + m*l^2), -(m^2)*g*l^2, -b*m*g*l];

G_x = tf(num, den);  % Plant: F → x
pidTuner(G_x, 'PID')
% Closed-loop transfer function
T_closed = feedback(C_pid * G_x, 1);  % unity feedback

% Simulate step response
figure;
step(T_closed);
title('Closed-loop Step Response with Tuned PID');
grid on;

% Your plant (already defined):
% G_x = tf(...);

% 1. Multiply PID with plant (open-loop system)
OL = C_pid * G_x;

% 2. Create closed-loop with unity feedback
CL = feedback(OL, 1);

% 3. Plot step response of closed-loop
figure;
step(CL);
title('Closed-loop Step Response with Tuned PID');
grid on;

% Define plant (G_x) if not already defined
% Example: G_x = tf([numerator], [denominator]);

% Use your existing G_x definition

% PID Controller (from workspace)
C_pid = pid(339, 758, 37.9);  % Kp, Ki, Kd

% Open-loop system: PID * Plant
OL = C_pid * G_x;

% Closed-loop system with unity feedback
CL = feedback(OL, 1);

% Step response
figure;
step(CL);
title('Closed-loop Step Response with Tuned PID');
xlabel('Time (s)');
ylabel('Output');
grid on;

