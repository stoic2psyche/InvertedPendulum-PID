% Physical parameters of the system
M = 0.5;   % Mass of the cart (kg)
m = 0.2;   % Mass of the pendulum (kg)
b = 0.1;   % Coefficient of friction (N/m/sec)
I = 0.006; % Moment of inertia of the pendulum (kg.m^2)
g = 9.8;   % Acceleration due to gravity (m/s^2)
l = 0.3;   % Length to pendulum center of mass (m)

% Derived parameter for linearized system
p = I*(M+m) + M*m*l^2; % Denominator for A and B

% State-space matrices (linearized about θ ≈ 0)
A = [0 1 0 0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p  0;
     0 0 0 1;
     0 -(m*l*b)/p  m*g*l*(M+m)/p  0];

B = [0;
     (I + m*l^2)/p;
     0;
     m*l/p];

C = [1 0 0 0;
     0 0 1 0];  % Output = cart position and pendulum angle
D = [0; 0];

% State feedback design using LQR
Q = diag([10, 1, 10, 1]);  % State penalty
R = 0.001;                 % Control penalty
K = lqr(A, B, Q, R);

% Closed-loop system
Acl = A - B*K;
sys_cl = ss(Acl, B, C, D);

% Initial condition: small angle deviation
x0 = [0; 0; pi/12; 0];  % [position, velocity, angle, angular velocity]

t = 0:0.01:10;
[y, t, x] = initial(sys_cl, x0, t);

figure;
subplot(2,1,1);
plot(t, x(:,3)); ylabel('Angle (rad)'); title('Pendulum Angle (LQR)');
subplot(2,1,2);
plot(t, x(:,1)); ylabel('Cart Position (m)'); xlabel('Time (s)');


