%% Two-Wheeled Inverted Pendulum Simulation
clear; close all; clc;

%% Initialize Model
p = init_model();

%% LQR Controller Design
% Linearize w.r.t. upright equilibrium
A = A_func(zeros(6,1),zeros(2,1));
B = B_func(zeros(6,1),zeros(2,1));
Ar = A([2,4:6],[2,4:6]);
Br = B([2,4:6],:);
Q = 0.001 * diag([1,0,0,0]);
R = 10 * diag([1,1]);
K = lqr(Ar,Br,Q,R);
umax = 2;
controller = @(y) max(-umax,min(umax,-K*y([2,4:6])));

%% Simulation
x0 = 0;
dx0 = 0;
th0 = deg2rad(35);
dth0 = deg2rad(0);
psi0 = 0;
dpsi0 = deg2rad(0);
y0 = [x0; th0; psi0; dx0; dth0; dpsi0];
N = 1000;
t = linspace(0,10,N);
closed_loop = @(t,y) twip_nonlinear_dynamics(y, controller(y), p);
[t,y] = ode45(closed_loop, t, y0);
y(:,[2,3,5,6]) = rad2deg(y(:,[2,3,5,6]));

%% Plots
figure('Name','Trajectory')
ylab = {'Distance [m]', 'Tilt [deg]', 'Yaw [deg]', 'Speed [m/s]', 'Tilt Rate [deg/s]', 'Yaw Rate [deg/s]'};
for i = 1:6
    ax(i) = subplot(3,2,i);
    plot(t, y(:,i))
    ylabel(ylab{i})
end

figure('Name','Torques')
u = zeros(N,2);
for i = 1:N
    u(i,:) = controller(y(i,:)');
end


ylab = {'Left Wheel Torque [Nm]', 'Right Wheel Torque [Nm]'}; clear ax
for i = 1:2
    ax(i) = subplot(2,1,i);
    plot(t, u(:,i))
    ylabel(ylab{i})
end

animate_pendulum(deg2rad(y(:,2)), deg2rad(y(:,3)), y(:,1), t(2)-t(1))

beautify_plots(...
    'Font', 'Helvetica', ...
    'FontSize', 20);


