%% FlyUC Controls Code
% 11 July 2020 AY
% Version 3.0

clear; clc; close all;
%% Setup
% Physical Parameters
P.g = 9.81;             % Gravitational Constant                    [m/s^2]
P.m = 0.468;            % Aircraft Mass                             [kg]
P.l = 0.225;            % Aircraft Arm Length                       [m]
P.k = 2.980E-6;         % Thrust Constant                           [kg.m]
P.b = 1.140E-7;         % Torque Constant                           [kg.m^2]
P.IM = 3.357E-5;        % Rotor Moment of Inertia                   [kg.m^2]
P.Ixx = 4.856E-3;       % Aircraft Moment of Inertia, x-axis        [kg.m^2]
P.Iyy = 4.856E-3;       % Aircraft Moment of Inertia, y-axis        [kg.m^2]
P.Izz = 4.856E-3;       % Aircraft Moment of Inertia, z-axis        [kg.m^2]
P.Axx = 0.25;           % Aerodynamic Effects, x-axis               [kg/s]
P.Ayy = 0.25;           % Aerodynamic Effects, y-axis               [kg/s]
P.Azz = 0.25;           % Aerodynamic Effects, z-axis               [kg/s]

% Initial Conditions
IN.x = [0,0,1]';        % Initial Linear Position, [x,y,z]'         [m]
IN.n = [0,0,0]';        % Initial Angular Position, [φ,θ,ψ]'        [rad]
IN.xdot = [0,0,0]';     % Initial Linear Velocity, [x,y,z]'         [m/s]
IN.ndot = [0,0,0]';     % Initial Angular Velocity, [φ,θ,ψ]'        [rad/s]

IN.n = deg2rad(IN.n);           % Converting Angular Position to Degrees
IN.ndot = deg2rad(IN.ndot);     % Converting Angular Velocity to Degrees

% Desired Conditions
D.x = [0,0,0]';         % Desired Linear Position, [x,y,z]'         [m]
D.n = [0,0,0]';         % Desired Angular Position, [φ,θ,ψ]'        [rad]
D.xdot = [0,0,0]';      % Desired Linear Velocity, [x,y,z]'         [m/s]
D.ndot = [0,0,0]';      % Desired Angular Velocity, [φ,θ,ψ]'        [rad/s]
D.xdotdot = [0,0,0]';   % Desired Linear Acceleration, [x,y,z]'     [m/s^2]

%% Simulation
dt = 0.0001;            % Simulation Step Size                      [s]
tf = 0.100;             % Total Simulation Time                     [s]

[wi,x,n,xdot,ndot,t] = simulation(P,D,IN,dt,tf); % SIMULATION FUNCTION

n = rad2deg(n);         % Converting Angular Positon to Degrees
ndot = rad2deg(ndot);   % Converting Angular Velocity to Degrees
wi = rad2deg(wi);       % Converting Rotor Velocities to Degrees

%% Plotting Linear Quantities
figure(1)
subplot(2,1,1) % Linear Position
plot(t,x(1,:),'k--')
hold on
plot(t,x(2,:),'b--')
hold on
plot(t,x(3,:),'r--')
title('Linear Position')
xlabel('Time [s]')
ylabel('Linear Position [m]')
legend('x-Position','y-Position','z-Position','location','best')
grid on
grid minor

subplot(2,1,2) % Linear Velocity
plot(t,xdot(1,:),'k--')
hold on
plot(t,xdot(2,:),'b--')
hold on
plot(t,xdot(3,:),'r--')
title('Linear Velocity')
xlabel('Time [s]')
ylabel('Linear Velocity [m/s]')
legend('x-Velocity','y-Velocity','z-Velocity','location','best')
grid on
grid minor

%% Plotting Angular Quantities
figure(2)
subplot(2,1,1) % Angular Position
plot(t,n(1,:),'k--')
hold on
plot(t,n(2,:),'b--')
hold on
plot(t,n(3,:),'r--')
title('Angular Position')
xlabel('Time [s]')
ylabel('Angular Position [deg]')
legend('φ-Position','θ-Position','ψ-Position','location','best')
grid on
grid minor

subplot(2,1,2) % Angular Velocity
plot(t,ndot(1,:),'k--')
hold on
plot(t,ndot(2,:),'b--')
hold on
plot(t,ndot(3,:),'r--')
title('Angular Velocity')
xlabel('Time [s]')
ylabel('Angular Velocity [deg/s]')
legend('φ-Velocity','θ-Velocity','ψ-Velocity','location','best')
grid on
grid minor

%% Plotting Rotor Velocity
figure(3)
plot(t,wi(1,:),'k--')
hold on
plot(t,wi(2,:),'b--')
hold on
plot(t,wi(3,:),'r--')
hold on
plot(t,wi(4,:),'b--')
title('Rotor Velocity')
xlabel('Time [s]')
ylabel('Rotor Velocity [deg/s]')
legend('Rotor 1','Rotor 2','Rotor 3','Rotor 4','location','best')
grid on
grid minor