function [wi_data,x_data,n_data,xdot_data,ndot_data,t] = simulation(P,D,IN,dt,tf)

%% Note:
% There are three structures in the code.
%   1. P  - Contains all physcial parameters.
%   2. IN - Contains all Initial Conditions: Position & Attitude.
%   3. D  - Contains all Desired Conditions: Position & Attitude.

%% Unloading Structures
% Unload Physical Parameters
g   = P.g;                  % Gravitational Constant                    [m/s^2]
m   = P.m;                  % Aircraft Mass                             [kg]
l   = P.l;                  % Aircraft Arm Length                       [m]
k   = P.k;                  % Thrust Constant                           [kg.m]
b   = P.b;                  % Torque Constant                           [kg.m^2]
IM  = P.IM;                 % Rotor Moment of Inertia                   [kg.m^2]
Ixx = P.Ixx;                % Aircraft Moment of Inertia, x-axis        [kg.m^2]
Iyy = P.Iyy;                % Aircraft Moment of Inertia, y-axis        [kg.m^2]
Izz = P.Izz;                % Aircraft Moment of Inertia, z-axis        [kg.m^2]
Axx = P.Axx;                % Aerodynamic Effects, x-axis               [kg/s]
Ayy = P.Ayy;                % Aerodynamic Effects, y-axis               [kg/s]
Azz = P.Azz;                % Aerodynamic Effects, z-axis               [kg/s]

I = [Ixx,0,0;               % Moment of Inertia Matrix
     0,Iyy,0;
     0,0,Izz];

A = [Axx,0,0;               % Aerodynamic Effects Matrix
     0,Ayy,0;
     0,0,Azz];

% k = k*ones(4,1);
% b = b*[1,-1,1,-1]';

% Unload Initial Conditions
x    = IN.x;                % Initial Linear Position, [x,y,z]'         [m]
n    = IN.n;                % Initial Angular Position, [φ,θ,ψ]'        [rad]
xdot = IN.xdot;             % Initial Linear Velocity, [x,y,z]'         [m/s]
ndot = IN.ndot;             % Initial Angular Velocity, [φ,θ,ψ]'        [rad/s]

%% Initialisation of Constants
i    = 0;                   % Iteration Counter                         [-]
t(1) = 0;                   % Time Vector                               [s]
Le   = zeros(6,1);          % PID Error, [x,y,z,φ,θ,ψ]'                 [m,m,m,rad,rad,rad]

er   = 0.1;                 % Error for Termination Condition.
er = [er;er;er];

wi_data   = zeros(4,1);     % Vector to Store Rotor Velocities at i.
x_data    = zeros(3,1);     % Vector to Store Linear Position at i.
xdot_data = zeros(3,1);     % Vector to Store Linear Velocity at i.
n_data    = zeros(3,1);     % Vector to Store Angular Position at i.
ndot_data = zeros(3,1);     % Vector to Store Angular Velocity at i.

while(1)
    % Time & Iteration
    t(i+1) = i * dt;        % Incremental Time Vector                   (1)
    i = i + 1;              % Iternation Counter                        (2)
    
    % PID & Motor Mixing
    [u,Le] = PID(D,x,n,Le,dt,i);                        % PID Controller
    [wsq1,wsq2,wsq3,wsq4] = motormix(u,n,k,b,l,m,g,i);  % Motor Mixing
    wi_sq = [wsq1,wsq2,wsq3,wsq4];                      % Rotor Velocity-Square Vector
    
    % Physical Simulation Calculation 
    fi = k.*wi_sq;                      % Thrust per Rotor              (3)
    tMi = b.*wi_sq; % +IM.*wi;          % Torque per Rotor              (4)
    
    T = sum(fi);                        % Total Thrust                  (5)
    TB = [0,0,T]';                      % Thrust Vector in Body Frame   (6)
    tB = [l*k*(-wi_sq(2) +wi_sq(4));    % Torque Vector in Body Frame   (7)
          l*k*(-wi_sq(2) +wi_sq(4));
          tMi(1)-tMi(2)+tMi(3)-tMi(4)];
    
    xdotdot = lin_acc(g,T,m,n,A,xdot);          % Linear Acceleration
    ndotdot = ang_acc(n,ndot,IM,I,wi_sq,tB);    % Angular Acceleration
    
    % Euler's Increment    
    ndot = ndot + dt*ndotdot;           % Angular Velocity              (8)
    n = n + dt*ndot;                    % Angular Position              (9)
    
    xdot = xdot + dt*xdotdot;           % Linear Velocity               (10)
    x = x + dt*xdot;                    % Linear Position               (11)
    
    % Data Saving
    wi_data(:,i) = sqrt(wi_sq);
    x_data(:,i) = x;
    n_data(:,i) = n;
    xdot_data(:,i) = xdot;
    ndot_data(:,i) = ndot;
    
    % Termination Condition
    if er > abs(D.x - x)
        xcheck = 1;
    else
        xcheck = 0;
    end
    if er > abs(D.n - n)
        ncheck = 1;
    else
        ncheck = 0;
    end
    if t(i) == tf || xcheck == 1 && ncheck == 1
        break
    end 
end