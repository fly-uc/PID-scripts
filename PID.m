function [u,Le] = PID(D,x,n,Le,dt,i)

% Gain Values.
Kp = 6;                               % Propotional Constant
Kd = 6;                               % Derivative Constant
Ki = 6;                               % Integral Constant

% States & Error
Lc = [x;n];                             % Current State.
Ld = [D.x;D.n];                         % Desired State.
Le(:,i+1) = Lc - Ld;                    % Error Calculation.            (12)

% Numerical Integration & Differentiation
Le_Ii = zeros(6,1);
for k = 1:i                             %                               (13)
    Ii = dt*(Le(:,k) + Le(:,k+1))/2;    %                                .
    Le_Ii(:,k) = Ii;                    %                                .
end                                     %                                .
Le_I = sum(Le_Ii,2);                    % Integrated Error              (13)

Le_D = (Le(:,i+1) - Le(:,i))/dt;        % Differentiated Error          (14)

% Control Function
u = Kp.*Le + Kd*Le_D + Ki*Le_I;          %                               (15)
end