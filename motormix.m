function [wsq1,wsq2,wsq3,wsq4] = motormix(u,n,k,b,l,m,g,i)

R = rotation(n);            % Body to Global Frame Rotation.            (16)
R = R';                     % Global to Body Frame Rotation.            (16a)
z = zeros(3,3);

u = u - [0;0;(m*g);0;0;0];  % Accounting for Gravity.                   (17)

uL = [R,z;z,R]*u;           % Control Inputs in Body Frame.             (18)

% Solving for Rotor Velocity Squares using Linear Systems. 
Lf = uL(3);                 % Lift Force in Body Frame.
tx = uL(4);                 % phi-Torque in Body Frame.
ty = uL(5);                 % theta-Torque in Body Frame.
tz = uL(6);                 % psi-Torque in Body Frame.
A_mat = [1,1,1,1; 1,-1,1,-1; 0,-1,0,1; -1,0,1,0];
RHS = [(Lf/k),(tz/b),(tx/(l*k)),(ty/(l*k))]';
wsq = A_mat\RHS;            %                                           (19)
wsq1 = wsq(1);              % Rotor 1 Velocity Square.
wsq2 = wsq(2);              % Rotor 2 Velocity Square.
wsq3 = wsq(3);              % Rotor 3 Velocity Square.
wsq4 = wsq(4);              % Rotor 4 Velocity Square.

% Negative Error Check
if wsq1 < 0 || wsq2 < 0 || wsq3 < 0 || wsq4 < 0
    fprintf('Rotor Velocity Squares:\nRotor 1: %10.4f\nRotor 2: %10.4f\nRotor 3: %10.4f\nRotor 4: %10.4f\nIteration: %i\n',wsq1,wsq2,wsq3,wsq4,i)
    error('Error 1: Rotor Velocity is Negative.')
end
end