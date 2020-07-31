function R = rotation(n)
% Rotation Matrix
% Equation (3)

n1 = n(1);
n2 = n(2);
n3 = n(3);

R11 = cos(n3)*cos(n2);
R12 = cos(n3)*sin(n2)*sin(n1) - sin(n3)*cos(n1);
R13 = cos(n3)*sin(n2)*cos(n1) + sin(n3)*sin(n1);

R21 = sin(n3)*cos(n2);
R22 = sin(n3)*sin(n2)*sin(n1) + cos(n3)*cos(n1);
R23 = sin(n3)*sin(n2)*cos(n1) - cos(n3)*sin(n1);

R31 = -sin(n2);
R32 = cos(n2)*sin(n1);
R33 = cos(n2)*cos(n1);

R = [R11,R12,R13;
     R21,R22,R23;
     R31,R32,R33];
 
end