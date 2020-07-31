function ndotdot = ang_acc(n,ndot,IM,I,wi_sq,tB)

n1 = n(1);
n2 = n(2);

Wn = trans_mat(n);              % Transformation Matrix                 (21)
v = Wn*ndot;                    %[p,q,r]'                               (22)       
wi = sqrt(wi_sq);
wL = wi(1) - wi(2) + wi(3) - wi(4);                                    %(23)

% Equation 24
term1 = cross(v,(I*v));
term2 = cross((IM.*v),([0;0;wL]));
vdot = I\(- term1 - term2 + tB);                                       %(24)

% Equation 25
M11 = 0;
M12 = ndot(1)*cos(n1)*tan(n2) + ndot(2)*sin(n1)/(cos(n2)^2);
M13 = -ndot(1)*sin(n1)*cos(n2) + ndot(2)*cos(n1)/(cos(n2)^2);
M21 = 0;
M22 = -ndot(1)*sin(n1);
M23 = -ndot(1)*cos(n1);
M31 = 0;
M32 = (ndot(1)*cos(n1)/cos(n2)) + (ndot(1)*sin(n1)*tan(n2)/cos(n2));
M33 = (-ndot(1)*sin(n1)/cos(n2)) + (ndot(2)*cos(n1)*tan(n2)/cos(n2));

M = [M11,M12,M13;               % Matrix in Equation (25)
     M21,M22,M23;
     M31,M32,M33];

termA = M*v;
termB = Wn\vdot;

ndotdot = termA + termB;        % Angular Acceleration in Global Frame  (25)
 
end