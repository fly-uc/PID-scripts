function xdotdot = lin_acc(g,T,m,n,A,xdot)
% Linear Acceleration Function


R = rotation(n);                                                   %(3)
TB = [0,0,T]';

% Linear Acceleration in Global Frame
xdotdot = - g*[0;0;1] + (1/m)*(R*TB) - (1/m)*(A*xdot);             %(20)

end