function Wn = trans_mat(n)
% Transformation Matrix
% Equation (4)

n1 = n(1);
n2 = n(2);

W11 = 1;
W12 = 0;
W13 = -sin(n2);

W21 = 0;
W22 = cos(n1);
W23 = cos(n2)*sin(n1);

W31 = 0;
W32 = -sin(n1);
W33 = cos(n2)*cos(n1);

Wn = [W11,W12,W13;
      W21,W22,W23;
      W31,W32,W33];
  
end