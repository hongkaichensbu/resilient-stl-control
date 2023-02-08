% specify the configurations of the parameters in Mata2019
I_z = 5757; % kg/m2
l_F = 1.4; % m
l_R = 2.55; % m
% C_alphaF = 50000; % N/rad
% C_alphaR = 36000; % N/rad
m = 2200; % kg
v = 10;

C_alphaF = 2200; % N/rad
C_alphaR = 2200; % N/rad

% I_z = 1536.7; % kg/m2
% l_F = 1.016; % m
% l_R = 1.564; % m
% C_alphaF = 956.08; % N/rad
% C_alphaR = 956.08; % N/rad
% m = 1412; % kg
% v = 22.222;  % m/s = 80 km/h

%
a_c1 = - (2*C_alphaF+2*C_alphaR)/(m*v);
a_c2 = - (2*l_F*C_alphaF-2*l_R*C_alphaR)/(m*v)-v;
a_c3 = - (2*l_F*C_alphaF-2*l_R*C_alphaR)/(I_z*v);
a_c4 = - (2*l_F^2*C_alphaF+2*l_R^2*C_alphaR)/(I_z*v);


