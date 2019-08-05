clear
clc
%%
psi         = 5*pi/180;
theta       = 10*pi/180;
phi         = -3*pi/180;
m           = 100;
I           = [34 1 6; 1 15 3; 6 3 10];
v           = 5;
psiDot      = -1*pi/180;
thetaDot    = 1*pi/180;
phiDot      = 4*pi/180;
%%
omega=[1 0 -sin(theta); 0 cos(phi) sin(phi)*cos(theta); 0 -sin(phi) cos(phi)*cos(theta)]*[phiDot; thetaDot; psiDot];
T=0.5*m*v^2+omega'*I*omega;
%%
[a b]=eig(I)


