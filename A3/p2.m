clear
clc

%% COM
x=([0 1.5 1]+4*[0 2.5 1])/(100*pi*0.5+5)

%% Cylinder  MOI
M=100*pi*0.5;
R=0.5;
h=2;
Jcyl=diag([M*h^2/12+0.25*M*R^2 M*h^2/12+0.25*M*R^2 0.5*M*R^2]);
r=0+x;   
rx=[0 r(3) -r(2); -r(3) 0 r(1); r(2) -r(1) 0];
Jcyl=Jcyl-M*rx*rx;      % parallel axis theorem


%% Rod MOI
m=1;
l=2;
Jrod=diag([m*l^2/12 0 m*l^2/12])
r=[0; -1.5; -1]; 
r=r+x';
rx=[0 r(3) -r(2); -r(3) 0 r(1); r(2) -r(1) 0];
Jrod=Jrod-m*rx*rx;      % parallel axis theorem


%% Panel MOI
mp=4;
a=2;
b=0.5;
Jpanel=diag([mp*a^2/12 mp*b^2/12 mp*(a^2+b^2)/12]);

% r=[0; -2.5; -1];
% r=r+x';
% rx=[0 r(3) -r(2); -r(3) 0 r(1); r(2) -r(1) 0];
% Jpanel=Jpanel-m*rx*rx;      % parallel axis theorem


theta=-87*pi/180;
C=[cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
Jpanel=C*Jpanel*C'       % rotation theorem

r=[0; -2.5; -1];
r=r+x';
rx=[0 r(3) -r(2); -r(3) 0 r(1); r(2) -r(1) 0];
Jpanel=Jpanel-m*rx*rx      % parallel axis theorem

%%  Total MOI
J=Jcyl+Jrod+Jpanel

