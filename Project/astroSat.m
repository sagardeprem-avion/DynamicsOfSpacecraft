clear
clc
 
%{
States  
[phiDot; psiDot; thetaDot; phi; psi; theta]
 %}

%% 
X0     = [0; 0; 0; 0; 0;0]*pi/180;     % initial state
tspan  = [0 500];              % [startTime endTime]
w      = [1 -2 2 1] ;           % wheel speeds
Iw     = [1 1 1 1]*2.5;           % wheel moment of inertia
Js     = [1763 -52 -16; -52 1591 25; -16 25 1185];
J      = eig(Js) ;           % principal moment of inertia of the spacecraft
w0     = 1.0741e-3;                   % earth rotation rate
Td     = [2e-3; 1e-4; 2e-3];   % disturbance torques
% Td     = [0;0;0 ];
func   = @(t,X)Xrate(t,X,w,Iw,J,w0,Td);
[tout, Xout]  = ode45(func, tspan, X0, 0.00000001);

for k=4:6
   subplot(3,1,k-3)
   plot(Xout(:,k))
end
    

