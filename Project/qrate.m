function qdot = qrate(t,q,w)

% w0=[0 0 0 0 0 0];
% tspan = [0 t];
% [tout, wout]    = ode45(@wrate, tspan, w0,   0.00000001);  
% w=wout(); 
qdot=0.5*w*q;
% Qq= [Q(1) Q(4) -Q(3) Q(2); Q(2) Q(3) Q(4) -Q(1);...
%          Q(3) -Q(2) Q(1) Q(4); Q(4) -Q(1) -Q(2) -Q(3)]

end