% Matlab code for studying dynamics of gravity gradient satellites
function gravitygradient
%program for torque-free rotational dynamics and Euler 3-2-1 kinematics
%of rigid spacecraft with linearized equations
%x(1)=omega_x, x(2)=omega_y, x(3)=omega_z (angular velocity in rad/s)
%x(4)=psi, x(5)=theta, x(6)=phi (rad)
%(c) 2017 R. Pandiyan
pi = 3.14159265;
xin(1) = 0.0;
xin(2) = 0.0;
xin(3) = 5.0*pi/180.0;
xin(4) = 0.0;
xin(5) = 0.0;
xin(6) = 0.0;
[t,x]=ode45(@f,[0 20000],[0.0,0.0,0.0*pi/180.0,0.0,0.0,0.0]');
subplot(311), plot(t,x(:,1)*180/pi), hold all, ...
title('phi');
xlabel('time in secs');
ylabel('phi in degrees');
subplot(312), plot(t,x(:,2)*180/pi), hold all, ...
title('theta');
xlabel('time in secs');
ylabel('theta in degrees');
subplot(313), plot(t,x(:,3)*180/pi), hold all, ...
title('psi');
xlabel('time in secs');
ylabel('psi in degrees');
