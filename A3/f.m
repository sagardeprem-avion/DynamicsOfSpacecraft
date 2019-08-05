function xdot=f(t,x)
%x(1)=omega_x, x(2)=omega_y, x(3)=omega_z (angular velocity in rad/s)
%x(4)=psi, x(5)=theta, x(6)=phi (rad)
%w0=orbital velocity in rad/s
J1=80; J2=82; J3=4; %principal moments of inertia (kg.m^2) before boom deployment
f1=(J2-J3)/J1;
f2=(J1+J3-J2)/J1;
f3=(J1-J3)/J2;
f4=(J2-J1)/J3;
f5=(J3+J1-J2)/J3;
w0=0.00104;   %  Orbital rate of the spacecraft motion
w02=w0*w0;
xdot(1,1)=x(4);
xdot(2,1)=x(5);
xdot(3,1)=x(6);
xdot(4,1)=-4*w02*f1*x(1)+w0*f2*x(6)+1.0e-5/J1;  % with disturbance torque
xdot(5,1)=-3*w02*f3*x(2)+0.0e-5/J2;
xdot(6,1)=-w02*f4*x(3)-w0*f5*x(4);
