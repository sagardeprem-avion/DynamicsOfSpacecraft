clear
clc
%% Case 1 with spin about major axis 
%% Numerically solving the ODE
Ixx=98;
Iyy=102;
Izz=150;
% w0              = [0.1; 0.02; 0.5] ;  % case 1 with major axis rotation
w0              = [0.5; 0.02; 0.01];      % case 2 with minor axis rotation
tspan           = [0 20000 ];            % [startTime endTime]
[tout, wout]    = rkf45(@wrates, tspan, w0,   0.00000001);  
% [tout, wout]    = ode45(@wrates, tspan, w0,0.00000001);    

%% Angular velocities
figure
subplot(3,1,1)
plot(tout, wout(:,1))
ylabel('\omega_x in radians/s')
title('\omega_x')
xlabel('Time in s')

subplot(3,1,2)
plot(tout, wout(:,2))
ylabel('\omega_y in radians/s')
title('\omega_y')

subplot(3,1,3)
plot(tout, wout(:,3))
ylabel('\omega_z in radians/s')
title('\omega_z')

%% Precession rate
 
H=diag([Ixx Iyy Izz])*wout';   % Angular momentum
for i=1:max(size(H)) 
h(i)=norm(H(:,i));              % magnitudes or angular momentum
end
% thetaDot= h/sqrt(Ixx^2+Iyy^2);    % case 1 precession rate
thetaDot= h/sqrt(Iyy^2+Izz^2);      % case 2 precession rate
theta=trapz(tout, thetaDot)
figure
subplot(2,1,1)
plot(tout, thetaDot)
ylabel('Precession rate in radians/s')
title(' Precession rate')

%% Nutation angle

% gamma=acos(H(3,:)./h);             % case 1 nutation angle
gamma=acos(H(1,:)./h);               % case 2 nutation angle

subplot(2,1,2)
plot(tout, gamma)
ylabel('Nutation angle in radians')
title('Nutation angle')




