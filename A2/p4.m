clear
clc
%%
y0              = [80; 30; 40]*pi/180;  % column vector of initial euler angles phi, theta, psi
tspan           = [0 121];              % [startTime endTime]
[tout, yout]    = rkf45(@Adot, tspan, y0, 0.00000001);    
%%
figure
plot(tout, yout(:,1))
ylabel('\phi in radians')
title('\phi')
xlabel('Time in s')

figure
plot(tout, yout(:,2))
ylabel('\theta in radians')
title('\theta')

figure
plot(tout, yout(:,3))
ylabel('\psi in radians')
title('\psi')