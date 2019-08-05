clc;
close all;
w0 = zeros(6,1);

M=1542; %%Mass of the spacecraft
J=[1763 -52 -16;-52 1591 25;-16 25 1185];%%Moment of inertia matrix of the entire SpaceCraft
Kpx=28; Kpy=28; Kpz=28;      %% Gain values Proportional
Kdx=0.95; Kdy=0.95; Kdz=0.95;   %% Gain values Derivative
Kp = [Kpx;Kpy;Kpz];
Kd = [Kdx;Kdy;Kdz];

%%case 1 zero initial guess with given disturbance torque without control%%
w0 = [0; 0; 0; 0; 0; 0;];
Tdx=2e-3; Tdy=1e-4; Tdz=2e-3;
Td = [Tdx;Tdy;Tdz];
c_on = 0;
tspan = [0 40000];
g_on = 0;
[tout,wout] = rkf45(@(t,w) AstroSatDynamics(t,w,Td,c_on,M,J,Kp,Kd,g_on),tspan,w0,0.0000001);
figure
for k=1:3
    subplot(3,1,k)
    wout(:,k) = wrapToPi(wout(:,k));
    plot(tout,wout(:,k)/pi()*180)
    if k == 1
        str = sprintf('Yaw rate %s \n (deg/s)','\psidot');
        ylabel(str)
        xlabel('Time in s')
        str = sprintf('Euler Angle Rates in deg/s Vs Time for zero initial guess \n with given disturbance torque and without control');
        title(str);
    elseif k ==2
        str = sprintf('Roll rate %s \n (deg/s)','\phidot');
        ylabel(str)
        xlabel('Time in s')
    else
        str = sprintf('Pitch rate %s \n (deg/s)','\thetadot');
        ylabel(str)
        xlabel('Time in s')
    end
end

str = sprintf('Euler_rate_zero_ig_wo_c')
str1  = sprintf('%s%s',str,'.fig')
savefig(str1);
saveas(gcf, str, 'jpg')
saveas(gcf, str, 'eps')

figure
for k=1:3
subplot(3,1,k)
wout(:,k+3) = wrapToPi(wout(:,k+3));
plot(tout,wout(:,k+3)/pi()*180)
    if k == 1
        str = sprintf('Yaw angle %s \n (deg)','\psi');
        ylabel(str)
        xlabel('Time in s')
        str = sprintf('Euler Angles in deg Vs Time for zero initial guess \n with given disturbance torque and without control');
        title(str);  
    elseif k ==2
        str = sprintf('Roll angle %s \n (deg)','\phi');
        ylabel(str)
        xlabel('Time in s')
    else
        str = sprintf('Pitch angle %s \n (deg)','\theta');
        ylabel(str)
        xlabel('Time in s')
    end
end

str = sprintf('Euler_angle_zero_ig_wo_c')
str1  = sprintf('%s%s',str,'.fig')
savefig(str1);
saveas(gcf, str, 'jpg')
saveas(gcf, str, 'eps')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%case 2 zero initial guess with given disturbance torque with control%%

w0 = [0; 0; 0; 0; 0; 0;]
Tdx=2e-3; Tdy=1e-4; Tdz=2e-3;
Kpx=28; Kpy=28; Kpz=28;      %% Gain values Proportional
Kdx=0.95; Kdy=0.95; Kdz=0.95;   %% Gain values Derivative
Kp = [Kpx;Kpy;Kpz];
Kd = [Kdx;Kdy;Kdz];
Td = [Tdx;Tdy;Tdz];
c_on = 1;
tspan = [0 20000]
[tout,wout] = rkf45(@(t,w) AstroSatDynamics(t,w,Td,c_on,M,J,Kp,Kd,g_on),tspan,w0,0.0000001);
wout_no_gg = wout;
figure
for k=1:3
    subplot(3,1,k)
    wout(:,k) = wrapToPi(wout(:,k));
    plot(tout,wout(:,k)/pi()*180)
    if k == 1
        str = sprintf('Yaw rate %s \n (deg/s)','\psidot');
        ylabel(str)
        xlabel('Time in s')
        str = sprintf('Euler Angle Rates in deg/s Vs Time for zero initial guess \n with given disturbance torque and with control');
        title(str);
    elseif k ==2
        str = sprintf('Roll rate %s \n (deg/s)','\phidot');
        ylabel(str)
        xlabel('Time in s')
    else
        str = sprintf('Pitch rate %s \n (deg/s)','\thetadot');
        ylabel(str)
        xlabel('Time in s')
    end
end

str = sprintf('Euler_rate_zero_ig_w_c')
str1  = sprintf('%s%s',str,'.fig')
savefig(str1);
saveas(gcf, str, 'jpg')
saveas(gcf, str, 'eps')

figure
for k=1:3
subplot(3,1,k)
wout(:,k+3) = wrapToPi(wout(:,k+3));
plot(tout,wout(:,k+3)/pi()*180)
    if k == 1
        str = sprintf('Yaw angle %s \n (deg)','\psi');
        ylabel(str)
        xlabel('Time in s')
        str = sprintf('Euler Angles in deg Vs Time for zero initial guess \n with given disturbance torque and with control');
        title(str);  
    elseif k ==2
        str = sprintf('Roll angle %s \n (deg)','\phi');
        ylabel(str)
        xlabel('Time in s')
    else
        str = sprintf('Pitch angle %s \n (deg)','\theta');
        ylabel(str)
        xlabel('Time in s')
    end
end

str = sprintf('Euler_angle_zero_ig_w_c')
str1  = sprintf('%s%s',str,'.fig')
savefig(str1);
saveas(gcf, str, 'jpg')
saveas(gcf, str, 'eps')
Rphi = 0;
Rpsy = 0;
Rtheta = 0;
Tcx=Kpx*(-Rphi+wout(:,4))+Kdx*wout(:,1);
    Tcy=Kpy*(-Rpsy+wout(:,5))+Kdy*wout(:,2);
    Tcz=Kpz*(-Rtheta+wout(:,6))+Kdz*wout(:,3);
T1=(sqrt(3)*(Tcx)+sqrt(3)*(Tcy)+sqrt(3)*(Tcz))/4;
T2=(-sqrt(3)*(Tcx)-sqrt(3)*(Tcy)+sqrt(3)*(Tcz))/4;
T3=(-sqrt(3)*(Tcx)+sqrt(3)*(Tcy)-sqrt(3)*(Tcz))/4;
T4=(sqrt(3)*(Tcx)-sqrt(3)*(Tcy)-sqrt(3)*(Tcz))/4;
figure
plot(tout,T1(:,1));
ylabel('Wheel 1 Torque in Nm')
xlabel('Time in s')
title('Wheel 1 Torque Vs Time')
str = sprintf('Wheel_1_torque_zero_ig_w_c')
str1  = sprintf('%s%s',str,'.fig')
savefig(str1);
saveas(gcf, str, 'jpg')
saveas(gcf, str, 'eps')
figure
plot(tout,T2(:,1));
ylabel('Wheel 2 Torque in Nm')
xlabel('Time in s')
title('Wheel 2 Torque Vs Time')
str = sprintf('Wheel_2_torque_zero_ig_w_c')
str1  = sprintf('%s%s',str,'.fig')
savefig(str1);
saveas(gcf, str, 'jpg')
saveas(gcf, str, 'eps')
figure
plot(tout,T3(:,1));
ylabel('Wheel 3 Torque in Nm')
xlabel('Time in s')
title('Wheel 3 Torque Vs Time')
str = sprintf('Wheel_3_torque_zero_ig_w_c')
str1  = sprintf('%s%s',str,'.fig')
savefig(str1);
saveas(gcf, str, 'jpg')
saveas(gcf, str, 'eps')
figure
plot(tout,T4(:,1));
ylabel('Wheel 4 Torque in Nm')
xlabel('Time in s')
title('Wheel 4 Torque Vs Time')
str = sprintf('Wheel_4_torque_zero_ig_w_c')
str1  = sprintf('%s%s',str,'.fig')
savefig(str1);
saveas(gcf, str, 'jpg')
saveas(gcf, str, 'eps')

g_on = 1;

% %%case 3 initial guess zero and gravity torque included in disturbance without control%%
w0 = [0; 0; 0; 0; 0; 0;]
Tdx=2e-3; Tdy=1e-4; Tdz=2e-3;
Td = [Tdx;Tdy;Tdz];
c_on = 0;
tspan = [0 20000]

[tout,wout] = rkf45(@(t,w) AstroSatDynamics(t,w,Td,c_on,M,J,Kp,Kd,g_on),tspan,w0,0.0000001)

figure
for k=1:3
    subplot(3,1,k)
    wout(:,k) = wrapToPi(wout(:,k));
    plot(tout,wout(:,k)/pi()*180)
    if k == 1
        str = sprintf('Yaw rate %s \n (deg/s)','\psidot');
        ylabel(str)
        xlabel('Time in s')
        str = sprintf('Euler Angle Rates in deg/s Vs Time with gravity gradient \n with given disturbance torque and without control');
        title(str);
    elseif k ==2
        str = sprintf('Roll rate %s \n (deg/s)','\phidot');
        ylabel(str)
        xlabel('Time in s')
    else
        str = sprintf('Pitch rate %s \n (deg/s)','\thetadot');
        ylabel(str)
        xlabel('Time in s')
    end
end

str = sprintf('Euler_rate_zero_ig_gg_wo_c')
str1  = sprintf('%s%s',str,'.fig')
savefig(str1);
saveas(gcf, str, 'jpg')
saveas(gcf, str, 'eps')

figure
for k=1:3
subplot(3,1,k)
wout(:,k+3) = wrapToPi(wout(:,k+3));
plot(tout,wout(:,k+3)/pi()*180)
    if k == 1
        str = sprintf('Yaw angle %s \n (deg)','\psi');
        ylabel(str)
        xlabel('Time in s')
        str = sprintf('Euler Angles in deg Vs Time with gravity gradient \n with given disturbance torque and without control');
        title(str);  
    elseif k ==2
        str = sprintf('Roll angle %s \n (deg)','\phi');
        ylabel(str)
        xlabel('Time in s')
    else
        str = sprintf('Pitch angle %s \n (deg)','\theta');
        ylabel(str)
        xlabel('Time in s')
    end
end

str = sprintf('Euler_angle_zero_ig_gg_wo_c')
str1  = sprintf('%s%s',str,'.fig')
savefig(str1);
saveas(gcf, str, 'jpg')
saveas(gcf, str, 'eps')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%case 4 initial guess zero and gravity torque included in disturbance with control%%
w0 = [0; 0; 0; 0; 0; 0;]
Tdx=2e-3; Tdy=1e-4; Tdz=2e-3;
Td = [Tdx;Tdy;Tdz];
Kpx=28; Kpy=28; Kpz=28;      %% Gain values Proportional
Kdx=0.95; Kdy=0.95; Kdz=0.95;   %% Gain values Derivative
Kp = [Kpx;Kpy;Kpz];
Kd = [Kdx;Kdy;Kdz];
Td = [Tdx;Tdy;Tdz];
c_on = 1;
tspan = [0 20000]

[tout,wout] = rkf45(@(t,w) AstroSatDynamics(t,w,Td,c_on,M,J,Kp,Kd,g_on),tspan,w0,0.0000001)
wout_w_gg = wout;
figure
for k=1:3
    subplot(3,1,k)
    wout(:,k) = wrapToPi(wout(:,k));
    plot(tout,wout(:,k)/pi()*180)
    if k == 1
        str = sprintf('Yaw rate %s \n (deg/s)','\psidot');
        ylabel(str)
        xlabel('Time in s')
        str = sprintf('Euler Angle Rates in deg/s Vs Time with gravity gradient \n with given disturbance torque and with control');
        title(str);
    elseif k ==2
        str = sprintf('Roll rate %s \n (deg/s)','\phidot');
        ylabel(str)
        xlabel('Time in s')
    else
        str = sprintf('Pitch rate %s \n (deg/s)','\thetadot');
        ylabel(str)
        xlabel('Time in s')
    end
end

str = sprintf('Euler_rate_zero_ig_gg_w_c')
str1  = sprintf('%s%s',str,'.fig')
savefig(str1);
saveas(gcf, str, 'jpg')
saveas(gcf, str, 'eps')

figure
for k=1:3
subplot(3,1,k)
wout(:,k+3) = wrapToPi(wout(:,k+3));
plot(tout,wout(:,k+3)/pi()*180)
    if k == 1
        str = sprintf('Yaw angle %s \n (deg)','\psi');
        ylabel(str)
        xlabel('Time in s')
        str = sprintf('Euler Angles in deg Vs Time with gravity gradient \n with given disturbance torque and with control');
        title(str);  
    elseif k ==2
        str = sprintf('Roll angle %s \n (deg)','\phi');
        ylabel(str)
        xlabel('Time in s')
    else
        str = sprintf('Pitch angle %s \n (deg)','\theta');
        ylabel(str)
        xlabel('Time in s')
    end
end

str = sprintf('Euler_angle_zero_ig_gg_w_c')
str1  = sprintf('%s%s',str,'.fig')
savefig(str1);
saveas(gcf, str, 'jpg')
saveas(gcf, str, 'eps')
Rphi = 0;
Rpsy = 0;
Rtheta = 0;
Tcx=Kpx*(-Rphi+wout(:,4))+Kdx*wout(:,1);
    Tcy=Kpy*(-Rpsy+wout(:,5))+Kdy*wout(:,2);
    Tcz=Kpz*(-Rtheta+wout(:,6))+Kdz*wout(:,3);
T1=(sqrt(3)*(Tcx)+sqrt(3)*(Tcy)+sqrt(3)*(Tcz))/4;
T2=(-sqrt(3)*(Tcx)-sqrt(3)*(Tcy)+sqrt(3)*(Tcz))/4;
T3=(-sqrt(3)*(Tcx)+sqrt(3)*(Tcy)-sqrt(3)*(Tcz))/4;
T4=(sqrt(3)*(Tcx)-sqrt(3)*(Tcy)-sqrt(3)*(Tcz))/4;
figure
plot(tout,T1(:,1));
ylabel('Wheel 1 Torque in Nm')
xlabel('Time in s')
title('Wheel 1 Torque Vs Time (GG included)')
str = sprintf('Wheel_1_torque_zero_ig_gg_w_c')
str1  = sprintf('%s%s',str,'.fig')
savefig(str1);
saveas(gcf, str, 'jpg')
saveas(gcf, str, 'eps')
figure
plot(tout,T2(:,1));
ylabel('Wheel 2 Torque in Nm')
xlabel('Time in s')
title('Wheel 2 Torque Vs Time (GG included)')
str = sprintf('Wheel_2_torque_zero_ig_gg_w_c')
str1  = sprintf('%s%s',str,'.fig')
savefig(str1);
saveas(gcf, str, 'jpg')
saveas(gcf, str, 'eps')
figure
plot(tout,T3(:,1));
ylabel('Wheel 3 Torque in Nm')
xlabel('Time in s')
title('Wheel 3 Torque Vs Time (GG included)')
str = sprintf('Wheel_3_torque_zero_ig_gg_w_c')
str1  = sprintf('%s%s',str,'.fig')
savefig(str1);
saveas(gcf, str, 'jpg')
saveas(gcf, str, 'eps')
figure
plot(tout,T4(:,1));
ylabel('Wheel 4 Torque in Nm')
xlabel('Time in s')
title('Wheel 4 Torque Vs Time (GG included)')
str = sprintf('Wheel_4_torque_zero_ig_gg_w_c')
str1  = sprintf('%s%s',str,'.fig')
savefig(str1);
saveas(gcf, str, 'jpg')
saveas(gcf, str, 'eps')


