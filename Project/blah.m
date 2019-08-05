clear
clc
w0 = zeros(6,1);
tspan           = [0 2000];            % [startTime endTime]
[tout, wout]    = rkf45(@AstroSatDynamics, tspan, w0,   0.01);
plot(tout, wout)