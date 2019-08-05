
theta0=0;
[tout, wout]    = ode45(@thetaDot, tspan, theta0,   0.00000001);  
