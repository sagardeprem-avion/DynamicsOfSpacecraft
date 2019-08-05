function wdot = AstroSatDynamics(t,w,Td,c_on,M,J,Kp,Kd,g_on)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
wdot=zeros(6,1);

Jxx=J(1,1); Jxy=J(1,2); Jxz=J(1,3); Jyx=J(2,1); Jyy=J(2,2); Jyz=J(2,3); Jzx=J(3,1); Jzy=J(3,2); Jzz=J(3,3);
                                  %%Mass of the SpaceCraft
Rphi=0;                                  %%Reference input signal for Phi angle
Rtheta=0;                                %%Reference input signal for theta angle
Rpsy=0;                                  %%Reference input signal for psy angle
 
Tdx  = Td(1);
Tdy = Td(2);
Tdz = Td(3);
if c_on == 1
    Kpx = Kp(1);
    Kpy = Kp(2);
    Kpz = Kp(3);
    Kdx = Kd(1);
    Kdy = Kd(2);
    Kdz = Kd(3);
    Tcx=Kpx*(Rphi-w(4))-Kdx*w(1);
    Tcy=Kpy*(Rpsy-w(5))-Kdy*w(2);
    Tcz=Kpz*(Rtheta-w(6))-Kdz*w(3);
else
    Tcx= 0;
    Tcy = 0;
    Tcz = 0;
end

%%%Wheel torgues (in wheel frame)

T1=(sqrt(3)*(Tcx)+sqrt(3)*(Tcy)+sqrt(3)*(Tcz))/4;
T2=(-sqrt(3)*(Tcx)-sqrt(3)*(Tcy)+sqrt(3)*(Tcz))/4;
T3=(-sqrt(3)*(Tcx)+sqrt(3)*(Tcy)-sqrt(3)*(Tcz))/4;
T4=(sqrt(3)*(Tcx)-sqrt(3)*(Tcy)-sqrt(3)*(Tcz))/4;

%% Wheel configuration details(You can use any general wheel config, 
%%but here  it is coded assuming a tetrahedron wheel configuration)

%%%for the purpose of momentum dumping
k1=[1/sqrt(3);1/sqrt(3);1/sqrt(3)];
k2=[-1/sqrt(3);-1/sqrt(3);1/sqrt(3)];
k3=[-1/sqrt(3);1/sqrt(3);-1/sqrt(3)];
k4=[1/sqrt(3);-1/sqrt(3);-1/sqrt(3)];    
a1=cross([1;0;0],k1)/norm(cross([1;0;0],k1));
a2=cross([1;0;0],k2)/norm(cross([1;0;0],k2));
a3=cross([1;0;0],k3)/norm(cross([1;0;0],k3));
a4=cross([1;0;0],k4)/norm(cross([1;0;0],k4));
phi1=acos(dot(k1,[1;0;0])/norm(k1)*norm([1;0;0]));
phi2=acos(dot(k2,[1;0;0])/norm(k2)*norm([1;0;0]));
phi3=acos(dot(k3,[1;0;0])/norm(k3)*norm([1;0;0]));
phi4=acos(dot(k4,[1;0;0])/norm(k4)*norm([1;0;0]));
I=[1 0 0;0 1 0;0 0 1];
a1star=[0 -a1(3) a1(2);a1(3) 0 -a1(1);-a1(2) a1(1) 0];
a2star=[0 -a2(3) a2(2);a2(3) 0 -a2(1);-a2(2) a2(1) 0];
a3star=[0 -a3(3) a3(2);a3(3) 0 -a3(1);-a3(2) a3(1) 0];
a4star=[0 -a4(3) a4(2);a4(3) 0 -a4(1);-a4(2) a4(1) 0];
C1=I*cos(phi1)+(1-cos(phi1))*(a1*a1')-sin(phi1)*a1star;
C2=I*cos(phi2)+(1-cos(phi2))*(a2*a2')-sin(phi2)*a2star;
C3=I*cos(phi3)+(1-cos(phi3))*(a3*a3')-sin(phi3)*a3star;
C4=I*cos(phi4)+(1-cos(phi4))*(a4*a4')-sin(phi4)*a4star;
Iw=[0.1 0 0;0 0.1 0; 0 0 0.1];          %% Moment of inertia matrix for the wheels
Ix=Iw(1,1); Iy=Iw(2,2); Iz=Iw(3,3);
a1=C1(1,1); b1=C1(1,2); c1=C1(1,3); p1=C1(2,1); q1=C1(2,2); r1=C1(2,3); x1=C1(3,1); y1=C1(3,2); z1=C1(3,3);
a2=C2(1,1); b2=C2(1,2); c2=C2(1,3); p2=C2(2,1); q2=C2(2,2); r2=C2(2,3); x2=C2(3,1); y2=C2(3,2); z2=C2(3,3);
a3=C3(1,1); b3=C3(1,2); c3=C3(1,3); p3=C3(2,1); q3=C3(2,2); r3=C3(2,3); x3=C3(3,1); y3=C3(3,2); z3=C3(3,3);
a4=C4(1,1); b4=C4(1,2); c4=C4(1,3); p4=C4(2,1); q4=C4(2,2); r4=C4(2,3); x4=C4(3,1); y4=C4(3,2); z4=C4(3,3);

%%%%%Gravity Gradient%%%%
if g_on == 1
    Tgy = 3*(1.0741e-3)^2*(Jxx-Jzz)*w(5);
    Tgz = 3*(1.0741e-3)^2*(Jxx-Jyy)*w(6);
else
    Tgy = 0;
    Tgz = 0;
end
X=Tcx+Tdx+(w(3)*((Jyx*w(1))+(Jyy*w(2))+(Jyz*w(3))))-(w(2)*((Jzx*w(1))+(Jzy*w(2))+(Jzz*w(3))));
Y=Tcy+Tdy+Tgy-(w(3)*((Jxx*w(1))+(Jxy*w(2))+(Jxz*w(3))))+(w(1)*((Jzx*w(1))+(Jzy*w(2))+(Jzz*w(3))));
Z=Tcz+Tdz+Tgz+(w(2)*((Jxx*w(1))+(Jxy*w(2))+(Jxz*w(3))))-(w(1)*((Jyx*w(1))+(Jyy*w(2))+(Jyz*w(3))));

%%Dynamic equations are written separating the SpaceCraft and the Wheel
P=(X-(Jxy*wdot(2))-(Jxz*wdot(3)))/Jxx;
Q=(Y-(Jyx*wdot(1))-(Jyz*wdot(3)))/Jyy;
R=(Z-(Jzx*wdot(1))-(Jzy*wdot(2)))/Jzz;
wdot(1) = P
wdot(2) = Q
wdot(3) = R
wdot(4)=w(1)-((cos(w(4)*tan(w(6))))*w(2))+((sin(w(4)*tan(w(6))))*w(3));
wdot(5)=((cos(w(4))*sec(w(6)))*w(2))-((sin(w(4))*sec(w(6)))*w(3));
wdot(6)= ((sin(w(4)))*w(2))+((cos(w(4)))*w(3));

end

