function wdot = AstroSatQuaternions(t,w)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
wdot=zeros(11,1);
J=[1763 -52 -16;-52 1591 25;-16 25 1185];%%Moment of inertia matrix of the SpaceCraft
Jxx=J(1,1); Jxy=J(1,2); Jxz=J(1,3); Jyx=J(2,1); Jyy=J(2,2); Jyz=J(2,3); Jzx=J(3,1); Jzy=J(3,2); Jzz=J(3,3);
M=1542;                                  %%Mass of the SpaceCraft
Re1=0;                                  %%Reference input signal for Phi angle
Re2=0;                                %%Reference input signal for theta angle
Re3=0;                                  %%Reference input signal for psy angle
Kpx=100; Kpy=100; Kpz=100;                     %% Gain values Proportional
Kdx=10; Kdy=10; Kdz=10;                     %% Gain values Derivative 
Tcx=2*Kpx*(Re1-w(4))*(w(11))+Kdx*wdot(1);
Tcy=2*Kpy*(Re2-w(5))*(w(11))+Kdy*wdot(2);
Tcz=2*Kpz*(Re3-w(6))*(w(11))+Kdz*wdot(3);
Tdx=0; Tdy=0; Tdz=0;                     %% Disturbance Torques
%% Wheel configuration details(You can use any general wheel config, but here  it is coded assuming a tetrahedron wheel configuration)
k1=[1/sqrt(3);1/sqrt(3);1/sqrt(3)];
k2=[-1/sqrt(3);-1/sqrt(3);1/sqrt(3)];
k3=[-1/sqrt(3);1/sqrt(3);-1/sqrt(3)];
k4=[1/sqrt(3);-1/sqrt(3);-1/sqrt(3)];    
A1=cross([1;0;0],k1)/norm(cross([1;0;0],k1));
A2=cross([1;0;0],k2)/norm(cross([1;0;0],k2));
A3=cross([1;0;0],k3)/norm(cross([1;0;0],k3));
A4=cross([1;0;0],k4)/norm(cross([1;0;0],k4));
phi1=acos(dot(k1,[1;0;0])/norm(k1)*norm([1;0;0]));
phi2=acos(dot(k2,[1;0;0])/norm(k2)*norm([1;0;0]));
phi3=acos(dot(k3,[1;0;0])/norm(k3)*norm([1;0;0]));
phi4=acos(dot(k4,[1;0;0])/norm(k4)*norm([1;0;0]));
I=[1 0 0;0 1 0;0 0 1];
A1star=[0 -A1(3) A1(2);A1(3) 0 -A1(1);-A1(2) A1(1) 0];
A2star=[0 -A2(3) A2(2);A2(3) 0 -A2(1);-A2(2) A2(1) 0];
A3star=[0 -A3(3) A3(2);A3(3) 0 -A3(1);-A3(2) A3(1) 0];
A4star=[0 -A4(3) A4(2);A4(3) 0 -A4(1);-A4(2) A4(1) 0];
C1=I*cos(phi1)+(1-cos(phi1))*(A1*A1')-sin(phi1)*A1star;
C2=I*cos(phi2)+(1-cos(phi2))*(A2*A2')-sin(phi2)*A2star;
C3=I*cos(phi3)+(1-cos(phi3))*(A3*A3')-sin(phi3)*A3star;
C4=I*cos(phi4)+(1-cos(phi4))*(A4*A4')-sin(phi4)*A4star;
Iw=[0.1 0 0;0 0.05 0; 0 0 0.05];          %% Moment of inertia matrix for the wheels
Ix=Iw(1,1); Iy=Iw(2,2); Iz=Iw(3,3);
a1=C1(1,1); b1=C1(1,2); c1=C1(1,3); p1=C1(2,1); q1=C1(2,2); r1=C1(2,3); x1=C1(3,1); y1=C1(3,2); z1=C1(3,3);
a2=C2(1,1); b2=C2(1,2); c2=C2(1,3); p2=C2(2,1); q2=C2(2,2); r2=C2(2,3); x2=C2(3,1); y2=C2(3,2); z2=C2(3,3);
a3=C3(1,1); b3=C3(1,2); c3=C3(1,3); p3=C3(2,1); q3=C3(2,2); r3=C3(2,3); x3=C3(3,1); y3=C3(3,2); z3=C3(3,3);
a4=C4(1,1); b4=C4(1,2); c4=C4(1,3); p4=C4(2,1); q4=C4(2,2); r4=C4(2,3); x4=C4(3,1); y4=C4(3,2); z4=C4(3,3);
X=Tcx+Tdx+(w(3)*((Jyx*w(1))+(Jyy*w(2))+(Jyz*w(3))))-(w(2)*((Jzx*w(1))+(Jzy*w(2))+(Jzz*w(3))));
Y=Tcy+Tdy-(w(3)*((Jxx*w(1))+(Jxy*w(2))+(Jxz*w(3))))+(w(1)*((Jzx*w(1))+(Jzy*w(2))+(Jzz*w(3))));
Z=Tcz+Tdz+(w(2)*((Jxx*w(1))+(Jxy*w(2))+(Jxz*w(3))))-(w(1)*((Jyx*w(1))+(Jyy*w(2))+(Jyz*w(3))));

%%Dynamic equations are written separating the SpaceCraft and the Wheel
wdot(1)=(((Y*Jzy)-(Z*Jyy))-((Jyz*Jzy)-(Jzz*Jyy))*wdot(3))/((Jyx*Jzy)-(Jzx*Jyy));
wdot(2)=(X-(Jxz*wdot(3))-(Jxx*wdot(1)))/Jyy;
wdot(3)=(((X*Jyy)-(Y*Jxy))*((Jyx*Jzy)-(Jzx*Jyy))-((Y*Jzy)-(Z*Jyy))*((Jxx*Jyy)-(Jyx*Jxy)))/(((Jyy*Jxz)-(Jyz*Jxy))*((Jyx*Jzy)-(Jzx*Jyy))-((Jyz*Jzy)-(Jzz*Jyy))*((Jxx*Jyy)-(Jyx*Jxy)));
wdot(4)=0.5*(w(1)*w(11)+w(3)*w(5)-w(2)*w(6));
wdot(5)=0.5*(w(2)*w(11)-w(3)*w(4)+w(1)*w(6));
wdot(6)=0.5*(w(3)*w(11)+w(2)*w(4)-w(1)*w(5));
T1=(sqrt(3)*(-Tcx)+sqrt(3)*(-Tcy)+sqrt(3)*(-Tcz))/4;
T2=(-sqrt(3)*(-Tcx)-sqrt(3)*(-Tcy)+sqrt(3)*(-Tcz))/4;
T3=(-sqrt(3)*(-Tcx)+sqrt(3)*(-Tcy)-sqrt(3)*(-Tcz))/4;
T4=(sqrt(3)*(-Tcx)-sqrt(3)*(-Tcy)-sqrt(3)*(-Tcz))/4;
wdot(7)=(T1-(Ix*((a1*wdot(1))+(b1*wdot(2))+(c1*wdot(3))))+(((x1*w(1))+(y1*w(2))+(z1*w(3)))*(Iy*((p1*w(1))+(q1*w(2))+(r1*w(3)))))-(((p1*w(1))+(q1*w(2))+(r1*w(3)))*(Iz*((x1*w(1))+(y1*w(2))+(z1*w(3))))))/Ix;
wdot(8)=(T2-(Ix*((a2*wdot(1))+(b2*wdot(2))+(c2*wdot(3))))+(((x2*w(1))+(y2*w(2))+(z2*w(3)))*(Iy*((p2*w(1))+(q2*w(2))+(r2*w(3)))))-(((p2*w(1))+(q2*w(2))+(r2*w(3)))*(Iz*((x2*w(1))+(y2*w(2))+(z2*w(3))))))/Ix;
wdot(9)=(T3-(Ix*((a3*wdot(1))+(b3*wdot(2))+(c3*wdot(3))))+(((x3*w(1))+(y3*w(2))+(z3*w(3)))*(Iy*((p3*w(1))+(q3*w(2))+(r3*w(3)))))-(((p3*w(1))+(q3*w(2))+(r3*w(3)))*(Iz*((x3*w(1))+(y3*w(2))+(z3*w(3))))))/Ix;
wdot(10)=(T4-(Ix*((a4*wdot(1))+(b4*wdot(2))+(c4*wdot(3))))+(((x4*w(1))+(y4*w(2))+(z4*w(3)))*(Iy*((p4*w(1))+(q4*w(2))+(r4*w(3)))))-(((p4*w(1))+(q4*w(2))+(r4*w(3)))*(Iz*((x4*w(1))+(y4*w(2))+(z4*w(3))))))/Ix;

end



