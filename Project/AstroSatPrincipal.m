function wdot = AstroSatPrincipal(t,w)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
wdot=zeros(6,1);
J=[1763 0 0;0 1591 0;0 0 1185];    %%Moment of Inertia Matrix of the system
Jxx=J(1,1); Jyy=J(2,2); Jzz=J(3,3);
Rphi=0; Rtheta=0; Rpsi=0;
Kpx=2000; Kpy=2000; Kpz=2000;
Kdx=0; Kdy=0; Kdz=0;
wdot(1)=(Kpx*(Rphi-w(4))+Kdx*(wdot(4))+((Jyy-Jzz)*w(2)*w(3)))/Jxx;
wdot(2)=(Kpy*(Rtheta-w(5))+Kdy*(wdot(5))+((Jzz-Jxx)*w(1)*w(3)))/Jyy;
wdot(3)=(Kpz*(Rpsi-w(6))+Kdz*(wdot(6))+((Jxx-Jyy)*w(2)*w(1)))/Jzz;
wdot(4)=w(1)-((cos(w(4))*tan(w(5)))*w(2))+((sin(w(4))*tan(w(5)))*w(3));
wdot(5)=((sin(w(4)))*w(2))+((cos(w(4))*w(3)));
wdot(6)=((cos(w(4)))*sec(w(5))*w(2))-((sin(w(4))*sec(w(5))*w(3)));


end

