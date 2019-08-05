n=cos(45*pi*0.5/180);
a=[1 1 1]*1/sqrt(3);
e=sin(45*pi*0.5/180)*a;
C=(2*n^2-1)*eye(3)+2*e'*e-2*n*[0 -a(3) a(2); a(3) 0 -a(1); -a(2) a(1) 0];
phi=atan(C(2,3)/C(3,3))*180/pi
theta=-asin(C(1,3))*180/pi
psi=atan(C(1,2)/C(1,1))*180/pi
 