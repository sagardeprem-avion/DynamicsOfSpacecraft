eul=[40 -30 80]*pi/180
% rotm = eul2rotm(eul)
C= (eul2rotm(eul,'ZYX'))'

n=0.5*(trace(C)+1)^0.5
e=[C(2,3)-C(3,2) C(3,1)-C(1,3) C(1,2)-C(2,1)]*1/4/n

theta=2*acos(n)*180/pi
a=e/norm(e)