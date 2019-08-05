clear
clc
%%
r= [4172.39512860  -854.25163879  5538.75384795]*1000;
v= [5.3839064673   -2.8358389294   -4.4805571591]*1000;

yaw=-r/norm(r);
pitch=-cross(r,v);
pitch=pitch/norm(pitch);
roll=-cross(yaw,pitch);
Coi=[yaw ; roll ;pitch];
 
Coi=Coi/det(Coi) 
 %%
Qoi=dcm2quat(Coi);
Cbo=[0.9958533 0.088210 -0.022251; -0.087126 0.995147 0.045733 ; 0.026177 -0.043604 0.998706];
Qbo=dcm2quat(Cbo);
Csb = [-0.731354 0 -0.681998; 0.222037 0.945519 -0.238105; 0.644842 -0.325568 -0.691509]
Qsb=dcm2quat(Csb);
quatmultiply(quatmultiply(Qoi,Qbo),Qsb)
quat2dcm(ans)
Csi=Csb*Cbo*Coi  