
r = [4172.39512860, -854.25163879, 5538.75384795];
v = [5.3839064673, -2.8358389294, -4.4805571591];
    
yaw = -r / norm(r);
pitch = -cross(r,v)/norm(cross(r,v));
roll = cross(pitch,yaw);

coi = [ yaw
        roll
        pitch ]
qoi = dcm2q(coi)

cbo = [ 0.9958533 0.088210 -0.022251
        -0.087126 0.995147 0.045733
        0.026177 -0.043604 0.998706];
qbo = dcm2q(cbo);
    
csb = [ -0.731354 0.0 -0.681998
        0.222037 0.945519 -0.238105
        0.644842 -0.325568 -0.691509 ];
qsb = dcm2q(csb);

csi = csb * cbo * coi 
% qsb = dcm2q(csi)
% 
% qmul(qmul(qoi,qbo),qsb)

