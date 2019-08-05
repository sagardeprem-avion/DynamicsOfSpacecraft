 function wrates=wrates(t,w0 )
 Ixx=98;
Iyy=102;
Izz=150;

% % Case 1 major axis spin
% wrates =[(Iyy-Izz)*w0(2)*w0(3)/Ixx; ...
%            (Izz-Ixx)*w0(1)*w0(3)/Iyy; ...
%           (Ixx-Iyy)*w0(1)*w0(2)/Izz ];
% Case 2 minor axis spin
wrates =[(Iyy-Izz)*w0(2)*w0(3)/Ixx; ...
        ((Izz-Ixx)*w0(1)*w0(3)-1e-5)/Iyy; ...
        (Ixx-Iyy)*w0(1)*w0(2)/Izz ];

 end