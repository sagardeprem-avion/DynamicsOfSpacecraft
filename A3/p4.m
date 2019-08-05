clear
clc

syms m l
Jcube=100*(1^2+1^2)/12*eye(3);
J=Jcube+ m*(mJ(60*pi/180, l)+mJ(120*pi/180, l)+mJ(240*pi/180, l)+mJ(300*pi/180, l))

%%
l=sqrt(1e-5/((2*pi/6/3600)^2*5*pi*6/180))





 