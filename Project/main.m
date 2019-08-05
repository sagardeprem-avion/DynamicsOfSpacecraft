clear
clc
%%

e0=[0 ; 0; 0]     ;     % [phi theta psi] 1-2-3 axis angles    
[q01,q02,q03,q04]=euler2quat(0,0,0,'yzx');  % q01 is the scalar
q0 = [q02 q03 q04 q01];

w0=[0 0 0 0] ;       % [0 wx wy wz] initial angular velocity
% tspan           = [0 20000];            % [startTime endTime]
% [tout, wout]    = ode45(@wrate, tspan, w0,   0.00000001);  

qdot = [];
qi=q0;
q=[];
 
for i=1:200
       
   [tout, wout]    = ode45(@wrate, [0 i*0.5], w0,  0.00000001);
   qdoti=0.5*qmatrix(qi)*[0 ; wout(i,:)'];
   [tout, qout]    = ode45(@qrate, [0 i*0.5], q0,  0.00000001);
   qdot = [qdot; qdoti];
   
   q = [q; qi];
end


