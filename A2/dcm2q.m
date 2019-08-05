function q = dcm2q(c)
eta = 0.5*sqrt( 1 + trace(c) );
q1 = (c(2,3) - c(3,2) )/4/eta;
q2 = (c(3,1) - c(1,3) )/4/eta;
q3 = (c(1,2) - c(2,1) )/4/eta;
q = [ eta q1 q2 q3 ]';
end