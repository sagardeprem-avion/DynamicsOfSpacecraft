function qrates=qrates(t,Q)
qrates =0.5*[Q(1) Q(4) -Q(3) Q(2); Q(2) Q(3) Q(4) -Q(1);...
         Q(3) -Q(2) Q(1) Q(4); Q(4) -Q(1) -Q(2) -Q(3)]* [0 sin(0.1*t); 0.01; cos(0.1*t)]*20*pi/180;

end