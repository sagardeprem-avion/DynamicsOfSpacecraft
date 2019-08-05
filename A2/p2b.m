syms t1 t2 t3 p

A=[cos(t2) sin(t2) 0; -sin(t2) cos(t2) 0; 0 0 1]
B=[cos(t1) 0 -sin(t1); 0 1 0; sin(t1) 0 cos(t1) ]
A*B*[0; p; 0 ]