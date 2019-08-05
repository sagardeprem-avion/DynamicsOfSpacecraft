syms t1 t2 t3
C=[cos(t3) 0 -sin(t3);
    0 1 0;
    sin(t3) 0 cos(t3)];
B=[cos(t2) sin(t2) 0;
    -sin(t2) cos(t2) 0;
    0 0 1];
A=[cos(t1) 0 -sin(t1);
    0 1 0;
    sin(t1) 0 cos(t1)] ;
Cbar=C*B*A