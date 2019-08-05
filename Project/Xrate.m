function   Xdot = Xrate(t,X,w,Iw,J,w0,Td)

A1     = [1 -1 -1 1]*diag(Iw)*w'/J(1)/sqrt(3);
A2     = [1 -1 1 -1]*diag(Iw)*w'/J(1)/sqrt(3);
B1     = [1 -1 -1 1]*diag(Iw)*w'/J(2)/sqrt(3);
B2     = [1 1 -1 -1]*diag(Iw)*w'/J(2)/sqrt(3);
D1     = [1 -1 -1 1]*diag(Iw)*w'/J(3)/sqrt(3);
D2     = [1 -1 -1 1]*diag(Iw)*w'/J(3)/sqrt(3);
G     = 3*w0^2*(J(2)-J(1))/J(3);
Td   = Td./J;
% state transition matrix
I    = [0 -A2 A1 0 0 0
        B2 0 -B1 0 0 0
        -D2 D1 0 G 0 0
        1 0 0 0 0 0 
        0 1 0 0 0 0
        0 0 1 0 0 0];
    
Xdot = I*X + [0; 0; 0; Td];

end