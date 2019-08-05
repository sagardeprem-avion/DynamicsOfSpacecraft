function q = qmul(p,q)
a = [ p(4) -p(3) p(2) p(1)
      p(3) p(4) -p(1) p(2)
      -p(2) p(1) p(4) p(3)
      -p(1) -p(2) -p(3) p(4) ];
[m n] = size(q);
q = a*q;
end