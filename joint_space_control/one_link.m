function dydt = one_link(t,y)

global H;
global C;
global G;
global u;
global temp1;
global temp2;
global m1;
global l1;
global g;

H = m1*l1*l1;
C = 0;
G = m1*g*l1*cos(y(1));
    

dydt = [y(2); (H^(-1))*(-C*y(2) - G + u) ];