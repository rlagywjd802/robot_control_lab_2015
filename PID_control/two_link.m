function dydt = two_link(t,y)

global H;
global C;
global G;
global u;
global temp1;
global temp2;
global m1;
global m2;
global l1;
global l2;
global g;

H = [m1*l1*l1 + m2*l1*l1 + m2*l2*l2 + 2*m2*l1*l2*cos(y(2)) , m2*l2*l2 + m2*l1*l2*cos(y(2)) ;
        m2*l2*l2 + m2*l1*l2*cos(y(2)) , m2*l2*l2 ];
C = [ -m2*l1*l2*y(4)*sin(y(2)) , -m2*l1*l2*(y(3)+y(4))*sin(y(2)) ;
        m2*l1*l2*y(3)*sin(y(2)) , 0 ];
G = [ (m1+m2)*g*l1*cos(y(1)) + m2*g*l2*cos(y(1)+y(2)) ; m2*g*l2*cos(y(1)+y(2)) ];
   
temp1 = [y(3) ; y(4)] ;
temp2 = (H^(-1))*(-C*temp1 - G + u);

dydt = [y(3); y(4) ; temp2(1) ; temp2(2) ];