function dydt = two_link_env(t,y)

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
global fa;
global Env_X;
global Env_Y;
global Ja;

% Environment Check (Contact Force Generation)
end_effector_x = l1*cos(y(1)) + l2*cos(y(1)+y(2));
end_effector_xdot = -(l1*sin(y(1)) + l2*sin(y(1)+y(2)))*y(3) - l2*sin(y(1)+y(2))*y(4);
Ja = [ -l1*sin(y(1))-l2*sin(y(1)+y(2)) , -l2*sin(y(1)+y(2));
        l1*cos(y(1))+l2*cos(y(1)+y(2)) , l2*cos(y(1)+y(2)) ];
    
if end_effector_x < Env_X(1) % Contact 
    fa(1) = 100000*(Env_X(1) - end_effector_x) - 20*end_effector_xdot;
    fa(2) = 0;
    fa = [ fa(1) ; fa(2) ];
else 
    fa(1) = 0;
    fa(2) = 0;
    fa = [ fa(1) ; fa(2) ];
end;

H = [m1*l1*l1 + m2*l1*l1 + m2*l2*l2 + 2*m2*l1*l2*cos(y(2)) , m2*l2*l2 + m2*l1*l2*cos(y(2)) ;
        m2*l2*l2 + m2*l1*l2*cos(y(2)) , m2*l2*l2 ];
C = [ -m2*l1*l2*y(4)*sin(y(2)) , -m2*l1*l2*(y(3)+y(4))*sin(y(2)) ;
        m2*l1*l2*y(3)*sin(y(2)) , 0 ];
G = [ (m1+m2)*g*l1*cos(y(1)) + m2*g*l2*cos(y(1)+y(2)) ; m2*g*l2*cos(y(1)+y(2)) ];
   
temp1 = [y(3) ; y(4)] ;
temp2 = (H^(-1))*(-C*temp1 - G + Ja'*fa + u);

dydt = [y(3); y(4) ; temp2(1) ; temp2(2) ];