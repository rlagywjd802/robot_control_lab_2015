function dydt = two_link_PD(t,y)

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

% Write the equation of motion of two-link robot here.
% Let y(1) = q1, y(2) = q2, y(3) = q1_dot, y(4) = q2_dot.
%
% Your code here ==>
Kp = 100;
Kd = 1;
% final configuration
qd = [pi/3; pi/6];  

H = [(m1+m2)*l1*l1 + m2*l2*l2 + 2*m2*l1*l2*cos(y(2)), m2*l2*l2 + m2*l1*l2*cos(y(2));
    m2*l2*l2 + m2*l1*l2*cos(y(2))                     , m2*l2*l2                    ];

C = [-m2*l1*l2*(2*y(3)*y(4) + y(4)*y(4))*sin(y(2));
    m2*l1*l2*y(3)*y(3)*sin(y(2))];

G = [(m1+m2)*g*l1*cos(y(1)) + m2*g*l2*cos(y(1)+y(2));
    m2*g*l2*cos(y(1)+y(2))];

u(1) = Kp*(qd(1)-y(1)) - Kd*y(3) ;
u(2) = Kp*(qd(2)-y(2)) - Kd*y(4) ;

Y = H \ (-C-G+u);

dydt = [y(3); y(4) ; Y(1) ; Y(2)];