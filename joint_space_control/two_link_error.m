function dedt = two_link_error(t,x)
% x = [e1;e2;edot1;edot2];

global kp;
global kd;

dedt = [x(3) ; x(4); -kp*x(1) - kd*x(3);-kp*x(2) - kd*x(4)];