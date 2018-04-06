clear
clear all
close all
home

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
global tf;
global t0;

% Robot Parameters 
m1 = 0.5;
m2 = 0.5;
l1 = 0.15;
l2 = 0.15;
g = 9.806;

% Control frequency is defined as follows %
s_time = 0.005;

% Terminal time %
t0=0;           tf=4;

% Variable for converting RADIAN -> DEGREE or DEGREE -> RADIAN
R2D = 180/pi;
D2R = pi/180;

% PD gain
kd = 5; lamb = 2;   % two_link
%kd = 5; lamb = 2;   % two_link_df
kd = kd*0.1; lamb = lamb*0.1;
lambda = [lamb,0;0,lamb];
Kd = [kd, 0;0,kd];

% Initial Configurations %
x = [0.15/sqrt(2); 0.15 + 0.15/sqrt(2)];    x_d = [-0.15;0];   
q = [pi/4;pi/4];
qdot = [0;0];   qddot = [0;0];
xdot = [0;0];   xddot = [0;0];
       
% Robot trajectories %     
x_0 = x;            x_f = x_d;
xdot_0 = xdot;      xdot_f = [0;0];
xddot_0 = xddot;    xddot_f = [0;0];
[a10,a11,a12,a13,a14,a15]=QuinticPolynomialPath(x_0(1), xdot_0(1), xddot_0(1), x_f(1), xdot_f(1), xddot_f(1));        
[a20,a21,a22,a23,a24,a25]=QuinticPolynomialPath(x_0(2), xdot_0(2), xddot_0(2), x_f(2), xdot_f(2), xddot_f(2));
    
% Iteration numbers
n=1;        %Iterator for main loop
n_trj=1;    %Iterator for trajectories of joint points

% Initial Input %
u = [0;0];   
q_d = [0;0];
qdot_d = [0;0];

% Plot Setting %
figure(1)
title('Animation')
grid
hold on
axis([-0.4 0.4 -0.4 0.4]);
   Ax1 = [0, l1];
   Ay1 = [0, 0];
   Ax2 = [l1, l1+l2];
   Ay2 = [0, 0];
   p1 = line(Ax1,Ay1,'EraseMode','xor','LineWidth',[5],'Color','b');
   p2 = line(Ax2,Ay2,'EraseMode','xor','LineWidth',[5],'Color','c');
   
% Robot Implementation 
for i = t0 : s_time : tf
  
    % Forward Dynamics Part for Motion Generation %
    [t,y] = ode45('two_link',[0, s_time] , [q(1); q(2); qdot(1); qdot(2)] );
    %[t,y] = ode45('two_link_df',[0, s_time] , [q(1); q(2); qdot(1); qdot(2)] );

    index = size(y);
    q(1) = y(index(1), 1);
    q(2) = y(index(1), 2);
    qdot(1) = y(index(1), 3);
    qdot(2) = y(index(1), 4);
    qddot(1) = temp2(1);
    qddot(2) = temp2(2);
    
    q = [q(1);q(2)];
    qdot = [qdot(1);qdot(2)];
    qddot = [qddot(1);qddot(2)];
    
    
    % Jacobian
    Ja = [-l1*sin(q(1))-l2*sin(q(1)+q(2)) -l2*sin(q(1)+q(2));
           l1*cos(q(1))+l2*cos(q(1)+q(2))  l2*cos(q(1)+q(2))]; 
    Jadot = [-l1*cos(q(1))*qdot(1)-l2*cos(q(1)+q(2))*(qdot(1)+qdot(2)), -l2*cos(q(1)+q(2))*(qdot(1)+qdot(2));
             -l1*sin(q(1))*qdot(1)-l2*sin(q(1)+q(2))*(qdot(1)+qdot(2)), -l2*sin(q(1)+q(2))*(qdot(1)+qdot(2))];  
    %Jadot = (Ja - Ja_old)/s_time;
    
    
    % Forward Kinematics
	x1 = l1*cos(q(1));
	y1 = l1*sin(q(1));
	x2 = x1 + l2*cos(q(1)+q(2));
	y2 = y1 + l2*sin(q(1)+q(2));
    
    x = [x2;y2];                    % end-effector position
    xdot = Ja*qdot;                 % end-effector velocity
    xddot = Ja*qddot + Jadot*qdot;  % end-effector acceleration
    
         
    % Desired Trajectory 
    t=i;    
    x_d(1)=a10+a11*t+a12*t^2+a13*t^3+a14*t^4+a15*t^5;
    x_d(2)=a20+a21*t+a22*t^2+a23*t^3+a24*t^4+a25*t^5;
    xdot_d(1)=a11+2*a12*t+3*a13*t^2+4*a14*t^3+5*a15*t^4;
    xdot_d(2)=a21+2*a22*t+3*a23*t^2+4*a24*t^3+5*a25*t^4;
    xddot_d(1)=2*a12+6*a13*t+12*a14*t^2+20*a15*t^3;
    xddot_d(2)=2*a22+6*a23*t+12*a24*t^2+20*a25*t^3;

    x_d = [x_d(1);x_d(2)];
    xdot_d = [xdot_d(1);xdot_d(2)];
    xddot_d = [xddot_d(1);xddot_d(2)];
  
    
    % Design Task Space Controller
    zdot = pinv(Ja)*(xdot_d + lambda*(x_d-x));
    zddot = pinv(Ja)*(xddot_d + lambda*(xdot_d-xdot) - Jadot*zdot);
    u = H*zddot + C*zdot + G + Ja'*Kd*Ja*(zdot-qdot);
    
    
    % Save the results of end-effector position, velocity, acceleration
	x1_save(n) = x(1);      % Save the end-effector position
	x2_save(n) = x(2);
    xdot1_save(n) = xdot(1);      % Save the end-effector velocity
	xdot2_save(n) = xdot(2);
    xddot1_save(n) = xddot(1);      % Save the end-effector acceleration
	xddot2_save(n) = xddot(2);
    e1_save(n) = x_d(1)-x2;      % Save the trajectory error
    e2_save(n) = x_d(2)-y2;    

    
    % Calculate the coordinates of robot geometry for animation 
	Ax1 = [0, x1];  
	Ay1 = [0, y1];
   	Ax2 = [x1, x2];   
	Ay2 = [y1, y2];

   
    % Update the animation
	if rem(n,10) == 0
        set(p1,'X', Ax1, 'Y',Ay1);
        set(p2,'X', Ax2, 'Y',Ay2);
        drawnow
    end
    
    
    % Save 1st and 2nd joint's location, (x1, y1) and (x2, y2)
    x2_save(n_trj) = x2;
    y2_save(n_trj) = y2;
    x2d_save(n_trj) = x_d(1);
    y2d_save(n_trj) = x_d(2);
        
    
    % Increase the iteration number
	n=n+1;
    n_trj = n_trj + 1;
    
end


% Draw trajectory of (x1, y1) and (x2, y2)
plot(x2_save, y2_save, 'k.', 'MarkerSize', 2)
plot(x2d_save, y2d_save, 'k.', 'MarkerSize', 2, 'Color', 'r')

T=t0:s_time:tf;

% Plot the graph of end-effector
figure(2)
subplot(3,1,1)
plot(T, x1_save, T, x2_save, 'r')
title('< End-effector Position >')
legend('x (m)', 'y (m)')
xlabel('t (sec)')
ylabel('x (m)')

subplot(3,1,2)
plot(T, xdot1_save, T, xdot2_save, 'r')
title('< End-effector Velocity >')
legend('v_x (m/sec)', 'v_y (m/sec)')
xlabel('t (sec)')
ylabel('v (m/sec)')

subplot(3,1,3)
plot(T, xddot1_save, T, xddot2_save, 'r')
title('< End-effector Acceleration >')
legend('a_x (m/sec^2)', 'a_y (m/sec^2)')
xlabel('t (sec)')
ylabel('a (m/sec^2)')

% Plot the graph of Tracking error
figure(4)
plot(T, e1_save)
xlabel('Time (sec)')
ylabel('e_1(m)')

figure(5)
plot(T, e2_save)
xlabel('Time (sec)')
ylabel('e_2(m)')