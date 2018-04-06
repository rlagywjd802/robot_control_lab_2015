
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
Kp = 5;
Kd = 1;
% final cnfiguration
q_d = [pi/2 + pi/6; pi-pi/3];  

% Control frequency is defined as follows %
s_time = 0.002;

% Terminal time %
t0=0;           tf=4;

% Variable for converting RADIAN -> DEGREE or DEGREE -> RADIAN
R2D = 180/pi;
D2R = pi/180;

% Initial Configurations should be defined in RADIAN %
q(1) = -pi/2;
q(2) = 0;
qdot(1) = 0;
qdot(2) = 0;
qddot(1) = 0;
qddot(2) = 0;
%Joint position save
q_save=zeros(2,tf/s_time+1);

%Joint velocity save
q_dot_save=zeros(2,tf/s_time+1);

% Iteration number %
n=1;

% Plot Setting %
figure(1)
title('Animation')
hold on
axis([-0.4 0.4 -0.4 0.4]);
grid
   Ax1 = [0, l1];
   Ay1 = [0, 0];
   Ax2 = [l1, l1+l2];
   Ay2 = [0, 0];
   p1 = line(Ax1,Ay1,'EraseMode','xor','LineWidth',[5],'Color','b');
   p2 = line(Ax2,Ay2,'EraseMode','xor','LineWidth',[5],'Color','c');
   
% Robot Implementation 
for i = 0 : s_time : tf

   % Control part to be designed %
    u(1) = Kp*(q_d(1)-q(1)) - Kd*qdot(1) + (m1+m2)*g*l1*cos(q(1)) + m2*g*l2*cos(q(1)+q(2));
    u(2) = Kp*(q_d(2)-q(2)) - Kd*qdot(2) + m2*g*l2*cos(q(1)+q(2));
    u = [u(1);u(2)];
    
   % Forward Dynamics Part for Motion Generation %
   
   [t,y] = ode45('two_link',[0, s_time] , [q(1); q(2); qdot(1); qdot(2)] );
   
    index = size(y);
    q(1) = y(index(1), 1);
    q(2) = y(index(1), 2);
    qdot(1) = y(index(1), 3);
    qdot(2) = y(index(1), 4);
    qddot(1) = temp2(1);
    qddot(2) = temp2(2);
   
     % Forward Kinematics %
	x1 = l1*cos(q(1));
	y1 = l1*sin(q(1));
	x2 = x1 + l2*cos(q(1)+q(2));
	y2 = y1 + l2*sin(q(1)+q(2));
    x = [x2;y2];    % end-effector position

   %Save the results
   
	q1_save(n) = q(1)*R2D;      % Save the joint angle in degree
	q2_save(n) = q(2)*R2D;
    qdot1_save(n) = qdot(1)*R2D;      % Save the angular velocity in degree
	qdot2_save(n) = qdot(2)*R2D;
    qddot1_save(n) = qddot(1)*R2D;      % Save the angular acceleration in degree
	qddot2_save(n) = qddot(2)*R2D;
    e1_save(n) = (q_d(1)-q(1))*R2D;      % Save the trajectory error in degree
    e2_save(n) = (q_d(2)-q(2))*R2D;   
   
    % Calculate the coordinates of robot geometry for animation 
	Ax1 = [0, x1];  
	Ay1 = [0, y1];
	Ax2 = [x1, x2];   
	Ay2 = [y1, y2];

   % Increase the Iteration Number
   n=n+1;
   
   % Draw now %
   if rem(n,10) == 0
      set(p1,'X', Ax1, 'Y',Ay1)
      set(p2,'X', Ax2, 'Y',Ay2)
   	drawnow
   end  
end

T=t0:s_time:tf;

% Plot the graph of joint angles
figure(2)
subplot(3,1,1)
plot(T, q1_save, T, q2_save, 'r')
title('< Joint angle >')
legend('q_1 (deg)', 'q_2 (deg)')
xlabel('Time (sec)')
ylabel('q (deg)')

subplot(3,1,2)
plot(T, qdot1_save, T, qdot2_save, 'r')
title('< Anglular velotity >')
legend('qdot_1 (deg/sec)', 'qdot_2 (deg/sec)')
xlabel('Time (sec)')
ylabel('qdot (deg/sec)')

subplot(3,1,3)
plot(T, qddot1_save, T, qddot2_save, 'r')
title('< Angular acceleration >')
legend('qddot_1 (deg/sec^2)', 'qddot_2 (deg/sec^2)')
xlabel('Time (sec)')
ylabel('qddot (deg/sec^2)')

% Plot the graph of Tracking error
figure(4)
subplot(2,1,1)
plot(T, e1_save)
xlabel('Time (sec)')
ylabel('e_1(deg)')

subplot(2,1,2)
plot(T, e2_save)
xlabel('Time (sec)')
ylabel('e_2(deg)')