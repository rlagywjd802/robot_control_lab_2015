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
s_time = 0.002;

% Terminal time %
t0=0;           tf=4;

% Variable for converting RADIAN -> DEGREE or DEGREE -> RADIAN
R2D = 180/pi;
D2R = pi/180;

% Initial Configurations defined in RADIAN %
q = [pi/4; pi/4];
qdot=[0;  0];
qddot=[0; 0];

% start position
x = [0.15/sqrt(2); 0.15 + 0.15/sqrt(2)];

% set position
x_d = [-0.15; 0];

% PD gain
kp = 10;    kd = 5;     % two_link
% kp = 12;    kd = 5;   % two_link_df
kp = kp*0.5; kd = kd*0.5; 
%kp = kp*1.5; kd = kd*1.5; k = k*1.5;
Kp = [kp, 0; 0, kp];
Kd = [kd, 0; 0, kd];

% Initial Input %
u = [0;0];   
q_d = [0; 0];   

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
for i = t0 : s_time : tf
    
    % Forward Dynamics Part for Motion Generation %
    %[t,y] = ode45('two_link_df',[0, s_time] , [q(1); q(2); qdot(1); qdot(2)] );
    [t,y] = ode45('two_link',[0, s_time] , [q(1); q(2); qdot(1); qdot(2)] );
    
    index = size(y);
    q(1) = y(index(1), 1);
    q(2) = y(index(1), 2);
    qdot(1) = y(index(1), 3);
    qdot(2) = y(index(1), 4);
    qddot(1) = temp2(1);
    qddot(2) = temp2(2);
    
    
    % Solve the IK using Jacobian transpose
    Ja = [-l1*sin(q(1))-l2*sin(q(1)+q(2)) -l2*sin(q(1)+q(2));
           l1*cos(q(1))+l2*cos(q(1)+q(2))  l2*cos(q(1)+q(2))]; 

    
    % Control part to be designed %
    u = Ja'*Kp*(x_d - x) - Ja'*Kd*Ja*qdot + [(m1+m2)*g*l1*cos(q(1)) + m2*g*l2*cos(q(1)+q(2)); m2*g*l2*cos(q(1)+q(2))];
        
  
    % Forward Kinematics %
	x1 = l1*cos(q(1));
	y1 = l1*sin(q(1));
	x2 = x1 + l2*cos(q(1)+q(2));
	y2 = y1 + l2*sin(q(1)+q(2));
    x = [x2;y2];
    
    
   	% Save the results of joint angles, angular velocities, angular acceleration
	q1_save(n) = q(1)*R2D;      % Save the joint angle in degree
	q2_save(n) = q(2)*R2D;
    qdot1_save(n) = qdot(1)*R2D;      % Save the angular velocity in degree
	qdot2_save(n) = qdot(2)*R2D;
    qddot1_save(n) = qddot(1)*R2D;      % Save the angular acceleration in degree
	qddot2_save(n) = qddot(2)*R2D;
    e1_save(n) = x_d(1)-x2;      % Save the trajectory error in degree
    e2_save(n) = x_d(2)-y2;    
    
    
    % Calculate the coordinates of robot geometry for animation 
	Ax1 = [0, x1];  
	Ay1 = [0, y1];
	Ax2 = [x1, x2];   
	Ay2 = [y1, y2];
    

   % Increase the Iteration Number
   n=n+1;
   
   
   % Draw Animation
   if rem(n,10) == 0
      set(p1,'X', Ax1, 'Y',Ay1)
      set(p2,'X', Ax2, 'Y',Ay2)
   	drawnow
   end  
   
end

plot(x_d(1), x_d(2),'*','LineWidth',2,'Color','r');

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
figure(3)
plot(T, e1_save)
xlabel('Time (sec)')
ylabel('e_1(m)')

figure(4)
plot(T, e2_save)
xlabel('Time (sec)')
ylabel('e_2(m)')
