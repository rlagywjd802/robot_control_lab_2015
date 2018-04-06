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
l1 = 0.25;
l2 = 0.25;
g = 9.806;

% Control frequency is defined as follows %
s_time = 0.0005;

% Terminal time %
t0=0;           tf=10;

% Variable for converting RADIAN -> DEGREE or DEGREE -> RADIAN
R2D = 180/pi;
D2R = pi/180;

% Initial Configurations %
X_0 = 0.41122;
Y_0 = 0.26359;
X_f = -0.0282;
Y_f = 0.37783;
q_d = [0; 0];

% Cubic path for x
% Move on a given line y = -0.25998*x + 0.3705 in 10sec
a10 = 0.41122;
a11 = 0;
a12 = -0.0131826;
a13 = 0.00087884;
a20 = 0.26359;
a21 = 0;
a22 = 0.003427;
a23 = -0.00022848;
    
% Iteration numbers
n=1;        %Iterator for main loop
n_trj=1;    %Iterator for trajectories of joint points


% Plot Setting %
figure(1)
title('Animation')
grid
hold on
axis([-0.6 0.6 -0.6 0.6]);
   Ax1 = [0, l1];
   Ay1 = [0, 0];
   Ax2 = [l1, l1+l2];
   Ay2 = [0, 0];
   p1 = line(Ax1,Ay1,'EraseMode','xor','LineWidth',[5],'Color','b');
   p2 = line(Ax2,Ay2,'EraseMode','xor','LineWidth',[5],'Color','c');
   
% Robot Implementation 
for i = t0 : s_time : tf
   	    
    % Desired Trajectory 
    t=i+s_time;    

    x_d=a10+a11*t+a12*t^2+a13*t^3;
    y_d=a20+a21*t+a22*t^2+a23*t^3;
    xdot_d=a11+2*a12*t+3*a13*t^2;
    ydot_d=a21+2*a22*t+3*a23*t^2;
    
    % Inverse Kinematics Part %
    th2 = 2*atan2(sqrt((l1+l2)^2 - (x_d*x_d+y_d*y_d)), sqrt((x_d*x_d+y_d*y_d) - (l1-l2)^2));
    % th1 = atan2(y_d, x_d) + atan2(l2*sin(th2), l1 + l2*cos(th2));   % elbow_up
    th1 = atan2(y_d, x_d) - atan2(l2*sin(th2), l1 + l2*cos(th2));   % elbow_down
    q_d = [th1; th2];
    
    % Forward Kinematics Part %
	x1 = l1*cos(q_d(1));
	y1 = l1*sin(q_d(1));
   
	x2 = x1 + l2*cos(q_d(1)+q_d(2));
	y2 = y1 + l2*sin(q_d(1)+q_d(2));    
   	
    % Calculate the coordinates of robot geometry for animation 
	Ax1 = [0, x1];  
	Ay1 = [0, y1];
   
	Ax2 = [x1, x2];   
	Ay2 = [y1, y2];
    
    % Save the results of joint angles, angular velocities, angular acceleration
	eex_save(n) = x_d;      % Save the point of the calculated end-effector path
	eey_save(n) = y_d;
    x2_save(n) = x2;       % Save the point of the end-effector
    y2_save(n) = y2;
	q1_save(n) = q_d(1)*R2D;      % Save the joint angle in degree
	q2_save(n) = q_d(2)*R2D;
    
    % Update the animation
	if rem(n,50) == 0
        set(p1,'X', Ax1, 'Y',Ay1);
        set(p2,'X', Ax2, 'Y',Ay2);
        drawnow
        plot(x_d, y_d, '-');
        hold on
    end
        
    % Increase the iteration number
	n=n+1;
    
end

hold on

% Draw trajectory of end-effector
plot(eex_save, eey_save, 'k.', 'MarkerSize', 2)
plot(x2_save, y2_save, 'r.', 'MarkerSize', 2)

T=t0:s_time:tf;

% Plot the graph of joint angles
figure(2)
subplot(2,1,1)
plot(T, eex_save, T, eey_save, 'r')
title('< End Effector >')
legend('x(m)', 'y(m)')
xlabel('Time (sec)')
ylabel('x (m)')

subplot(2,1,2)
plot(T, q1_save, T, q2_save, 'r')
title('< Joint angle >')
legend('q_1 (deg)', 'q_2 (deg)')
xlabel('Time (sec)')
ylabel('q (deg)')
