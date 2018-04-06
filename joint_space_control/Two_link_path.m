clear 
clear all

global tf;
global t0;

% Variable for converting RADIAN -> DEGREE or DEGREE -> RADIAN
R2D = 180/pi;
D2R = pi/180;

% Control frequency is defined as follows %
s_time = 0.002;

% Terminal time %
ft = 4;

% 2 link robot, geometry
l1=1;   % length of 1st link
l2=1;   % length of 2nd link
ew=0.18;    % width of gripper
et=0.12;    % length of gripper


% Initial Configurations should be defined in RADIAN %
q = [-pi/2; 0];
qdot=[0;  0];
qddot=[0; 0];
% Iteration numbers
n=1;        %Iterator for main loop
n_trj=1;    %Iterator for trajectories of joint points


% Set the default font size
set(0,'DefaultAxesFontSize',12)

% Plot Setting %
figure(1)
xlabel('X (m)')
ylabel('Y (m)')

axis('square')
hold on
axis([-0.7 2.2 -0.7 2.2]);
grid off


% Calculate the coordinates of robot geometry for animation  ------------->
	% Forward Kinematics Part %
	x1 = l1*cos(q(1));
	y1 = l1*sin(q(1));
   
	x2 = x1 + l2*cos(q(1)+q(2));
	y2 = y1 + l2*sin(q(1)+q(2));

	% Robot geometry for animation
	Ax1 = [0, x1];  
	Ay1 = [0, y1];
   
	Ax2 = [x1, x2];   
	Ay2 = [y1, y2];
   
    % End-effector (gripper) geometry for animation
	EEx1 = [l1*cos(q(1))+l2*cos(q(1)+q(2))+(1/2)*ew*sin(q(1)+q(2)), l1*cos(q(1))+l2*cos(q(1)+q(2))-(1/2)*ew*sin(q(1)+q(2))];
	EEy1 = [l1*sin(q(1))+l2*sin(q(1)+q(2))-(1/2)*ew*cos(q(1)+q(2)), l1*sin(q(1))+l2*sin(q(1)+q(2))+(1/2)*ew*cos(q(1)+q(2))];
    
    EEx2 = [l1*cos(q(1))+l2*cos(q(1)+q(2))-(1/2)*ew*sin(q(1)+q(2)),	l1*cos(q(1))+l2*cos(q(1)+q(2))-(1/2)*ew*sin(q(1)+q(2))+et*cos(q(1)+q(2))];
    EEy2 = [l1*sin(q(1))+l2*sin(q(1)+q(2))+(1/2)*ew*cos(q(1)+q(2)),	l1*sin(q(1))+l2*sin(q(1)+q(2))+(1/2)*ew*cos(q(1)+q(2))+et*sin(q(1)+q(2))];  
    
    EEx3 = [l1*cos(q(1))+l2*cos(q(1)+q(2))+(1/2)*ew*sin(q(1)+q(2)), l1*cos(q(1))+l2*cos(q(1)+q(2))+(1/2)*ew*sin(q(1)+q(2))+et*cos(q(1)+q(2))];
    EEy3 = [l1*sin(q(1))+l2*sin(q(1)+q(2))-(1/2)*ew*cos(q(1)+q(2)), l1*sin(q(1))+l2*sin(q(1)+q(2))-(1/2)*ew*cos(q(1)+q(2))+et*sin(q(1)+q(2))];
   
	p1 = line(Ax1,Ay1,'EraseMode','xor','LineWidth',[5],'Color','b');    %1st link, line 그리기
	p2 = line(Ax2,Ay2,'EraseMode','xor','LineWidth',[5],'Color','b');    %2nd link, line 그리기
	p3 = plot(0, 0, 'ro', 'LineWidth', 10, 'MarkerSize', 10);            %1st joint, small circle 그리기
	p4 = plot(Ax1(2), Ay1(2), 'ro', 'LineWidth', 10, 'MarkerSize', 10);  %2nd joint, small circle 그리기
	p5 = line(EEx1,EEy1,'EraseMode','xor','LineWidth',[3],'Color','r');    % End-effector (gripper), 첫 번째 line 그리기
	p6 = line(EEx2,EEy2,'EraseMode','xor','LineWidth',[3],'Color','r');    % End-effector (gripper), 두 번째 line 그리기
	p7 = line(EEx3,EEy3,'EraseMode','xor','LineWidth',[3],'Color','r');    % End-effector (gripper), 세 번째 line 그리기
%<--------------------------------------------------------------------
    
    
    
    
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Your code here, calculates the robot trajectories ---------------> 
    % Initial Condition
    t0=0;           tf=ft;
    
    q_0(1)=q(1);            q_f(1)=pi/3;
    qdot_0(1)=qdot(1);      qdot_f(1)=0;
    qddot_0(1)=qddot(1);    qddot_f(1)=0;
    
    q_0(2)=q(2);            q_f(2)=pi/6;
    qdot_0(2)=qdot(2);      qdot_f(2)=0;
    qddot_0(2)=qddot(2);    qddot_f(2)=0;
   

    [a10,a11,a12,a13,a14,a15]=QuinticPolynomialPath(q_0(1), qdot_0(1), qddot_0(1), q_f(1), qdot_f(1), qddot_f(1));        
    [a20,a21,a22,a23,a24,a25]=QuinticPolynomialPath(q_0(2), qdot_0(2), qddot_0(2), q_f(2), qdot_f(2), qddot_f(2));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
% Robot Implementation 
for i = 0 : s_time : ft
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% Your code here, input the joint angle values ------------->
    t=i+s_time;
    

    q(1)=a10+a11*t+a12*t^2+a13*t^3+a14*t^4+a15*t^5;
    q(2)=a20+a21*t+a22*t^2+a23*t^3+a24*t^4+a25*t^5;
    qdot(1)=a11+2*a12*t+3*a13*t^2+4*a14*t^3+5*a15*t^4;
    qdot(2)=a21+2*a22*t+3*a23*t^2+4*a24*t^3+5*a25*t^4;
    qddot(1)=2*a12+6*a13*t+12*a14*t^2+20*a15*t^3;
    qddot(2)=2*a22+6*a23*t+12*a24*t^2+20*a25*t^3;

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
	% Forward Kinematics Part %
	x1 = l1*cos(q(1));
	y1 = l1*sin(q(1));
   
	x2 = x1 + l2*cos(q(1)+q(2));
	y2 = y1 + l2*sin(q(1)+q(2));
   
   
	% Calculate the coordinates of robot geometry for animation ---------->
	Ax1 = [0, x1];  
	Ay1 = [0, y1];
   
	Ax2 = [x1, x2];   
	Ay2 = [y1, y2];
   
	EEx1 = [l1*cos(q(1))+l2*cos(q(1)+q(2))+(1/2)*ew*sin(q(1)+q(2)), l1*cos(q(1))+l2*cos(q(1)+q(2))-(1/2)*ew*sin(q(1)+q(2))];
	EEy1 = [l1*sin(q(1))+l2*sin(q(1)+q(2))-(1/2)*ew*cos(q(1)+q(2)), l1*sin(q(1))+l2*sin(q(1)+q(2))+(1/2)*ew*cos(q(1)+q(2))];
    
    EEx2 = [l1*cos(q(1))+l2*cos(q(1)+q(2))-(1/2)*ew*sin(q(1)+q(2)),	l1*cos(q(1))+l2*cos(q(1)+q(2))-(1/2)*ew*sin(q(1)+q(2))+et*cos(q(1)+q(2))];
    EEy2 = [l1*sin(q(1))+l2*sin(q(1)+q(2))+(1/2)*ew*cos(q(1)+q(2)),	l1*sin(q(1))+l2*sin(q(1)+q(2))+(1/2)*ew*cos(q(1)+q(2))+et*sin(q(1)+q(2))];  
    
    EEx3 = [l1*cos(q(1))+l2*cos(q(1)+q(2))+(1/2)*ew*sin(q(1)+q(2)), l1*cos(q(1))+l2*cos(q(1)+q(2))+(1/2)*ew*sin(q(1)+q(2))+et*cos(q(1)+q(2))];
    EEy3 = [l1*sin(q(1))+l2*sin(q(1)+q(2))-(1/2)*ew*cos(q(1)+q(2)), l1*sin(q(1))+l2*sin(q(1)+q(2))-(1/2)*ew*cos(q(1)+q(2))+et*sin(q(1)+q(2))];
    % <--------------------------------------------------------------------
    
    
	% Save the results of joint angles, angular velocities, angular acceleration
	q1_save(n) = q(1)*R2D;      % Save the joint angle in degree
	q2_save(n) = q(2)*R2D;
    qdot1_save(n) = qdot(1)*R2D;      % Save the angular velocity in degree
	qdot2_save(n) = qdot(2)*R2D;
    qddot1_save(n) = qddot(1)*R2D;      % Save the angular acceleration in degree
	qddot2_save(n) = qddot(2)*R2D;
    
	% Save 1st and 2nd joint's location, (x1, y1) and (x2, y2)
	if rem(n,10) == 0
        x1_save(n_trj) = x1;
        y1_save(n_trj) = y1;
        x2_save(n_trj) = x2;
        y2_save(n_trj) = y2;
        n_trj = n_trj + 1;
    end

	% Increase the iteration number
	n=n+1;
   
	% Update the animation 
	if rem(n,20) == 0
        set(p1,'X', Ax1, 'Y',Ay1);
        set(p2,'X', Ax2, 'Y',Ay2);
        set(p3);
        set(p4,'X', Ax1(2), 'Y',Ay1(2));
        set(p5,'X', EEx1, 'Y',EEy1);
        set(p6,'X', EEx2, 'Y',EEy2);
        set(p7,'X', EEx3, 'Y',EEy3);
        drawnow
    end  
end

hold on

% Draw trajectory of (x1, y1) and (x2, y2)
plot(x1_save, y1_save, 'k.', 'MarkerSize', 2)
plot(x2_save, y2_save, 'k.', 'MarkerSize', 2)


% Plot the graph of joint angles
T=0:s_time:ft;

figure(2)
subplot(3,1,1)
plot(T, q1_save, T, q2_save, 'r--')
legend('q_1 (deg)', 'q_2 (deg)')
xlabel('Time (sec)')
ylabel('Joint angle (deg)')

subplot(3,1,2)
plot(T, qdot1_save, T, qdot2_save, 'r--')
legend('qdot_1 (deg/sec)', 'qdot_2 (deg/sec)')
xlabel('Time (sec)')
ylabel('Anglular velotity (deg/sec)')

subplot(3,1,3)
plot(T, qddot1_save, T, qddot2_save, 'r--')
legend('qddot_1 (deg/sec^2)', 'qddot_2 (deg/sec^2)')
xlabel('Time (sec)')
ylabel('Angular acceleration (deg/sec^2)')