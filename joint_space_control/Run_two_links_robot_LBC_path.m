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
global kp;
global kd;

% Robot Parameters 
m1 = 0.5;
m2 = 0.5;
l1 = 0.15;
l2 = 0.15;
g = 9.806;

d = [ -0.008 ; 0.008];
f = [ 0.006 ; 0.006 ];

% Control frequency is defined as follows %
s_time = 0.005;

% Terminal time %
t0=0;           tf=4;

% PD gain
% Choose between cubic and quintic
kd = 0.6
lamb = 1
%kd = 1; lamb = 3;
%kd = 5; lamb = 30;
lambda = [lamb,0;0,lamb];
Kd = [kd, 0;0,kd];


% Variable for converting RADIAN -> DEGREE or DEGREE -> RADIAN
R2D = 180/pi;
D2R = pi/180;

% Initial Configurations defined in RADIAN %
q = [-pi/2; 0];
qdot=[0;  0];
qddot=[0; 0];
e = [1;1];
edot = [0;0];


% Robot trajectories %     
q_0 = q;            q_f = [pi/3, pi/6];
qdot_0 = qdot;      qdot_f = [0;0];
qddot_0 = qddot;    qddot_f = [0;0];

[a10,a11,a12,a13,a14,a15]=QuinticPolynomialPath(q_0(1), qdot_0(1), qddot_0(1), q_f(1), qdot_f(1), qddot_f(1));        
[a20,a21,a22,a23,a24,a25]=QuinticPolynomialPath(q_0(2), qdot_0(2), qddot_0(2), q_f(2), qdot_f(2), qddot_f(2));

    
% Iteration numbers
n=1;        %Iterator for main loop
n_trj=1;    %Iterator for trajectories of joint points


% Choose between cubic and quintic
k=input('Choose between none(0), disturbances(1) and disturbances+frinction(2): ');


% Plot Setting %
figure(1)
title('Animation')
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
   	
    
    % Desired Trajectory 
    t=i+s_time;    

    q_d(1)=a10+a11*t+a12*t^2+a13*t^3+a14*t^4+a15*t^5;
    q_d(2)=a20+a21*t+a22*t^2+a23*t^3+a24*t^4+a25*t^5;
    qdot_d(1)=a11+2*a12*t+3*a13*t^2+4*a14*t^3+5*a15*t^4;
    qdot_d(2)=a21+2*a22*t+3*a23*t^2+4*a24*t^3+5*a25*t^4;
    qddot_d(1)=2*a12+6*a13*t+12*a14*t^2+20*a15*t^3;
    qddot_d(2)=2*a22+6*a23*t+12*a24*t^2+20*a25*t^3;
    
    q_d = [q_d(1);q_d(2)];
    qdot_d = [qdot_d(1);qdot_d(2)];
    qddot_d = [qddot_d(1);qddot_d(2)];
    
    % Control part to be designed %
    H = [m1*l1*l1 + m2*l1*l1 + m2*l2*l2 + 2*m2*l1*l2*cos(q(2)) , m2*l2*l2 + m2*l1*l2*cos(q(2)) ;
        m2*l2*l2 + m2*l1*l2*cos(q(2)) , m2*l2*l2 ];
    C = [ -m2*l1*l2*qdot(2)*sin(q(2)) , -m2*l1*l2*(qdot(1)+qdot(2))*sin(q(2)) ;
        m2*l1*l2*qdot(1)*sin(q(2)) , 0 ];
    G = [ (m1+m2)*g*l1*cos(q(1)) + m2*g*l2*cos(q(1)+q(2)) ; m2*g*l2*cos(q(1)+q(2)) ];

    
    zdot = qdot_d - lambda*(q-q_d);
    zddot = qddot_d - lambda*(qdot-qdot_d);
    sigma = (qdot-qdot_d) + lambda*(q-q_d);
    
    u = H*zddot + C*zdot + G - Kd*sigma;
    f = [f(1)*sign(qdot(1));f(2)*sign(qdot(2))];
    
    % Forward Dynamics Part for Motion Generation %

    
    if(k==0)
        qddot = (H^(-1))*(-C*qdot- G + u);
    elseif(k==1)
        qddot = (H^(-1))*(-C*qdot- G + u + d);        
    elseif(k==2)
        qddot = (H^(-1))*(-C*qdot- G + u + d - f);    
    end
    
    % Forward Dynamics Part for Motion Generation %
    if(k==0)
        [t,y] = ode45('two_link',[0, s_time] , [q(1); q(2); qdot(1); qdot(2)] );
    elseif(k==1)
        [t,y] = ode45('two_link_d',[0, s_time] , [q(1); q(2); qdot(1); qdot(2)] );    
    elseif(k==2)
        [t,y] = ode45('two_link_df',[0, s_time] , [q(1); q(2); qdot(1); qdot(2)] );  
    end
    
    index = size(y);
    q(1) = y(index(1), 1);
    q(2) = y(index(1), 2);
    qdot(1) = y(index(1), 3);
    qdot(2) = y(index(1), 4);
 
    %[t,e] = ode45('two_link_error',[0, s_time] , [e(1);e(2);edot(1);edot(2)]);
    %index = size(e);
    %e(1) = e(index(1), 1);
    %e(2) = e(index(1), 2);
    %edot(1) = e(index(1), 3);
    %edot(2) = e(index(1), 4);
    
    % Forward Kinematics Part %
	x1 = l1*cos(q(1));
	y1 = l1*sin(q(1));
   
	x2 = x1 + l2*cos(q(1)+q(2));
	y2 = y1 + l2*sin(q(1)+q(2));
    
    x1d = l1*cos(q_d(1));
	y1d = l1*sin(q_d(1));
   
	x2d = x1 + l2*cos(q_d(1)+q_d(2));
	y2d = y1 + l2*sin(q_d(1)+q_d(2));
   
   	
    % Calculate the coordinates of robot geometry for animation 
	Ax1 = [0, x1];  
	Ay1 = [0, y1];
   
	Ax2 = [x1, x2];   
	Ay2 = [y1, y2];

    
    % Save the results of joint angles, angular velocities, angular acceleration
	q1_save(n) = q(1)*R2D;      % Save the joint angle in degree
	q2_save(n) = q(2)*R2D;
    qdot1_save(n) = qdot(1)*R2D;      % Save the angular velocity in degree
	qdot2_save(n) = qdot(2)*R2D;
    qddot1_save(n) = qddot(1)*R2D;      % Save the angular acceleration in degree
	qddot2_save(n) = qddot(2)*R2D;
    e1_save(n) = (q(1)-q_d(1))*R2D;      % Save the trajectory error in degree
    e2_save(n) = (q(2)-q_d(2))*R2D;      
    u1_save(n) = u(1);      % Save the trajectory error in degree
	u2_save(n) = u(2);
	
 % Save 1st and 2nd joint's location, (x1, y1) and (x2, y2)
	if rem(n,20) == 0
        x1_save(n_trj) = x1;
        y1_save(n_trj) = y1;
        x2_save(n_trj) = x2;
        y2_save(n_trj) = y2;
        x1d_save(n_trj) = x1d;
        y1d_save(n_trj) = y1d;
        x2d_save(n_trj) = x2d;
        y2d_save(n_trj) = y2d;
        n_trj = n_trj + 1;
    end
    
    
    % Update the animation
	if rem(n,10) == 0
        set(p1,'X', Ax1, 'Y',Ay1);
        set(p2,'X', Ax2, 'Y',Ay2);
        drawnow
    end
    
    
    % Increase the iteration number
	n=n+1;
    
end

% Draw trajectory of (x1, y1) and (x2, y2)
plot(x1_save, y1_save, 'k.', 'MarkerSize', 5)
plot(x2_save, y2_save, 'k.', 'MarkerSize', 5)
plot(x1d_save, y1d_save,  'MarkerSize', 2)
plot(x2d_save, y2d_save,  'MarkerSize', 2)

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


% Plot the graph of Applied Torque
figure(3)
subplot(2,1,1)
plot(T, u1_save)
xlabel('Time (sec)')
ylabel('\tau_1(t)')

subplot(2,1,2)
plot(T, u2_save)
xlabel('Time (sec)')
ylabel('\tau_2(t)')

% Plot the graph of Applied Torque
figure(4)
subplot(2,1,1)
plot(T, e1_save)
xlabel('Time (sec)')
ylabel('e_1(t)')

subplot(2,1,2)
plot(T, e2_save)
xlabel('Time (sec)')
ylabel('e_2(t)')
