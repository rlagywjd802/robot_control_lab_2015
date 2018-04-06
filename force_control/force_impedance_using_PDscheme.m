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
global fa;
global Env_X;
global Env_Y;
global Ja;
global t0;
global tf;

% Radian -> Degree 변환 계수
R2D = 180/pi;

% Robot Parameters 
m1 = 0.5;
m2 = 0.5;
l1 = 0.15;
l2 = 0.15;
g = 9.806;

% Control frequency is defined as follows %
s_time = 0.002;

% Terminal time %
t0 = 0;
tf = 10;

% Constant
%k = 1;  d = 1;
%k = 20;  d = 10;
%k = 2;  d = 1;
%k = 50;  d = 20;
%k = 40;  d = 45;
k = 32;  d = 35;

Km = [k,k;k,k];
Dm = [d,d;d,d];

% Initial Configurations should be defined in RADIAN %
q(1) = pi/4;
q(2) = pi/4;
qdot(1) = 0;
qdot(2) = 0;
x = [0.15/sqrt(2); 0.15+0.15/sqrt(2)];


% set position to be regulated %
xd = [-0.18;-0.06];


% Iteration number %
n=1;
n_trj=1;

% Plot Setting %
figure(1)
title('Animation')
hold on
axis([-0.4 0.4 -0.4 0.4]);
grid
   x1 = l1*cos(q(1));
   Ax1 = [0, x1];
   y1 = l1*sin(q(1));
   Ay1 = [0, y1];
   x2 = x1 + l2*cos(q(1)+q(2));
   Ax2 = [x1, x2];
   y2 = y1 + l2*sin(q(1)+q(2));
   Ay2 = [y1, y2];
   Env_X = [-l1, -l1];
   Env_Y = [-l1*3, l1*3];   
   p1 = line(Ax1,Ay1,'EraseMode','xor','LineWidth',[5],'Color','b');
   p2 = line(Ax2,Ay2,'EraseMode','xor','LineWidth',[5],'Color','c');
   p3 = line(Env_X,Env_Y,'EraseMode','xor','LineWidth',[3],'Color','m');
   
% Initial Control Input %
u=[0;0];
e_old = 0;  e=0;
plot(xd(1), xd(2),'*','LineWidth',2,'Color','r');
% Robot Implementation 
for i = 0 : s_time : tf
   
    % Forward Dynamics Part for Motion Generation %
    [t,y] = ode45('two_link_env',[0, s_time] , [q(1); q(2); qdot(1); qdot(2)] );
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
    
    
    
    % Forward Kinematics
	Jadot = [-l1*cos(q(1))*qdot(1)-l2*cos(q(1)+q(2))*(qdot(1)+qdot(2)), -l2*cos(q(1)+q(2))*(qdot(1)+qdot(2));
             -l1*sin(q(1))*qdot(1)-l2*sin(q(1)+q(2))*(qdot(1)+qdot(2)), -l2*sin(q(1)+q(2))*(qdot(1)+qdot(2))];  
         
    x1 = l1*cos(q(1));
	y1 = l1*sin(q(1));
	x2 = x1 + l2*cos(q(1)+q(2));
	y2 = y1 + l2*sin(q(1)+q(2));
    
    x = [x2;y2];                    % end-effector position
    xdot = Ja*qdot;                 % end-effector velocity
    xddot = Ja*qddot + Jadot*qdot;  % end-effector acceleration
   
   
   % For Animation %
   Ax1 = [0, x1];
   Ay1 = [0, y1];
   Ax2 = [x1, x2];
   Ay2 = [y1, y2];
      
   
   % Control part to be designed %
   u = Ja'*(Km*(xd-x) - Dm*xdot) + G;

   
   % Save environment force
   f_a(:,n)=fa;   
   
   err(1) = abs(xd(1)-x2);
   err(2) = abs(xd(2)-y2);
   e_old = e; 
   e = sqrt(err(1)^2 + err(2)^2);
   if(i==0) de = 0;
   else   de = e - e_old;
   end
   
      % Save the results of end-effector position, velocity, acceleration
	x1_save(n) = x(1);      % Save the end-effector position
	x2_save(n) = x(2);
    xdot1_save(n) = xdot(1);      % Save the end-effector velocity
	xdot2_save(n) = xdot(2);
    xddot1_save(n) = xddot(1);      % Save the end-effector acceleration
	xddot2_save(n) = xddot(2);
    e1_save(n) = err(1);      % Save the trajectory error
    e2_save(n) = err(2);
    e_save(n) = e;
    de_save(n) = de;
    
   % Iteration Number
    n=n+1;        %Iterator for main loop

   % Draw now %
   if rem(n,10) == 0
      set(p1,'X', Ax1, 'Y',Ay1)
      set(p2,'X', Ax2, 'Y',Ay2)
      set(p3,'X', Env_X, 'Y',Env_Y)
   	drawnow
   end  
    
    
    % Save 1st and 2nd joint's location, (x1, y1) and (x2, y2)
%	if rem(n,10) == 0
        x_save(n_trj) = x(1);
        y_save(n_trj) = x(2);
        n_trj = n_trj + 1;
%    end
  
end

plot(x(1), x(2),'*','LineWidth',2,'Color','b');


% Draw trajectory 
plot(x_save, y_save, 'k.', 'MarkerSize', 2)
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

figure(3)
plot(T, e1_save, T, e2_save, 'r')
title('< End-effector Error >')
legend('error_x(m)', 'error_y(m)')
xlabel('t (sec)')
ylabel('error (m)')

figure(4)
plot(T, e_save, 'r')
title('< End-effector Error >')
xlabel('t (sec)')
ylabel('error (m)')

figure(5)
plot(T, de_save, 'r')
title('< End-effector Error Difference >')
xlabel('t (sec)')
ylabel('error difference (m)')

figure(6)
plot(T,f_a)
xlabel('Time(sec)')
ylabel('Enviroment forces(N)')
legend('fx(N)','fy(N)')

