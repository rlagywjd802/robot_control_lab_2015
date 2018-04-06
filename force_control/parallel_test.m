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
tf = 5;

% Constant
h = 1;  kf = 20; ki = 10; k = 40; d = 60;

%k = 500; d = 10; h = 0.1; kf = 10; ki = 10;
%k = 500; d = 10; h = 1; kf = 10; ki = 10;
%k = 20; d = 0.1; h = 1; kf = 10; ki = 20;
%k = 20; d = 0.1; h = 1; kf = 10; ki = 20;
%k = 50; d = 20; h = 1; kf = 10; ki = 20;
%k = 50; d = 10; h = 1; kf = 10; ki = 15;
%h = 1;  kf = 20; ki = 15; k = 70; d = 50;
%h = 1;  kf = 20; ki = 15; k = 60; d = 40; 

Km = [k,k;k,k];
Dm = [d,d;d,d];
Hm = [h,0;0,h];
Kf = [kf,kf;kf,kf];
Ki = [ki,ki;ki,ki];
% Initial Configurations defined in RADIAN %
x = [0.15/sqrt(2); 0.15 + 0.15/sqrt(2)];    x_d = [-0.18;-0.06];
q = [pi/4;pi/4];    
qdot = [0;0];   qddot = [0;0];
xdot = [0;0];   xddot = [0;0];


% Robot trajectories %     
x_0 = x;            x_f = x_d;
xdot_0 = xdot;      xdot_f = [0;0];
xddot_0 = xddot;    xddot_f = [0;0];
[a10,a11,a12,a13,a14,a15]=QuinticPolynomialPath(x_0(1), xdot_0(1), xddot_0(1), x_f(1), xdot_f(1), xddot_f(1));        
[a20,a21,a22,a23,a24,a25]=QuinticPolynomialPath(x_0(2), xdot_0(2), xddot_0(2), x_f(2), xdot_f(2), xddot_f(2));


% Iteration number %
n=1;
n_trj = 1;

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
u = [0;0];
fad = [5;0];
error_sum_f = 0;
sum_err =[0;0];

% Draw trajectory 
for i = 0 : s_time : tf
    % Desired Trajectory 
    t=i;    
    X_d(1)=a10+a11*t+a12*t^2+a13*t^3+a14*t^4+a15*t^5;
    X_d(2)=a20+a21*t+a22*t^2+a23*t^3+a24*t^4+a25*t^5;
    
    % Save 1st and 2nd joint's location, (x1, y1) and (x2, y2)
    Xd_save(n_trj) = X_d(1);
    Yd_save(n_trj) = X_d(2);
    n_trj = n_trj + 1;
end

plot(Xd_save, Yd_save, 'k.', 'MarkerSize', 2,'Color','r')
n_trj = 1;

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

   
    % Control part to be designed %
    error_f = fad-fa;
    error_sum_f = error_f*s_time + error_sum_f;
    u0x = xddot_d + (Hm)\(Dm*(xdot_d-xdot) + Km*(x_d-x));
    u0f = -(Hm)\(Kf*error_f+Ki*error_sum_f);
    %u0x = xddot_d + inv(H)*(Dm*(xdot_d-xdot) + Km*(x_d-x));
    %u0f = -inv(H)*(Kf*error_f+Ki*error_sum_f);
    u = Ja'*(H*(u0x+u0f) + C*xdot + G -fa);
  
   
   err(1) = x_d(1)-x2;
   err(2) = x_d(2)-y2;
   sum_err(1) = err(1) + sum_err(1);
   sum_err(2) = err(2) + sum_err(2);
   
    % Save the results of end-effector position, velocity, acceleration
	x1_save(n) = x(1);      % Save the end-effector position
	x2_save(n) = x(2);
    xdot1_save(n) = xdot(1);      % Save the end-effector velocity
	xdot2_save(n) = xdot(2);
    xddot1_save(n) = xddot(1);      % Save the end-effector acceleration
	xddot2_save(n) = xddot(2);
    e1_save(n) = err(1);      % Save the trajectory error
    e2_save(n) = err(2);   
    sum_e1_save(n) = sum_err(1);
    sum_e2_save(n) = sum_err(2);
    
   % Save environment force
   f_a(:,n)=fa;           
   
  % For Animation %
   Ax1 = [0, x1];
   Ay1 = [0, y1];
   Ax2 = [x1, x2];
   Ay2 = [y1, y2];
   

   
      % Save the results of end-effector position, velocity, acceleration
	x1_save(n) = x(1);      % Save the end-effector position
	x2_save(n) = x(2);
    xdot1_save(n) = xdot(1);      % Save the end-effector velocity
	xdot2_save(n) = xdot(2);
    xddot1_save(n) = xddot(1);      % Save the end-effector acceleration
	xddot2_save(n) = xddot(2);
   
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

        x_save(n_trj) = x(1);
        y_save(n_trj) = x(2);
        n_trj = n_trj + 1;

    % Draw trajectory 
  %  plot(x(1), x(2), 'k.', 'MarkerSize', 3)
        % Save 1st and 2nd joint's location, (x1, y1) and (x2, y2)

        xd_save(n_trj) = x_d(1);
        yd_save(n_trj) = x_d(2);

% Draw trajectory 
plot(x2, y2, 'k.', 'MarkerSize', 2)
end





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
plot(T, sum_e1_save, T, sum_e2_save, 'r')

