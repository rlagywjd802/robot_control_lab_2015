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
tf = 10;

% Initial Configurations should be defined in RADIAN %
q(1) = 0;
q(2) = 0;
qdot(1) = 0;
qdot(2) = 0;

% Iteration number %
n=1;

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
   
   
% Robot Implementation 
for i = 0 : s_time : tf
    
   % Control part to be designed %
   u(1) = 0; 
   u(2) = 0;
   u=[ u(1) ; u(2) ];
   fa;

   % Forward Dynamics Part for Motion Generation %
   [t,y] = ode45('two_link_env',[0, s_time] , [q(1); q(2); qdot(1); qdot(2)] );
   index = size(y);
   q(1) = y(index(1), 1);
   q(2) = y(index(1), 2);
   qdot(1) = y(index(1), 3);
   qdot(2) = y(index(1), 4);

   % Forward Kinematics Part %
   x1 = l1*cos(q(1));
   Ax1 = [0, x1];
   y1 = l1*sin(q(1));
   Ay1 = [0, y1];
   x2 = x1 + l2*cos(q(1)+q(2));
   Ax2 = [x1, x2];
   y2 = y1 + l2*sin(q(1)+q(2));
   Ay2 = [y1, y2];
   
   
   % Save environment force
   f_a(:,n)=fa;   
   

   % Iteration Number
   n=n+1;
   
   % Draw now %
   if rem(n,10) == 0
      set(p1,'X', Ax1, 'Y',Ay1)
      set(p2,'X', Ax2, 'Y',Ay2)
      set(p3,'X', Env_X, 'Y',Env_Y)
   	drawnow
   end  
end


t= 0 : s_time : tf;

figure(2)
plot(t,f_a)
xlabel('Time(sec)')
ylabel('Enviroment forces(N)')
legend('fx(N)','fy(N)')