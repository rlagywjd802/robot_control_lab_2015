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

% Robot Parameters 
m1 = 0.5;
m2 = 0.5;
l1 = 0.15;
l2 = 0.15;
g = 9.806;

% Control frequency is defined as follows %
s_time = 0.002;

% Terminal time %
tf = 3;

% Initial Configurations should be defined in RADIAN %
q(1) = 0;
q(2) = 0;
qdot(1) = 0;
qdot(2) = 0;

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
   u(1) = 0; 
   u(2) = 0;
   u=[ u(1) ; u(2) ];

   % Forward Dynamics Part for Motion Generation %
   [t,y] = ode45('two_link',[0, s_time] , [q(1); q(2); qdot(1); qdot(2)] );

   index = size(y);
   q(1) = y(index(1), 1);
   q(2) = y(index(1), 2);
   qdot(1) = y(index(1), 3);
   qdot(2) = y(index(1), 4);
 
   %Save the results
   
   q_save(1,n)=q(1);
   q_dot_save(1,n)=qdot(1);
   q_save(2,n)=q(2);
   q_dot_save(2,n)=qdot(2);
   
   % Forward Kinematics Part %
   x1 = l1*cos(q(1));
   Ax1 = [0, x1];
   y1 = l1*sin(q(1));
   Ay1 = [0, y1];
   x2 = x1 + l2*cos(q(1)+q(2));
   Ax2 = [x1, x2];
   y2 = y1 + l2*sin(q(1)+q(2));
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

t= 0 : s_time : tf;

figure(2)
title('Joint position Graph')
plot(t,q_save)
xlabel('Time(sec)')
ylabel('Joint angle(rad)')
legend('q1(rad)','q2(rad)')

figure(3)
title('Joint velocity Graph')
plot(t,q_dot_save)
xlabel('Time(sec)')
ylabel('Joint angular velocity(rad/sec)')
legend('q1 dot(rad/sec)','q2 dot(rad/sec)')

%Calculate the kinetic evergy, and the potential energy
v1_sq = l1*l1*q_dot_save(1, :).^2;
v2_sq = l1*l1*q_dot_save(1, :).^2 + l2*l2*(q_dot_save(1, :) + q_dot_save(2, :) ).^2 + 2*l1*l2*q_dot_save(1, :).*( q_dot_save(1, :) + q_dot_save(2, :) ).*cos(q_save(2, :));

h1=l1*sin(q_save(1, :));
h2=h1 + l2*sin(q_save(1, :) + q_save(2, :) );

%T = (1/2)*m1*v1_sq + (1/2)*m2*v2_sq;    %Kinetic energy
%V = m1*g*h1 + m2*g*h2;                  %Potential energy
    
%TotalE= T + V;                          %Total energy
T = (1/2)*m1*l1*l1*q_dot_save(1, :).^2 + (1/2)*m2*(l1*l1*q_dot_save(1, :).^2 + l2*l2*(q_dot_save(1, :) + q_dot_save(2, :) ).^2 + 2*l1*l2*q_dot_save(1, :).*( q_dot_save(1, :) + q_dot_save(2, :) ).*cos(q_save(2, :)));          %Kinetic energy
V = m1*g*l1*sin(q_save(1, :)) + m2*g*l2*sin(q_save(1, :) + q_save(2, :) ) + m2*g*l1*sin(q_save(1, :));                 %Potential energy

TotalE= T + V;                          %Total energy

figure(4)
title('Energy Graph')
xlabel('Time(sec)')
ylabel('Energy (J)')
plot(t, T, t, V, t, TotalE)
legend('Kinetic Energy','Potential Energy','Total Energy')