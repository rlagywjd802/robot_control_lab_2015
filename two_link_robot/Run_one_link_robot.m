clear
clear all
close all

global m;
global lm;
global g;
global u;
global N; % 기어비
global Imot; %모터 로터의 moment of inertia

% Control frequency is defined as follows (sec) %
s_time = 0.002;

% Terminal time (sec)
ft = 4;

% Radian to Degree
R2D = 180/pi;

% Initial Configuration should be defined in RADIAN %
q_old = 0;
qdot_old = 0;

% Robot Parameters 
N = 26;

m = 0.5;
lm = 0.15;
g = 9.806;
Imot = 0;% No friction, 원래는 181*10^(-7);임


% Iteration number %
n=1;

% Plot Setting %
hold on
axis([-0.3 0.3 -0.3 0.3]);
grid
    x = lm*cos(q_old);
    Ax = [0, x];
    y = lm*sin(q_old);
    Ay = [0, y];
	p_link = line(Ax,Ay,'EraseMode','xor','LineWidth',[5],'Color','b');
	p_pt1 = plot(0, 0, 'ro', 'LineWidth', 10, 'MarkerSize', 10);          %1st joint, small circle 그리기
	p_pt2 = plot(Ax(2), Ay(2), 'ro', 'LineWidth', 10, 'MarkerSize', 10);  %2nd joint, small circle 그리기
   
    
% Robot Implementation 
for i = 0 : s_time : ft

    % Control part to be designed %
    u = 0;
   
	% Forward Dynamics Part for Motion Generation %
    [t,y] = ode45('one_link', [0, s_time], [q_old; qdot_old]);
   
    indx = size(y);
    q_old = y(indx(1), 1);
    qdot_old = y(indx(1), 2);
     
	%Save the results
	q_save(n)=q_old;
	q_dot_save(n)=qdot_old;

	% Forward Kinematics Part %
    x = lm*cos(q_old);
    Ax = [0, x];
    y = lm*sin(q_old);
    Ay = [0, y];

	% Increase the Iteration Number
    n=n+1;

	% Draw now %
    if rem(n,10) == 0
        set(p_link,'X', Ax, 'Y',Ay)
        set(p_pt1);
        set(p_pt2,'X', Ax(2), 'Y', Ay(2));
   	drawnow
 	end  
end


t= 0 : s_time : ft;

figure(2)
title('Joint position Graph')
plot(t,q_save.*R2D)
xlabel('Time(sec)')
ylabel('Joint angle(degree)')
legend('q(degree)')

figure(3)
title('Joint velocity Graph')
plot(t,q_dot_save.*R2D)
xlabel('Time(sec)')
ylabel('Joint angular velocity(degree/sec)')
legend('q dot(degree/sec)')


%Calculate the kinetic evergy, and the potential energy
T = (1/2)*m*lm^2*q_dot_save.^2;          %Kinetic energy
V = m*g*lm*sin(q_save);                 %Potential energy

TotalE= T + V;                          %Total energy

figure(4)
title('Energy Graph')
xlabel('Time(sec)')
ylabel('Energy (J)')
plot(t, T, t, V, t, TotalE)
legend('Kinetic Energy','Potential Energy','Total Energy')