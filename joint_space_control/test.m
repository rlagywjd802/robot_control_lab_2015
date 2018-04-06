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
s_time = 0.05;

% Terminal time %
t0=0;           tf=4;

% PD gain
kd=input('Kd(positive)[2x1]: ');
lambda = input('\lambda(positive definite)[2x2]: ');
Kd = [kd(1),0;0,kd(2)];
pos_def = 0;
if(lambda(1,1)>0) 
    if(det(lambda))
        pos_def = 1;
        fprintf('\lambda is Positive Definite');
    end
else
end


  if(pos_def==1)  
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
grid
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
    
       sigma1_save(n) = sigma(1);
    sigma2_save(n) = sigma(2);
    sigma1_sq_save(n) = sigma(1)*sigma(1);
    sigma2_sq_save(n) = sigma(2)*sigma(2);
    if(n==1)
        sigma1_sq_sum_save(n) =  sigma1_sq_save(n);
        sigma2_sq_sum_save(n) =  sigma2_sq_save(n);
    else
        sigma1_sq_sum_save(n) = sigma1_sq_sum_save(n-1) + sigma1_sq_save(n);
        sigma2_sq_sum_save(n) = sigma2_sq_sum_save(n-1) + sigma2_sq_save(n);
    end
    
    qt1_save(n) = q(1)*R2D-q_d(1)*R2D;
    qt2_save(n) = q(2)*R2D-q_d(2)*R2D;
    qt1_sq_save(n) = qt1_save(n)*qt1_save(n);
    qt2_sq_save(n) = qt2_save(n)*qt2_save(n);
    if(n==1)
        qt1_sq_sum_save(n) =  qt1_sq_save(n);
        qt2_sq_sum_save(n) =  qt2_sq_save(n);
    else
        qt1_sq_sum_save(n) = qt1_sq_sum_save(n-1) + qt1_sq_save(n);
        qt2_sq_sum_save(n) = qt2_sq_sum_save(n-1) + qt2_sq_save(n);
    end
    qtdot1_save(n) = qdot(1)*R2D-qdot_d(1)*R2D;
    qtdot2_save(n) = qdot(2)*R2D-qdot_d(2)*R2D;
    
    if(
    
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
 
    
    % Forward Kinematics Part %
	x1 = l1*cos(q(1));
	y1 = l1*sin(q(1));
   
	x2 = x1 + l2*cos(q(1)+q(2));
	y2 = y1 + l2*sin(q(1)+q(2));
   
   	
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
    
 
    
	
    % Update the animation
	if rem(n,10) == 0
        set(p1,'X', Ax1, 'Y',Ay1);
        set(p2,'X', Ax2, 'Y',Ay2);
        drawnow
    end
    
    
    % Increase the iteration number
	n=n+1;
    
end

T=t0:s_time:tf;

% Plot the graph of sigma
figure(2)
subplot(3,1,1)  % L_infinity
plot(T, abs(sigma1_save))
xlabel('Time (sec)')
ylabel('\sigma_1(t)')

subplot(3,1,2)  % L_2
plot(T, sqrt(sigma1_sq_sum_save))
xlabel('Time (sec)')
ylabel('\int_{0}^{t} ||\sigma_1(t)||^2 d\tau')

subplot(3,1,3) % (Lemma 2.3.2)exponentially decaying func
plot(T, sigma1_sq_save)
xlabel('Time (sec)')
ylabel('||\sigma_1(t)||^2')


figure(3)
subplot(3,1,1)  % L_infinity
plot(T, abs(sigma2_save))
xlabel('Time (sec)')
ylabel('\sigma_2(t)')

subplot(3,1,2)  % L_2
plot(T, sqrt(sigma2_sq_sum_save))
xlabel('Time (sec)')
ylabel('\int_{0}^{t} ||\sigma_2(t)||^2 d\tau')

subplot(3,1,3) % (Lemma 2.3.2)exponentially decaying func
plot(T, sigma2_sq_save)
xlabel('Time (sec)')
ylabel('||\sigma_2(t)||^2')


% Plot the graph of q
figure(4)
subplot(3,1,1)  % L_infinity, (Lemma 2.3.2)exponentially decaying func
plot(T, abs(qt1_save))
xlabel('Time (sec)')
ylabel('$\bar{q_1}$(t)', 'interpreter', 'latex')

subplot(3,1,2)  % L_2
plot(T, sqrt(qt1_sq_sum_save))
xlabel('Time (sec)')
ylabel('$\int_{0}^{t} ||\bar{q_1}(t)||^2 d\tau$', 'interpreter', 'latex')

subplot(3,1,3) 
plot(T, abs(qtdot1_save))
xlabel('Time (sec)')
ylabel('$||\dot{\bar{q_1}}(t)||$', 'interpreter', 'latex')

figure(5)
subplot(3,1,1)  % L_infinity, (Lemma 2.3.2)exponentially decaying func
plot(T, abs(qt1_save))
xlabel('Time (sec)')
ylabel('$\bar{q_2}$(t)', 'interpreter', 'latex')

subplot(3,1,2)  % L_2
plot(T, sqrt(qt1_sq_sum_save))
xlabel('Time (sec)')
ylabel('$\int_{0}^{t} ||\bar{q_2}(t)||^2 d\tau$', 'interpreter', 'latex')

subplot(3,1,3) 
plot(T, abs(qtdot1_save))
xlabel('Time (sec)')
ylabel('$||\dot{\bar{q_2}}(t)||$', 'interpreter', 'latex')

  end