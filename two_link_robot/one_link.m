function dydt = one_link(t, y)

global m;
global lm;
global g;
global u;
global N; % 기어비
global Imot; %모터 로터의 moment of inertia

% No friction
Fs=0;
Fv=0;

tau_f = Fs*sign(y(2)) + Fv*y(2);

dydt = [y(2) ; (1/(m*lm^2 + N*Imot))*(-m*g*lm*cos(y(1)) + u - tau_f) ];