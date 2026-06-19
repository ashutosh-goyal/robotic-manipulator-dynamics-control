clear;close all;clc
%%
syms t l1 l2 l3 l4 th1(t) th2(t) th3(t) th4(t) m1 m2 m3 m4 g th1dd th2dd th3dd th4dd th1d th2d th3d th4d
I1 = (m1*l1*l1)/3; % Inertia of link 1 wrt base frame
I2 = (m2*l2*l2)/12; % Inertia of link 2 wrt its COM
I3 = (m3*l3*l3)/12; % % Inertia of link 2 wrt its COM


x1=l1*cos(th1(t)); % Co-ordinates of link-1 end effector
y1=l1*sin(th1(t)); % Co-ordinates of link-1 end effector
x2=x1 + (l2*cos(th2(t))/2); % Co-ordinates of link-2 COM
y2=y1 + (l2*sin(th2(t))/2); % Co-ordinates of link-2 COM
x3=x2 + (l2*cos(th2(t))/2)+(l3*cos(th3(t))/2); % Co-ordinates of link-3 COM
y3=y2 +(l2*sin(th2(t))/2)+ (l3*sin(th3(t))/2); % Co-ordinates of link-3 COM


% Now, we will calculate the translational kinetic energy for link 2 and 3
% dx1=diff(x1,t);
dx2=diff(x2,t);dx3=diff(x3,t);
% dy1=diff(y1,t);
dy2=diff(y2,t);dy3=diff(y3,t);


% Total kinetic energy of system
K=(0.5*m2*(dx2^2 + dy2^2))+(0.5*m3*(dx3^2 + dy3^2)) + 0.5*I1*diff(th1(t),t)^2 + 0.5*I2*diff(th2(t),t)^2 + 0.5*I3*diff(th3(t),t)^2 ;
% Total potential energy
P = m1*g*y1/2+m2*g*y2+m3*g*y3;
% Langrangian formulation
Lg=  K - P;

dLbdth1=diff(Lg,diff(th1(t), t));
dLbdth2=diff(Lg,diff(th2(t), t));
dLbdth3=diff(Lg,diff(th3(t), t));


dbdtodLbdth1=diff(dLbdth1, t);
dbdtodLbdth2=diff(dLbdth2, t);
dbdtodLbdth3=diff(dLbdth3, t);


dLbth1=diff(Lg,th1(t));
dLbth2=diff(Lg,th2(t));
dLbth3=diff(Lg,th3(t));

% Defining torque equation
T1 = dbdtodLbdth1-dLbth1;
% Extracting M (Mass/Inertia) matrix.
M1 = subs(T1,{diff(th1(t),t,t),diff(th2(t),t,t),diff(th3(t),t,t)},{th1dd,th2dd,th3dd});
M1_1 = subs(M1,{diff(th1(t),t),diff(th2(t),t),diff(th3(t),t),g},{0,0,0,0});
% Extracting gravitional terms
G1 = subs(T1,{diff(th1(t),t),diff(th2(t),t),diff(th3(t),t)},{0,0,0});
% Extracting remaining theta_dot and other coefficient (H matrix)
H1 = subs(T1,{diff(th1(t),t,t),diff(th2(t),t,t),diff(th3(t),t,t),g,diff(th1(t),t),diff(th2(t),t),diff(th3(t),t)},{0,0,0,0,th1d,th2d,th3d});



% Same Procedure for link 2 and 3
T2 = dbdtodLbdth2-dLbth2;
M2 = subs(T2,{diff(th1(t),t,t),diff(th2(t),t,t),diff(th3(t),t,t)},{th1dd,th2dd,th3dd});
M2_2 = subs(M2,{diff(th1(t),t),diff(th2(t),t),diff(th3(t),t),g},{0,0,0,0});
G2 = subs(T2,{diff(th1(t),t),diff(th2(t),t),diff(th3(t),t)},{0,0,0});
H2 = subs(T2,{diff(th1(t),t,t),diff(th2(t),t,t),diff(th3(t),t,t),g,diff(th1(t),t),diff(th2(t),t),diff(th3(t),t)},{0,0,0,0,th1d,th2d,th3d});


T3 = dbdtodLbdth3-dLbth3;
M3 = subs(T3,{diff(th1(t),t,t),diff(th2(t),t,t),diff(th3(t),t,t)},{th1dd,th2dd,th3dd});
M3_3 = subs(M3,{diff(th1(t),t),diff(th2(t),t),diff(th3(t),t),g},{0,0,0,0});
G3 = subs(T3,{diff(th1(t),t),diff(th2(t),t),diff(th3(t),t)},{0,0,0});
H3 = subs(T3,{diff(th1(t),t,t),diff(th2(t),t,t),diff(th3(t),t,t),g,diff(th1(t),t),diff(th2(t),t),diff(th3(t),t)},{0,0,0,0,th1d,th2d,th3d});


[c1,t1] = coeffs(simplify(M1_1),[th1dd,th2dd,th3dd]);
[c2,t2] = coeffs(simplify(M2_2),[th1dd,th2dd,th3dd]);
[c3,t3] = coeffs(simplify(M3_3),[th1dd,th2dd,th3dd]);


K1= subs(K,{diff(th1(t),t),diff(th2(t),t),diff(th3(t),t)},{th1d,th2d,th3d});

