clear;close all;clc
%%
syms t l1 l2 l3 l4 th1(t) th2(t) th3(t) th4(t) m1 m2 m3 m4 g th1dd th2dd th3dd th4dd

x1=l1*cos(th1(t));
y1=l1*sin(th1(t));

x2=x1 + l2*cos(th2(t));
y2=y1 + l2*sin(th2(t));

x3=x2 + l3*cos(th3(t));
y3=y2 + l3*sin(th3(t));

x4=x3 + l4*cos(th4(t));
y4=y3 + l4*sin(th4(t));

dx1=diff(x1,t);dx2=diff(x2,t);dx3=diff(x3,t);dx4=diff(x4,t);
dy1=diff(y1,t);dy2=diff(y2,t);dy3=diff(y3,t);dy4=diff(y4,t);

K=(0.5*m1*(dx1^2 + dy1^2))+(0.5*m2*(dx2^2 + dy2^2))+(0.5*m3*(dx3^2 + dy3^2))+(0.5*m4*(dx4^2 + dy4^2));

P = m1*g*y1+m2*g*y2+m3*g*y3+m4*g*y4;
Lg=  K - P;

dLbdth1=diff(Lg,diff(th1(t), t));dLbdth2=diff(Lg,diff(th2(t), t));dLbdth3=diff(Lg,diff(th3(t), t));dLbdth4=diff(Lg,diff(th4(t), t));
dbdtodLbdth1=diff(dLbdth1, t);dbdtodLbdth2=diff(dLbdth2, t);dbdtodLbdth3=diff(dLbdth3, t);dbdtodLbdth4=diff(dLbdth4, t);
dLbth1=diff(Lg,th1(t));dLbth2=diff(Lg,th2(t));dLbth3=diff(Lg,th3(t));dLbth4=diff(Lg,th4(t));

T1 = dbdtodLbdth1-dLbth1;
M1 = subs(T1,{diff(th1(t),t,t),diff(th2(t),t,t),diff(th3(t),t,t),diff(th4(t),t,t)},{th1dd,th2dd,th3dd,th4dd});
M1_1 = subs(M1,{diff(th1(t),t),diff(th2(t),t),diff(th3(t),t),diff(th4(t),t),g},{0,0,0,0,0});
G1 = subs(T1,{diff(th1(t),t),diff(th2(t),t),diff(th3(t),t),diff(th4(t),t)},{0,0,0,0});
H1 = subs(T1,{diff(th1(t),t,t),diff(th2(t),t,t),diff(th3(t),t,t),diff(th4(t),t,t),g},{0,0,0,0,0});

T2 = dbdtodLbdth2-dLbth2;
M2 = subs(T2,{diff(th1(t),t,t),diff(th2(t),t,t),diff(th3(t),t,t),diff(th4(t),t,t)},{th1dd,th2dd,th3dd,th4dd});
M2_2 = subs(M2,{diff(th1(t),t),diff(th2(t),t),diff(th3(t),t),diff(th4(t),t),g},{0,0,0,0,0});
G2 = subs(T2,{diff(th1(t),t),diff(th2(t),t),diff(th3(t),t),diff(th4(t),t)},{0,0,0,0});
H2 = subs(T2,{diff(th1(t),t,t),diff(th2(t),t,t),diff(th3(t),t,t),diff(th4(t),t,t),g},{0,0,0,0,0});

T3 = dbdtodLbdth3-dLbth3;
M3 = subs(T3,{diff(th1(t),t,t),diff(th2(t),t,t),diff(th3(t),t,t),diff(th4(t),t,t)},{th1dd,th2dd,th3dd,th4dd});
M3_3 = subs(M3,{diff(th1(t),t),diff(th2(t),t),diff(th3(t),t),diff(th4(t),t),g},{0,0,0,0,0});
G3 = subs(T3,{diff(th1(t),t),diff(th2(t),t),diff(th3(t),t),diff(th4(t),t)},{0,0,0,0});
H3 = subs(T3,{diff(th1(t),t,t),diff(th2(t),t,t),diff(th3(t),t,t),diff(th4(t),t,t),g},{0,0,0,0,0});

T4 = dbdtodLbdth4-dLbth4;
M4 = subs(T4,{diff(th1(t),t,t),diff(th2(t),t,t),diff(th3(t),t,t),diff(th4(t),t,t)},{th1dd,th2dd,th3dd,th4dd});
M4_4 = subs(M4,{diff(th1(t),t),diff(th2(t),t),diff(th3(t),t),diff(th4(t),t),g},{0,0,0,0,0});
G4 = subs(T4,{diff(th1(t),t),diff(th2(t),t),diff(th3(t),t),diff(th4(t),t)},{0,0,0,0});
H4 = subs(T4,{diff(th1(t),t,t),diff(th2(t),t,t),diff(th3(t),t,t),diff(th4(t),t,t),g},{0,0,0,0,0});

[c1,t1] = coeffs(simplify(M1_1),[th1dd,th2dd,th3dd,th4dd]);
[c2,t2] = coeffs(simplify(M2_2),[th1dd,th2dd,th3dd,th4dd]);
[c3,t3] = coeffs(simplify(M3_3),[th1dd,th2dd,th3dd,th4dd]);
[c4,t4] = coeffs(simplify(M4_4),[th1dd,th2dd,th3dd,th4dd]);


