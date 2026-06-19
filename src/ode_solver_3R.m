function Out= ode_solver_3R(t,x)

%% Input parameters
m1=1; m2=1; m3=1;  l1=1; l2=1; l3=1; g=9.81;
                                                                                                                                                                                            g=0;
%% Equation of motion 

%M matrix or Inertia Matrix
M11 = (l1^2*(m1 + 3*m2 + 3*m3))/3;
M12 = (l1*l2*cos(x(1) - x(3))*(m2 + 2*m3))/2;
M13 = (l1*l3*m3*cos(x(1) - x(5)))/2;

M21 = (l1*l2*cos(x(1) - x(3))*(m2 + 2*m3))/2;
M22 =(l2^2*(m2 + 3*m3))/3;
M23 = (l2*l3*m3*cos(x(3) - x(5)))/2;

M31 = (l1*l3*m3*cos(x(1) - x(5)))/2;
M32 = (l2*l3*m3*cos(x(3) - x(5)))/2;
M33 = (l3^2*m3)/3;

                                                                                                                                                                               
%H and G matrix or Coriolisis 
H1 = (g*l1*m1*cos(x(1)))/2 + g*l1*m2*cos(x(1)) + g*l1*m3*cos(x(1)) + (l1*l2*m2*x(4)^2*sin(x(1) - x(3)))/2 + l1*l2*m3*x(4)^2*sin(x(1) - x(3)) + (l1*l3*m3*x(6)^2*sin(x(1) - x(5)))/2;
H2 = (g*l2*m2*cos(x(3)))/2 + g*l2*m3*cos(x(3)) - (l1*l2*m2*x(2)^2*sin(x(1) - x(3)))/2 - l1*l2*m3*x(2)^2*sin(x(1) - x(3)) + (l2*l3*m3*x(6)^2*sin(x(3) - x(5)))/2;
H3 = (g*l3*m3*cos(x(5)))/2 - (l1*l3*m3*x(2)^2*sin(x(1) - x(5)))/2 - (l2*l3*m3*x(4)^2*sin(x(3) - x(5)))/2;
 
M = [M11 M12 M13 ; M21 M22 M23 ; M31 M32 M33];
HG = [H1 ; H2 ; H3];
T=[0; 0; 0]; %Torques and Forces

%% Equation in terms of acceleration

th_dd = (inv(M)) * (T  - HG )    ;% Joint  accelaration

OP=zeros(6,1);

%% Output
OP(1)=x(2); % It is theta1_dot
OP(2)=th_dd(1);% It is theta1_double_dot
OP(3)=x(4); % It is theta2_dot
OP(4)=th_dd(2); % It is theta2_double_dot
OP(5) = x(6);% It is theta3_dot
OP(6)=th_dd(3);% It is theta3_double_dot


Out=OP; % Output

end