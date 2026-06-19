clear all;
close all;
clc;
%% ODE solver
 T=0:1/100000:6;
[t,x]=ode45 ('ode_solver_3R',T,[0,0,0,0,0,0]); 
  

m1=1; m2=1; m3=1;   l1=1; l2=1; l3=1; g=9.81;
                                                                                                                                                        g=0;
th1 = x(:,1); th1_d = x(:,2); th2 = x(:,3); th2_d = x(:,4); th3 = x(:,5); th3_d = x(:,6) ; %Joint position and velocities

%% Energy Calculation
                                                                                                                                                                      
for i=1:1:length(th1)
% Equation for kinetic energy
KE(i,1) = (m3*((l1*th1_d(i)*cos(th1(i)) + l2*th2_d(i)*cos(th2(i)) + (l3*th3_d(i)*cos(th3(i)))/2)^2 + (l1*th1_d(i)*sin(th1(i)) + l2*th2_d(i)*sin(th2(i)) + (l3*th3_d(i)*sin(th3(i)))/2)^2))/2 + (m2*((l1*th1_d(i)*cos(th1(i)) + (l2*th2_d(i)*cos(th2(i)))/2)^2 + (l1*th1_d(i)*sin(th1(i)) + (l2*th2_d(i)*sin(th2(i)))/2)^2))/2 + (l1^2*m1*th1_d(i)^2)/6 + (l2^2*m2*th2_d(i)^2)/24 + (l3^2*m3*th3_d(i)^2)/24;
 
% Equation for potential energy
PE(i,1) = g*m3*(l1*sin(th1(i)) + l2*sin(th2(i)) + 0.5*l3*sin(th3(i)))  + g*m2*(l1*sin(th1(i)) + 0.5*l2*sin(th2(i))) + 0.5*g*l1*m1*sin(th1(i));
end

%Total energy
TE=KE+PE;

% figure
% plot(TE)

%% Display The Results

% %Ploting energies
% figure('units','normalized','outerposition',[0 0 1 1]);
% subplot(211);
% plot(t,KE,'r','LineWidth',1.5);
% title("Kinetic Energy",'Interpreter','latex');
% xlabel('Time (s)','Interpreter','latex');
% ylabel('Energy (J) ','Interpreter','latex');
% % ylim([-50 1600]);
% set(gca,'FontSize',18);
% grid minor;
% 
% 
% subplot(212);
% plot(t,PE,'b','LineWidth',1.5);
% title("Potential Energy",'Interpreter','latex');
% xlabel('Time (s)','Interpreter','latex');
% ylabel('Energy  (J) ','Interpreter','latex');
% % ylim([-1600 50]);
% set(gca,'FontSize',18);
% grid minor;
% set(gca);
% % saveas(gcf,'Q1_a_KE_PE2.png')


figure('units','normalized','outerposition',[0 0 1 1]);
subplot(211);
plot(t,TE,'LineWidth',1.5);
title("Total Energy",'Interpreter','latex');
xlabel('Time (s)','Interpreter','latex');
ylabel('Energy  (J)','Interpreter','latex');
ylim([-5 5]);
set(gca,'FontSize',18);
grid minor;
% saveas(gcf,'Q1_a_TE2.png')