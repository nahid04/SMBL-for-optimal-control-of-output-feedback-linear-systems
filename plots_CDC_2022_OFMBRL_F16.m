%% Paper title "Safe Controller for Output Feedback Linear Systems using Model-Based Reinforcement Learning"
%% arXiv:2204.01409
%% Coded by S M Nahid Mahmud, MS Grad Student, Oklahoma State University.
%% nahid.mahmud@okstate.edu
%% This code presents a F16 aircraft's three-state longitudinal dynamical system for the state estimation
%% The notations are similar to the mentioned paper. 
%% Transformed system denotes the system we have after applying the barrier transformation to the original system.
%% In this document, x is the original state of the system, xH is the estimated state of the system, s is the original state of the transformed system,...
%% sH is the estimated state of the transformed system.
% Plots need to replicate the Arxiv paper! 



clear all
clc

ADP = load('new_test_F-16.mat');

% Barrier Boundaries;
z_low_1 = -0.1; z_up_1 = 0.1; %Barrier Boundaries for x2, x2-coordinate values will be remained within (z_low_1,z_up_1) 
z_low_2 = -0.1; z_up_2 = 0.1; %Barrier Boundaries for x3, x3-coordinate values will be remained within (z_low_2,z_up_2) 
z_low_3 = -0.1; z_up_3 = 0.1; %Barrier Boundaries for x4, x4-coordinate values will be remained within (z_low_3,z_up_3) 


%Phase Portrait 
figure(1)
Point = [0,0,0] ;   % you center point 
Cube_L = [.2,.2,.2] ;  % your cube dimensions 
Origin = Point-Cube_L/2 ;       % Get the origin of cube so that P is at center 
plot3(Point(1),Point(2),Point(3),'o')
hold on
plot3(ADP.z(1,1),ADP.z(1,2),ADP.z(1,3),'*')
hold on
plotcube(Cube_L,Origin,0,[1 0 0]); 
hold on
p=plot3(ADP.z(:,1),ADP.z(:,2),ADP.z(:,3),'Linewidth',1);
xlabel('$x_{1}(t)$','interpreter','latex')
ylabel({'$x_{2}(t)$'},'interpreter','latex')
zlabel('$x_{3}(t)$','interpreter','latex')
set(gca,'FontSize',25)
hold on
l=legend('\textbf{Origin}','\textbf{Initial Position}','interpreter','latex','FontSize',15)
set(l,'Interpreter','latex','Location','northeast','NumColumns',3);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8 4]);
set(gcf, 'PaperPosition', [0 0 8 4]);
grid on
print -dpdf Time_Vs_States_MBRL_SE_Optimal__F-16.png
print -dpdf Time_Vs_States_MBRL_SE_Optimal__F-16.pdf


% State error trajectories of the x-coordinates
figure(2)
p=plot(ADP.t,(ADP.z(:,1:ADP.P.n)-ADP.z(:,ADP.P.n+1:2*ADP.P.n,1)),'Linewidth',1);
mrk1={'s','v','o','*','^','x','+','.','d','h'};
mrk=(mrk1(1,1:size(p,1)))';
set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
f_nummarkers(p,20,1);
for i=1:size(p,1)
    hasbehavior(p(i), 'legend', false);
end
xlabel('$t$','interpreter','latex')
ylabel({'$\tilde{x}(t)$'},'interpreter','latex')
% ylim([-0.2 0.2])
% xlim([0 0.2])
set(gca,'FontSize',25)
l=legend('$\tilde{x}_{1}$','$\tilde{x}_{2}$','$\tilde{x}_{3}$','interpreter','latex')
set(l,'Interpreter','latex','Location','northeast','NumColumns',3);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8 4]);
set(gcf, 'PaperPosition', [0 0 8 4]);
grid on
print -dpdf Time_Vs_States_Errors_MBRL_SE_Optimal__F-16.png
print -dpdf Time_Vs_States_Errors_MBRL_SE_Optimal__F-16.pdf


% Trajectory of the Critic weights 
figure(3)
p=plot(ADP.t,((ADP.z(:,2*ADP.P.n+ADP.P.d+ADP.P.W+1:2*ADP.P.n+ADP.P.d+2*ADP.P.W))),'Linewidth',1);
mrk1={'s','v','o','*','^','x','+','.','d','h'};
mrk=(mrk1(1,1:size(p,1)))';
set(p,{'marker'},mrk,{'markerfacecolor'},get(p,'Color'),'markersize',5);
f_nummarkers(p,10,1);
for i=1:size(p,1)
    hasbehavior(p(i), 'legend', false);
end
xlabel('$t$','interpreter','latex');
ylabel('$\hat{W}_{c}(t)$','interpreter','latex');
% xlim([0 20])
legend('$\hat{W}_{c_{1}}$','$\hat{W}_{c_{2}}$','$\hat{W}_{c_{3}}$','$\hat{W}_{c_{4}}$','$\hat{W}_{c_{5}}$','$\hat{W}_{c_{6}}$','interpreter','latex','NumColumns',3)
l= set(l,'Interpreter','latex','Location','northeast','NumColumns',2);
set(gca,'FontSize',25)
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperUnits', 'inches');
set(gcf, 'PaperSize', [8 4]);
set(gcf, 'PaperPosition', [0 0 8 4]);
drawnow; 
grid on
print -dpdf Time_Vs_States_Weights_MBRL_SE_Optimal__F-16.png
print -dpdf Time_Vs_States_Weights_MBRL_SE_Optimal__F-16.pdf

