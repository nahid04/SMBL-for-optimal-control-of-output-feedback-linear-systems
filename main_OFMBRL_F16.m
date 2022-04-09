%% Paper title "Safe Controller for Output Feedback Linear Systems using Model-Based Reinforcement Learning"
%% arXiv:2204.01409
%% Coded by S M Nahid Mahmud, MS Grad Student, Oklahoma State University.
%% nahid.mahmud@okstate.edu
%% This code presents a F16 aircraft's three-state longitudinal dynamical system for the state estimation
%% The notations are similar to the mentioned paper. 
%% Transformed system denotes the system we have after applying the barrier transformation to the original system.
%% In this document, x is the original state of the system, xH is the estimated state of the system, s is the original state of the transformed system,...
%% sH is the estimated state of the transformed system.

clear all
clc

tic
%% PreSim

z_low_1 = -0.1; z_up_1 = 0.1; %Barrier Boundaries for x2, x2-coordinate values will be remained within (z_low_1,z_up_1) 
z_low_2 = -0.1; z_up_2 = 0.1; %Barrier Boundaries for x3, x3-coordinate values will be remained within (z_low_2,z_up_2) 
z_low_3 = -0.1; z_up_3 = 0.1; %Barrier Boundaries for x4, x4-coordinate values will be remained within (z_low_3,z_up_3) 



P.n = 3; %number of states
P.d = 0; %fixed constant just to use later
P.m = 1; %number of dimension

%% Dynamics: please check the manuscript
P.A = [-1.01887,0.90506,-0.00215;
    0.82225,-1.07741,-0.17555;
    0,0,-1;
    ];
P.B = [0;0;1];
P.C = [1,0,0];
P.L = [1;1;1];
%%

P.x0 = [0.045;0;0.0393]; % Initial x-coordinate values, it has to be within (a,A).
P.xH0 = [0.05;.05;0.05];  % Initial x-coordinate values, it has to be within (a,A).

%transforming initial x-coordinates to initial s-coordinates. 
P.s10 = barrier(P.x0(1),z_low_1,z_up_1);
P.s20 = barrier(P.x0(2),z_low_2,z_up_2);
P.s30 = barrier(P.x0(3),z_low_3,z_up_3);
P.s0 = [P.s10;P.s20;P.s30]; % Initial s-coordinate values

%transforming initial x-coordinates to initial s-coordinates. 
P.s1H0 = barrier(P.xH0(1),z_low_1,z_up_1);
P.s2H0 = barrier(P.xH0(2),z_low_2,z_up_2);
P.s3H0 = barrier(P.xH0(3),z_low_3,z_up_3);
P.sH0 = [P.s1H0;P.s2H0;P.s3H0]; % Initial s-coordinate values


P.W = 6; % elements of the Actor-Critic Weight vector
P.G = 6;  % elements of the Gamma vector


P.Wa0 = 1*[ 1; 1; 1; 1; 1; 1]; % Initial value of the Actor weight vector. 
P.Wc0 = 1*[ 1; 1; 1; 1; 1; 1]; % Initial value of the Critic weight vector. 

%Initial values for x = 1-4, xH = 5-8, eta1 = 9-10, 
% WaH = 11-20, WcH = 21-30, Gamma = 31-130, s = 131-134, sH = 135-138.

% Control Penalty Matrix  
P.R = 1;
% State Penalty Matrix  
P.Q= 10*[1,0,0;0,1,0;0,0,1]; 


% Tuning Gains>> Please check the ArXiv paper to understand the notation
P.kc = 100;
P.ka1 = 100;
P.ka2 = 1;
P.beta = 0.1;
P.v = 1;

% WaH denotes Estimated Actor weight, WcH denotes Estimated Critic weight.
P.z0 = [P.x0;P.xH0;P.Wa0;P.Wc0;1*reshape(eye(6),36,1);P.s0;P.sH0;];
%% Integration
%ODE45 function
% options = odeset('OutputFcn',@odeplot,'OutputSel',P.n+1:P.n+P.l);
[t,z] = ode45(@(t,z) closedLoopDynamics_F16(t,z,P),[0 20],P.z0); %ODE45 function

toc
save 'new_test_F-16.mat';


