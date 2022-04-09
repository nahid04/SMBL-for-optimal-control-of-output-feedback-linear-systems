%% Paper title "Safe Controller for Output Feedback Linear Systems using Model-Based Reinforcement Learning"
%% arXiv:2204.01409
%% Coded by S M Nahid Mahmud, MS Grad Student, Oklahoma State University.
%% nahid.mahmud@okstate.edu
%% This code presents a F16 aircraft's three-state longitudinal dynamical system for the state estimation
%% The notations are similar to the mentioned paper. 
%% Transformed system denotes the system we have after applying the barrier transformation to the original system.
%% In this document, x is the original state of the system, xH is the estimated state of the system, s is the original state of the transformed system,...
%% sH is the estimated state of the transformed system.

%% Main Function 
function zDot = closedLoopDynamics_F16(t,z,P)

z_low_1 = -0.1; z_up_1 = 0.1; %Barrier Boundaries for x2, x2-coordinate values will be remained within (z_low_1,z_up_1) 
z_low_2 = -0.1; z_up_2 = 0.1; %Barrier Boundaries for x3, x3-coordinate values will be remained within (z_low_2,z_up_2) 
z_low_3 = -0.1; z_up_3 = 0.1; %Barrier Boundaries for x4, x4-coordinate values will be remained within (z_low_3,z_up_3) 

% of the zDot matrix given at the end of this coding.  
x = z(1:P.n,1);% row 1-3 in z vector
xH = z(P.n+1:2*P.n,1);% row 4-6 in z vector 
WaH = z(2*P.n+P.d+1:2*P.n+P.d+P.W); % row 7-12 in z vector
WcH = z(2*P.n+P.d+P.W+1:2*P.n+P.d+2*P.W); % row 13-18 in z vector
Gamma = reshape(z(2*P.n+P.d+2*P.W+1:2*P.n+P.d+2*P.W+P.G^2),P.G,P.G);  % row 19-54 in z vector 
s = z(2*P.n+P.d+2*P.W+P.G^2+1: 2*P.n+P.d+2*P.W+P.G^2+P.n,1); % row 55-57 in z vector
sH = z( 2*P.n+P.d+2*P.W+P.G^2+P.n+1:2*P.n+P.d+2*P.W+P.G^2+2*P.n,1); % row 58-60 in z vector
%% BE Extrapolation - Terms in weight update laws that are evaluated along arbitrarily selected trajectories ki


%Creating a meshgrid to train to get the optimal actor weight and critic weight along the meshgrid
%points using BE extrapolation 
X1 = linspace(-0.08,0.08,5);
X2 = linspace(-0.08,0.08,5);
X3 = linspace(-0.08,0.08,5);
M = length(X1)*length(X2)*length(X3);
[XX,XY,XZ]=ndgrid(X1,X2,X3);
XC=[reshape(XX,M,1) reshape(XY,M,1) reshape(XZ,M,1) ]';


%initialization of the trained weights in the meshgrid 
clWc=zeros(size(WcH)); %initialization of the critic weights in the meshgrid
clWa=zeros(size(WaH)); %initialization of the actor weights in the meshgrid
CLmatrix=zeros(size(WcH,1),size(WcH,1)); %Matrixized
CLGamma=zeros(size(Gamma)); %initialization of the Gamma weights in the meshgrid

for i=1:M
    ki=XC(:,i); % Vectorized
    
z_low_1 = -0.1; z_up_1 = 0.1; %Barrier Boundaries for x2, x2-coordinate values will be remained within (z_low_1,z_up_1) 
z_low_2 = -0.1; z_up_2 = 0.1; %Barrier Boundaries for x3, x3-coordinate values will be remained within (z_low_2,z_up_2) 
z_low_3 = -0.1; z_up_3 = 0.1; %Barrier Boundaries for x4, x4-coordinate values will be remained within (z_low_3,z_up_3) 


    si_1 = barrier(ki(1),z_low_1,z_up_1);
    si_2 = barrier(ki(2),z_low_2,z_up_2);
    si_3 = barrier(ki(3),z_low_3,z_up_3);
    si = [si_1;si_2;si_3];

%transforming initial s-coordinates to initial x-coordinates in the meshgrid. 
x1_si = z_low_1*z_up_1*((exp(si(1)))-1)/((z_low_1*exp(si(1)))-z_up_1);
x2_si = z_low_2*z_up_2*((exp(si(2)))-1)/((z_low_2*exp(si(2)))-z_up_2);
x3_si = z_low_3*z_up_3*((exp(si(3)))-1)/((z_low_3*exp(si(3)))-z_up_3);
x_si = [x1_si;x2_si;x3_si];


%Note: We do not need to do the barrier transformation in the meshgrid to train
%our controller as this method is an extrapolation method, and we are picking some random
%points (around the meshgrid) to train. Therefore, we just need to create a
%meshgrid to train, but here we are doing the barrier transformation just
%to mimic the environment, not sure whether it gives a better result or
%not. 


%Transformation matrix in the meshgrid 
Ts_si = [(((z_low_1^2*exp(si(1))) - (2*z_low_1*z_up_1) + (z_up_1^2 * exp (-si(1)))) / ( z_up_1*z_low_1^2 -z_low_1*z_up_1^2));
    (((z_low_2^2*exp(si(2))) - (2*z_low_2*z_up_2) + (z_up_2^2 * exp (-si(2)))) / ( z_up_2*z_low_2^2 -z_low_2*z_up_2^2));
    (((z_low_3^2*exp(si(3))) - (2*z_low_3*z_up_3) + (z_up_3^2 * exp (-si(3)))) / ( z_up_3*z_low_3^2 -z_low_3*z_up_3^2));
   ];


%Dynamics in the meshgrid 
F_si =   Ts_si.*(P.A * si);

G_si =  Ts_si.*(P.B);


%ADP-Controller
% Basis vector in the meshgrid: [si(1)^2, si(1)si(2),si(2)^2]
phi_p_si = [si(2),si(1),0;si(3),0,si(1);...
    0,si(3),si(2);...
    2*si(1),0,0;0,2*si(2),0;...
    0,0,2*si(3)]; % Jacobian of the basis vector in the meshgrid
mu_si = -0.5*inv(P.R)*(G_si)'*phi_p_si'*WaH; % trained controller of the meshgrid
%Definition
omegai = phi_p_si*(F_si+G_si*mu_si);
Gsigmai = phi_p_si*G_si*(P.R\G_si')*phi_p_si';  
rhoi=(1+P.v*(omegai'*omegai));
 
%Cost function in the meshgrid 
 ri = si'*P.Q*si + mu_si'*P.R*mu_si;
% 
%     sig_pi =[2*xi(1) 0; xi(2) xi(1); 0 2*xi(2)]; 
%     mui=-0.5*(R\Gi')*sig_pi'*WaH;
%     omegai = sig_pi*(Fi+Gi*mui);
%     rhoi=(1+v*(omegai'*omegai));
%     ri = xi'*Q*xi + mui'*R*mui;
deltai = WcH'*omegai + ri;
% actor weight, critic weight, Gamma update law  
    clWc=clWc+Gamma*omegai*deltai/rhoi;
    clWa=clWa+Gsigmai'*WaH*omegai'*WcH/(4*rhoi);
    CLmatrix=CLmatrix+omegai*omegai'/(M*rhoi);
    CLGamma=CLGamma+Gamma*(omegai*omegai'/rhoi^2)*Gamma;
end
clWc=-P.kc*clWc/M;
clWa=P.kc*clWa/M;
CLGamma=CLGamma/M;
cbar =min(svd(CLmatrix));


%% Original dynamics

Y1 = P.A*x;
G1 = P.B;

%% Estimated original dynamics
YH1 = P.A*xH;
GH1 = P.B;

%Definition according to the paper
s1H = sH(1);
s1 = s(1);
s1Tilde = s1-s1H;
s2H = sH(2);
s2 = s(2);
s2Tilde = s2-s2H;
s3H = sH(3);
s3 = s(3);
s3Tilde = s3-s3H;


s = [s1;s2;s3];
sH = [s1H;s2H;s3H];
sTilde= [s1Tilde;s2Tilde;s3Tilde];

%Transformation matrix in the meshgrid
Ts = [(((z_low_1^2*exp(s(1))) - (2*z_low_1*z_up_1) + (z_up_1^2 * exp (-s(1)))) / ( z_up_1*z_low_1^2 -z_low_1*z_up_1^2));
    (((z_low_2^2*exp(s(2))) - (2*z_low_2*z_up_2) + (z_up_2^2 * exp (-s(2)))) / ( z_up_2*z_low_2^2 -z_low_2*z_up_2^2));
    (((z_low_3^2*exp(s(3))) - (2*z_low_3*z_up_3) + (z_up_3^2 * exp (-s(3)))) / ( z_up_3*z_low_3^2 -z_low_3*z_up_3^2));
    ];

TsH = [(((z_low_1^2*exp(sH(1))) - (2*z_low_1*z_up_1) + (z_up_1^2 * exp (-sH(1)))) / ( z_up_1*z_low_1^2 -z_low_1*z_up_1^2));
        (((z_low_2^2*exp(sH(2))) - (2*z_low_2*z_up_2) + (z_up_2^2 * exp (-sH(2)))) / ( z_up_2*z_low_2^2 -z_low_2*z_up_2^2));
        (((z_low_3^2*exp(sH(3))) - (2*z_low_3*z_up_3) + (z_up_3^2 * exp (-sH(3)))) / ( z_up_3*z_low_3^2 -z_low_3*z_up_3^2));
        ];

%%transforming initial s-coordinates to initial x-coordinates. 

%%Transformed dynamics
Fs =  Ts.* P.A * x;

Gs =  Ts.* P.B;



%%Transformed estimated dynamics
FsH =  TsH.* P.A * xH;

GsH =  TsH.* P.B;


%%
%% Terms in weight update laws that are evaluated along the system trajectory x


%% Jacobian of estimated basis vector
phi_pH = [sH(2),sH(1),0;sH(3),0,sH(1);...
    0,sH(3),sH(2);...
    2*sH(1),0,0;0,2*sH(2),0;...
    0,0,2*sH(3)];
%Learned controller of the transformed system
 muH = -0.5.*(P.R\GsH')*phi_pH'*WaH;

%Time step checking
t

%Integrating function
zDot = [Y1+G1*muH;
       (P.B*muH)+(((P.A*xH)+(P.L*P.C*sTilde))./TsH);
        (P.ka1*(WcH-WaH))-(P.ka2*WaH)+clWa;
        clWc;
        reshape(P.beta*Gamma-CLGamma,P.G^2,1);
        Fs+Gs*muH;
        (P.A*xH+TsH.*P.B*muH+(P.L*P.C*sTilde));
       ];
end