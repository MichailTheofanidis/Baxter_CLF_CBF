function [pos1,pos2,pos3,pos4,pos5,pos6,pos7,vel1,vel2,vel3,vel4,vel5,vel6,vel7,u,TttT] = QP_CLFCBF(x1,y1,z1,z2,z3,z4,dt,robot)

x0 = [x1(1),x1(2),x1(3),x1(4),x1(5),x1(6),x1(7),x1(8),x1(9),x1(10),x1(11),x1(12), x1(13), x1(14)]; 
y0 = [y1(1),y1(2),y1(3),y1(4),y1(5),y1(6),y1(7),y1(8),y1(9),y1(10),y1(11),y1(12),y1(13),y1(14),y1(15),y1(16),y1(17),y1(18),y1(19),y1(20),y1(21)];
z01= [z1(1),z1(2),z1(3),z1(4),z1(5),z1(6),z1(7),z1(8),z1(9),z1(10),z1(11),z1(12)];
z02= [z2(1),z2(2),z2(3),z2(4),z2(5),z2(6),z2(7),z2(8),z2(9),z2(10),z2(11),z2(12)];
z03= [z3(1),z3(2),z3(3),z3(4),z3(5),z3(6),z3(7),z3(8),z3(9),z3(10),z3(11),z3(12)];
z04= [z4(1),z4(2),z4(3),z4(4),z4(5),z4(6)];

%%% Read the robot matrices
M=robot.inertia([x1(1), x1(2), x1(3), x1(4), x1(5), x1(6), x1(7)]);
C=robot.coriolis([x1(1), x1(2), x1(3), x1(4), x1(5), x1(6), x1(7)],[x1(8), x1(9), x1(10), x1(11), x1(12), x1(13), x1(14)]);
K=robot.gravload([x1(1), x1(2), x1(3), x1(4), x1(5), x1(6), x1(7)]);

C = (C*[x1(8), x1(9), x1(10), x1(11), x1(12), x1(13), x1(14)]')';

%%%% Linear System as the model of each joint after feedback linearization
F=[0 1;0 0];
G=[0;1];

%%% eps1 - eps6 are the parameters of CLF for 6 joints
eps1=.1; eps2=.1; eps3=.1; eps4=.1; eps5=.1; eps6=.01; eps7=.01;
gamma=1.1;

%%% Designing the Lyapunov matrices
[P,~,~] = care(F,G,eye(2));

P_eps1=[1/eps1 0;0 1]*P*[1/eps1 0;0 1];
P_eps2=[1/eps2 0;0 1]*P*[1/eps2 0;0 1];
P_eps3=[1/eps3 0;0 1]*P*[1/eps3 0;0 1];
P_eps4=[1/eps4 0;0 1]*P*[1/eps4 0;0 1];
P_eps5=[1/eps5 0;0 1]*P*[1/eps5 0;0 1];
P_eps6=[1/eps6 0;0 1]*P*[1/eps6 0;0 1];
P_eps7=[1/eps7 0;0 1]*P*[1/eps7 0;0 1];

eta1=[x0(1)-y0(1);x0(7)-y0(7)];
eta2=[x0(2)-y0(2);x0(8)-y0(8)];
eta3=[x0(3)-y0(3);x0(9)-y0(9)];
eta4=[x0(4)-y0(4);x0(10)-y0(10)];
eta5=[x0(5)-y0(5);x0(11)-y0(11)];
eta6=[x0(6)-y0(6);x0(12)-y0(12)];
eta7=[x0(7)-y0(7);x0(13)-y0(14)];

%% Quadratic Program for joint 1
%%% CLF
psi0_1 = eta1'*(F'*P_eps1 + P_eps1*F)*eta1 + (gamma/eps1)*eta1'*P_eps1*eta1; 
psi1_1 = 2*eta1'*P_eps1*G;
    
Aclf1=[psi1_1 -1];bclf1=-psi0_1;

%%% CBF parameters
delta=1.15; alphaE=2; betaE=1; Gam=1.5;
alphaE=20.1; betaE=1; Gam=10.1;

%%% Safety sets
h1_1 = x0(1)-y0(1)+ z01(4);
h1_1D = x0(8)-y0(8)+ z01(5);
h2_1 = z01(1) - (x0(1)-y0(1));
h2_1D = z01(2) - (x0(8)-y0(8));

%%% Two barrier functions
B1_1 = -log(h1_1/(1+h1_1))+alphaE*betaE*h1_1D^2/(1+betaE*h1_1D^2);
B2_1 = -log(h2_1/(1+h2_1))+alphaE*betaE*h2_1D^2/(1+betaE*h2_1D^2);

%%% Two Barrier conditions
phi01_1 = -h1_1D/(h1_1*(1+h1_1))+2*alphaE*betaE*h1_1D/(1+betaE*h1_1D^2)^2*z01(6)-Gam/B1_1;
phi11_1=2*alphaE*betaE*h1_1D/(1+betaE*h1_1D^2)^2;

phi02_1 = -h2_1D/(h2_1*(1+h2_1))+2*alphaE*betaE*h2_1D/(1+betaE*h2_1D^2)^2*z01(3)-Gam/B2_1;
phi12_1=-2*alphaE*betaE*h2_1D/(1+betaE*h2_1D^2)^2;

Acbf11=[phi11_1 0];bcbf11=-phi01_1;
Acbf12=[phi12_1 0];bcbf12=-phi02_1;

A1 = [Aclf1;Acbf11;Acbf12];
b1 = [bclf1;bcbf11;bcbf12];

%%% Wight on CLF, This wight gives the priority between CLF and CBF
%psc=2e2;    %%% To Give more priority to the CLF
 psc=1e-3;   %%% To give less priority to the CLF, and more to the CBF

 H = [1 0;0 psc];f = [0;0];

 %%% First QP
options = optimoptions('quadprog','Algorithm','interior-point-convex','Display','off');
tic
[mu11] = quadprog(H,f,A1,b1,[],[],[],[],[],options);
Ttt1=toc;
mu1=mu11(1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Quadratic Program for joint 2

%%% CLF condition
psi0_2 = eta2'*(F'*P_eps2 + P_eps2*F)*eta2 + (gamma/eps2)*eta2'*P_eps2*eta2; 
psi1_2 = 2*eta2'*P_eps2*G;

Aclf2=[psi1_2 -1];bclf2=-psi0_2;

%%% Safety sets
h1_2 = x0(2)-y0(2)+ z01(10);
h1_2D = x0(9)-y0(9) + z01(11);
h2_2 = z01(7) - (x0(2)-y0(2));
h2_2D = z01(8) - (x0(9)-y0(9));
%%% Two Barrier functions
B1_2 = -log(h1_2/(1+h1_2))+alphaE*betaE*h1_2D^2/(1+betaE*h1_2D^2);
B2_2 = -log(h2_2/(1+h2_2))+alphaE*betaE*h2_2D^2/(1+betaE*h2_2D^2);

%%% Two barrier conditions
phi01_2 = -h1_2D/(h1_2*(1+h1_2))+2*alphaE*betaE*h1_2D/(1+betaE*h1_2D^2)^2*z01(12)-Gam/B1_2;
phi11_2=2*alphaE*betaE*h1_2D/(1+betaE*h1_2D^2)^2;

phi02_2 = -h2_2D/(h2_2*(1+h2_2))+2*alphaE*betaE*h2_2D/(1+betaE*h2_2D^2)^2*z01(9)-Gam/B2_2;
phi12_2=-2*alphaE*betaE*h2_2D/(1+betaE*h2_2D^2)^2;

Aclf2=[psi1_2 -1];bclf2=-psi0_2;
Acbf21=[phi11_2 0];bcbf21=-phi01_2;
Acbf22=[phi12_2 0];bcbf22=-phi02_2;

A2 = [Aclf2;Acbf21;Acbf22];
b2 = [bclf2;bcbf21;bcbf22];

%%% Wight on CLF, This wight gives the priority between CLF and CBF       
%psc=1e1;     

%%% Second QP
H = [1 0;0 psc];f = [0;0];

options = optimoptions('quadprog','Algorithm','interior-point-convex','Display','off');
tic
[mu22] = quadprog(H,f,A2,b2,[],[],[],[],[],options);
Ttt2=toc;
mu2=mu22(1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Quadratic Program for joint 3
%%% CLF
psi0_3 = eta3'*(F'*P_eps3 + P_eps3*F)*eta3 + (gamma/eps3)*eta3'*P_eps3*eta3; 
psi1_3 = 2*eta3'*P_eps3*G;
    
Aclf3=[psi1_3 -1];bclf3=-psi0_3;

%%% CBF parameters
% delta=1.15; alphaE=1; betaE=1; Gam=.5;
%%% Safety sets
h1_3 = x0(3)-y0(3)+ z02(4);
h1_3D = x0(10)-y0(10)+ z02(5);
h2_3 = z02(1) - (x0(3)-y0(3));
h2_3D = z02(2) - (x0(10)-y0(10));

%%% Two Barrier functions
B1_3 = -log(h1_3/(1+h1_3))+alphaE*betaE*h1_3D^2/(1+betaE*h1_3D^2);
B2_3 = -log(h2_3/(1+h2_3))+alphaE*betaE*h2_3D^2/(1+betaE*h2_3D^2);

%%% Two barrier conditions
phi01_3 = -h1_3D/(h1_3*(1+h1_3))+2*alphaE*betaE*h1_3D/(1+betaE*h1_3D^2)^2*z02(6)-Gam/B1_3;
phi11_3=2*alphaE*betaE*h1_3D/(1+betaE*h1_3D^2)^2;

phi02_3 = -h2_3D/(h2_3*(1+h2_3))+2*alphaE*betaE*h2_3D/(1+betaE*h2_3D^2)^2*z02(3)-Gam/B2_3;
phi12_3=-2*alphaE*betaE*h2_3D/(1+betaE*h2_3D^2)^2;

Acbf31=[phi11_3 0];bcbf31=-phi01_3;
Acbf32=[phi12_3 0];bcbf32=-phi02_3;

A3 = [Aclf3;Acbf31;Acbf32];
b3 = [bclf3;bcbf31;bcbf32];

%%% Wight on CLF, This wight gives the priority between CLF and CBF
% psc=1e1;       

%%% Thirs QP       
H = [1 0;0 psc];f = [0;0];

options = optimoptions('quadprog','Algorithm','interior-point-convex','Display','off');
tic
[mu33] = quadprog(H,f,A3,b3,[],[],[],[],[],options);
Ttt3=toc;
mu3=mu33(1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Quadratic Program for joint 4
%%% CLF
psi0_4 = eta4'*(F'*P_eps4 + P_eps4*F)*eta4 + (gamma/eps4)*eta4'*P_eps4*eta4; 
psi1_4 = 2*eta4'*P_eps4*G;
    
Aclf4=[psi1_4 -1];bclf4=-psi0_4;

%%% Safety sets
h1_4 = x0(4)-y0(4)+ z02(10);
h1_4D = x0(11)-y0(11) + z02(11);
h2_4 = z02(7) - (x0(4)-y0(4));
h2_4D = z02(8) - (x0(11)-y0(11));

%%% Two CBFs
B1_4 = -log(h1_4/(1+h1_4))+alphaE*betaE*h1_4D^2/(1+betaE*h1_4D^2);
B2_4 = -log(h2_4/(1+h2_4))+alphaE*betaE*h2_4D^2/(1+betaE*h2_4D^2);

%%% barrier conditions
phi01_4 = -h1_4D/(h1_4*(1+h1_4))+2*alphaE*betaE*h1_4D/(1+betaE*h1_4D^2)^2*z02(12)-Gam/B1_4;
phi11_4=2*alphaE*betaE*h1_4D/(1+betaE*h1_4D^2)^2;

phi02_4 = -h2_4D/(h2_4*(1+h2_4))+2*alphaE*betaE*h2_4D/(1+betaE*h2_4D^2)^2*z02(9)-Gam/B2_4;
phi12_4=-2*alphaE*betaE*h2_4D/(1+betaE*h2_4D^2)^2;

Acbf41=[phi11_4 0];bcbf41=-phi01_4;
Acbf42=[phi12_4 0];bcbf42=-phi02_4;

A4 = [Aclf4;Acbf41;Acbf42];
b4 = [bclf4;bcbf41;bcbf42];

%%% Wight on CLF, This wight gives the priority between CLF and CBF       
% psc=1e1;       
H = [1 0;0 psc];f = [0;0];


%%% Fourth QP
options = optimoptions('quadprog','Algorithm','interior-point-convex','Display','off');
tic
[mu44] = quadprog(H,f,A4,b4,[],[],[],[],[],options);
Ttt4=toc;
mu4=mu44(1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Quadratic Program for joint 5
%%% CLF
psi0_5 = eta5'*(F'*P_eps5 + P_eps5*F)*eta5 + (gamma/eps5)*eta5'*P_eps5*eta5; 
psi1_5 = 2*eta5'*P_eps5*G;
    
Aclf5=[psi1_5 -1];bclf5=-psi0_5;

%%% CBF parameters
% delta=1.15; alphaE=1; betaE=1; Gam=.5;

%%% Safety sets
h1_5 = x0(5)-y0(5)+ z03(4);
h1_5D = x0(12)-y0(12)+ z03(5);
h2_5 = z03(1) - (x0(5)-y0(5));
h2_5D = z03(2) - (x0(12)-y0(12));

%%% Two barrier functions
B1_5 = -log(h1_5/(1+h1_5))+alphaE*betaE*h1_5D^2/(1+betaE*h1_5D^2);
B2_5 = -log(h2_5/(1+h2_5))+alphaE*betaE*h2_5D^2/(1+betaE*h2_5D^2);

%%% Two barrier conditions
phi01_5 = -h1_5D/(h1_5*(1+h1_5))+2*alphaE*betaE*h1_5D/(1+betaE*h1_5D^2)^2*z03(6)-Gam/B1_5;
phi11_5=2*alphaE*betaE*h1_5D/(1+betaE*h1_5D^2)^2;

phi02_5 = -h2_5D/(h2_5*(1+h2_5))+2*alphaE*betaE*h2_5D/(1+betaE*h2_5D^2)^2*z03(3)-Gam/B2_5;
phi12_5=-2*alphaE*betaE*h2_5D/(1+betaE*h2_5D^2)^2;

Acbf51=[phi11_5 0];bcbf51=-phi01_5;
Acbf52=[phi12_5 0];bcbf52=-phi02_5;

A5 = [Aclf5;Acbf51;Acbf52];
b5 = [bclf5;bcbf51;bcbf52];


%%% Wight on CLF, This wight gives the priority between CLF and CBF
% psc=1e1;            
H = [1 0;0 psc];f = [0;0];

%%% Fifth QP
options = optimoptions('quadprog','Algorithm','interior-point-convex','Display','off');
tic
[mu55] = quadprog(H,f,A5,b5,[],[],[],[],[],options);
Ttt5=toc;
mu5=mu55(1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Quadratic Program for joint 6
%%% CLF
psi0_6 = eta6'*(F'*P_eps6 + P_eps6*F)*eta6 + (gamma/eps6)*eta6'*P_eps6*eta6; 
psi1_6 = 2*eta6'*P_eps6*G;
    
Aclf6=[psi1_6 -1];bclf6=-psi0_6;

%%% Safety sets
h1_6 = x0(6)-y0(6)+ z03(10);
h1_6D = x0(13)-y0(13) + z03(11);
h2_6 = z03(7) - (x0(6)-y0(6));
h2_6D = z03(8) - (x0(13)-y0(13));
%%%Two Barrier functions
B1_6 = -log(h1_6/(1+h1_6))+alphaE*betaE*h1_6D^2/(1+betaE*h1_6D^2);
B2_6 = -log(h2_6/(1+h2_6))+alphaE*betaE*h2_6D^2/(1+betaE*h2_6D^2);

%%% Two barrier conditions
phi01_6 = -h1_6D/(h1_6*(1+h1_6))+2*alphaE*betaE*h1_6D/(1+betaE*h1_6D^2)^2*z03(12)-Gam/B1_6;
phi11_6=2*alphaE*betaE*h1_6D/(1+betaE*h1_6D^2)^2;

phi02_6 = -h2_6D/(h2_6*(1+h2_6))+2*alphaE*betaE*h2_6D/(1+betaE*h2_6D^2)^2*z03(9)-Gam/B2_6;
phi12_6=-2*alphaE*betaE*h2_6D/(1+betaE*h2_6D^2)^2;

Acbf61=[phi11_6 0];bcbf61=-phi01_6;
Acbf62=[phi12_6 0];bcbf62=-phi02_6;

A6 = [Aclf6;Acbf61;Acbf62];
b6 = [bclf6;bcbf61;bcbf62];

%%% Wight on CLF, This wight gives the priority between CLF and CBF     
% psc=1e1;       
H = [1 0;0 psc];f = [0;0];

%%% Sixth QP
options = optimoptions('quadprog','Algorithm','interior-point-convex','Display','off');
tic
[mu66] = quadprog(H,f,A6,b6,[],[],[],[],[],options);
Ttt6=toc;
mu6=mu66(1);

%% Quadratic Program for joint 7
%%% CLF
psi0_7 = eta7'*(F'*P_eps7 + P_eps7*F)*eta7 + (gamma/eps7)*eta7'*P_eps7*eta7; 
psi1_7 = 2*eta7'*P_eps7*G;
    
Aclf7=[psi1_7 -1];bclf7=-psi0_7;

%%% Safety sets
h1_7 = abs(x0(7)-y0(7)+ z04(4));
h1_7D = x0(14)-y0(14) + z04(5);
h2_7 = z04(1) - (x0(7)-y0(7));
h2_7D = z04(2) - (x0(14)-y0(14));
%%%Two Barrier functions
h1_7/(1+h1_7);
B1_7 = -log(h1_7/(1+h1_7))+alphaE*betaE*h1_7D^2/(1+betaE*h1_7D^2);
B2_7 = -log(h2_7/(1+h2_7))+alphaE*betaE*h2_7D^2/(1+betaE*h2_7D^2);

%%% Two barrier conditions
phi01_7=-h1_7D/(h1_7*(1+h1_7))+2*alphaE*betaE*h1_7D/(1+betaE*h1_7D^2)^2*z04(3)-Gam/B1_7;
phi11_7=2*alphaE*betaE*h1_7D/(1+betaE*h1_7D^2)^2;

phi02_7 = -h2_7D/(h2_7*(1+h2_7))+2*alphaE*betaE*h2_7D/(1+betaE*h2_6D^2)^2*z04(6)-Gam/B2_7;
phi12_7=-2*alphaE*betaE*h2_7D/(1+betaE*h2_7D^2)^2;

Acbf71=[phi11_7 0];bcbf71=-phi01_7;
Acbf72=[phi12_7 0];bcbf72=-phi02_7;

A7 = [Aclf7;Acbf71;Acbf72];
b7 = [bclf7;bcbf71;bcbf72];

%%% Wight on CLF, This wight gives the priority between CLF and CBF     
% psc=1e1;       
H = [1 0;0 psc];f = [0;0];

%%% Sixth QP
options = optimoptions('quadprog','Algorithm','interior-point-convex','Display','off');
tic
[mu77] = quadprog(H,f,A7,b7,[],[],[],[],[],options);
Ttt6=toc;
mu7=mu77(1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Feedback Linearization and simulating the robot dynamics %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

TttT=[Ttt1;Ttt2;Ttt3;Ttt4;Ttt5;Ttt6];    
%%%Feedback linearization
mu=[mu1;mu2;mu3;mu4;mu5;mu6;mu7]

u = C+K+(M*(mu+[y1(15);y1(16);y1(17);y1(18);y1(19);y1(20);y1(21)]))';
 
%%% Simulating the robot dynamics
t=[0 dt];

[~,xx] = ode45(@baxter_dynamics, t, [x0, u(1), u(2), u(3), u(4), u(5), u(6), u(7)], [], robot);

pos1 = xx(end, 1); pos2 = xx(end, 2); pos3 = xx(end, 3); pos4 = xx(end, 4); pos5 = xx(end, 5); pos6 = xx(end, 6); pos7 = xx(end, 7); 
vel1 = xx(end, 8); vel2 = xx(end, 9); vel3 = xx(end, 10); vel4 = xx(end, 11); vel5 = xx(end, 12); vel6 = xx(end, 13); vel7 = xx(end, 14);

end

function dx = baxter_dynamics(t, x, robot)
    
    M=robot.inertia([x(1), x(2), x(3), x(4), x(5), x(6), x(7)]);
    C=robot.coriolis([x(1), x(2), x(3), x(4), x(5), x(6), x(7)],[x(8), x(9), x(10), x(11), x(12), x(13), x(14)]);
    K=robot.gravload([x(1), x(2), x(3), x(4), x(5), x(6), x(7)]);
    
    C = (C*[x(8), x(9), x(10), x(11), x(12), x(13), x(14)]')';
    
    dx = zeros(21,1); 
    u = [x(15);x(16);x(17);x(18);x(19);x(20);x(21)];
    
    dx=[x(8); x(9); x(10); x(11); x(12); x(13); x(14);];
     
    DD = -M\(C)'-M\(K)'+M\u; 
    
    dx(8)=DD(1); dx(9)=DD(2); dx(10)=DD(3);dx(11)=DD(4); dx(12)=DD(5); dx(13)=DD(6); dx(14)=DD(7);
    dx(15) = 0; dx(16) = 0; dx(17) = 0;dx(18) = 0; dx(19) = 0; dx(20) = 0; dx(21) = 0;

 
end
