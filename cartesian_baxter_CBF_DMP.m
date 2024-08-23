%% Load DMP polices and execute them with a CLF/CBF Controller for the Baxter Robot
%% Michail Theofanidis

close all
clear all
clc

%% Load Baxter robot
mdl_baxter
wd=[0.002 0.002 0.002 0.002]; % Variable to widen the boundaries of the cbf

% Insert mass, inertia and mass distribution
right.links(1).m=5.70044;
right.links(2).m=3.22698;
right.links(3).m=4.31272;
right.links(4).m=2.07206;
right.links(5).m=2.24665;
right.links(6).m=1.60979;
right.links(7).m=0.54218;

right.links(1).r=[-0.05117, 0.07908, 0.00086];
right.links(2).r=[0.00269, -0.00529, 0.06845];
right.links(3).r=[-0.07176, 0.08149, 0.00132];
right.links(4).r=[0.00159, -0.01117, 0.02618];
right.links(5).r=[-0.01168, 0.13111, 0.0046];
right.links(6).r=[0.00697, 0.006, 0.06048];
right.links(7).r=[0.005117, 0.0009572, -0.06682];

right.links(1).I=[0.0470910226,-0.0061487003, 0.0001278755; -0.0061487003, 0.035959884, -0.0007808689; 0.0001278755, -0.0007808689, 0.0376697645];
right.links(2).I=[0.027885975, -0.0001882199, -0.00030096397; -0.0001882199, 0.020787492, 0.0020767576; -0.00030096397, 0.0020767576, 0.0117520941];
right.links(3).I=[0.0266173355, -0.0039218988, 0.0002927063; -0.0039218988, 0.012480083, -0.001083893; 0.0002927063, -0.001083893, 0.0284435520];
right.links(4).I=[0.0131822787, -0.0001966341, 0.0003603617; -0.0001966341, 0.009268520,  0.000745949; 0.0003603617,  0.000745949,  0.0071158268];
right.links(5).I=[0.0166774282, -0.0001865762,  0.0001840370; -0.0001865762, 0.003746311, 0.0006473235;  0.0001840370,  0.0006473235, 0.0167545726];
right.links(6).I=[0.0070053791,  0.0001534806,  -0.0004438478;  0.0001534806, 0.005527552, -0.0002111503; -0.0004438478, -0.0002111503, 0.0038760715];
right.links(7).I=[0.0008162135,  0.000128440,  0.00018969891;  0.000128440, 0.0008735012,  0.0001057726;  0.00018969891, 0.0001057726, 0.0005494148];

%% Load the data
date = "8_31_23";
path = append("Data\",date,"\dmp_data.mat");
load(path)

cnt=1;
index=[1, 3];

for j =1:2

    %% Pass the Results through the Controller
    for k=1:2
        
        widen=wd(cnt);

        if k==1
            field1 = 'x';  value1 = dmp_data{index(j)}.x;
            field2 = 'y';  value2 = dmp_data{index(j)}.y;
            field3 = 'z';  value3 = dmp_data{index(j)}.z;
        else
            field1 = 'x';  value1 = dmp_data{index(j)+1}.x;
            field2 = 'y';  value2 = dmp_data{index(j)+1}.y;
            field3 = 'z';  value3 = dmp_data{index(j)+1}.z;
        end

        field4 = 'dx'; value4 = zeros(length(value1),3);
        field5 = 'dy'; value5 = zeros(length(value1),3);
        field6 = 'dz'; value6 = zeros(length(value1),3);
        
        field7 = 'ddx'; value7 = zeros(length(value1),3);
        field8 = 'ddy'; value8 = zeros(length(value1),3);
        field9 = 'ddz'; value9 = zeros(length(value1),3);

        field10 = 'time'; value10=time;
        field11= 'p'; value11=  zeros(length(value1),1);
        field12= 'pu'; value12=  zeros(length(value1),1);

        cnt_data = struct(field1,value1',field2,value2',field3,value3',field4,value4,field5,value5, ...
                          field6,value6,field7,value7,field8,value8,field9,value9,field10,value10,field11,value11,field12,value12);
        
        % Rearrange the data so tha the 1rst element is the mean, the 2nd is the
        % upper boundary and the third is the lower boundary
        temp=cnt_data.x(:,2);
        cnt_data.x(:,2)=cnt_data.x(:,3)-cnt_data.x(:,1)+widen;
        cnt_data.x(:,3)=cnt_data.x(:,1)-temp+widen;
        
        temp=cnt_data.y(:,2);
        cnt_data.y(:,2)=cnt_data.y(:,3)-cnt_data.y(:,1)+widen;
        cnt_data.y(:,3)=cnt_data.y(:,1)-temp+widen;
        
        temp=cnt_data.z(:,2);
        cnt_data.z(:,2)=cnt_data.z(:,3)-cnt_data.z(:,1)+widen;
        cnt_data.z(:,3)=cnt_data.z(:,1)-temp+widen;
        
        %% Calculation of the first and second derivatives of the ProMP mean and standard deviation
        
        % Sampling time
        dt=.01;
        
        for w=1:3
        
            cnt_data.dx(2:end,w)=diff(cnt_data.x(:,w))./dt;
            cnt_data.ddx(2:end,w)=diff(cnt_data.dx(:,w))./dt;
        
            cnt_data.dy(2:end,w)=diff(cnt_data.y(:,w))./dt;
            cnt_data.ddy(2:end,w)=diff(cnt_data.dy(:,w))./dt;
        
            cnt_data.dz(2:end,w)=diff(cnt_data.z(:,w))./dt;
            cnt_data.ddz(2:end,w)=diff(cnt_data.dz(:,w))./dt;
        
        end

        %% Use IK to start to find the starting point of the robot
    
        % Initialize IK
        initialGuess = [0.233,-0.246,-0.502,0.803,0.555,1.019,-2.337];
        baxterOrientation = axang2rotm([0 1 0 pi]);
    
        % Fix the orientation of the end effector
        [~,rot_target]=tr2angvec(baxterOrientation);
        pose = [baxterOrientation [cnt_data.x(1,1) cnt_data.y(1,1) cnt_data.z(1,1)]'; 0 0 0 1];
        
        q_init=right.ikine(pose,initialGuess) % initial joint position for possible IK
        x_init=transl(right.fkine(q_init)); % initial cartesian position of the robot

        %% Controller Implementation

        % Null Space controller gain 
        Gain=1;
        
        % CLF parameters
        F=[0 1;0 0];
        G=[0;1];
        [P,~,~] = care(F,G,eye(2));
        
        eps1=.1; eps2=.1; eps3=.1; 
        gamma1=1.1; gamma2=1.1; gamma3=1.1; 
        
        % CBF parameters

        %psc1=200; psc2=200; psc3=200; %results1
        psc1=0.04; psc2=0.04; psc3=0.04; %results2
 
        
        %Final set
%         alphaE1=20.1; betaE1=1.1; Gam1=45.1; %results1
%         alphaE2=20.1; betaE2=1.1; Gam2=45.1; %results1
%         alphaE3=20.1; betaE3=1.1; Gam3=45.1; %results1
        
        alphaE1=7.1; betaE1=1.1; Gam1=10.1; 
        alphaE2=7.1; betaE2=1.1; Gam2=10.1;
        alphaE3=7.1; betaE3=1.1; Gam3=10.1;
        
        if k==2 && j==2

            alphaE1=1.1; betaE1=1; Gam1=10; 
            alphaE2=1.1; betaE2=1; Gam2=10;
            alphaE3=1.1; betaE3=1; Gam3=10;
        end
        

        % Initial joint state variables
        q=zeros(length(cnt_data.x),7);
        dq=zeros(length(cnt_data.x),7);
        ddq=zeros(length(cnt_data.x),7);
        q(1,:)=q_init;
        
        % Initialize cartesian state variables
        p=zeros(length(cnt_data.x),3);
        dp=zeros(length(cnt_data.x),3);
        ddp=zeros(length(cnt_data.x),3);
        pu=zeros(length(cnt_data.x),3);
        p(1,:)=x_init;
        
        for i=2:length(cnt_data.x)-2
            
            % Time vector
            t=(i-1)*dt;   
            
            %% Read the robot matrices
            M=right.inertia(q(i-1,:));
            C=right.coriolis(q(i-1,:),dq(i-1,:));
            C=(C*(dq(i-1,:))');
            K=right.gravload(q(i-1,:))';
        
            %% CLF and CBF
            % The last variable in the clf_cbf function determines if the
            % controller runs both the cbf/clf or just the clf controller.
            % Use 1 to simple use the clf, while 0 to use the clf/cbf controller.
        
            % QP for x dimension
            mu1=clf_cbf(alphaE1,betaE1,Gam1,F,G,P,eps1,psc1,gamma1,p(i-1,1),dp(i-1,1),cnt_data,'x',i-1,'0');
            
            % QP for y dimension
            mu2=clf_cbf(alphaE2,betaE2,Gam2,F,G,P,eps2,psc2,gamma2,p(i-1,2),dp(i-1,2),cnt_data,'y',i-1,'0');
        
            % QP for z dimension
            mu3=clf_cbf(alphaE3,betaE3,Gam3,F,G,P,eps3,psc3,gamma3,p(i-1,3),dp(i-1,3),cnt_data,'z',i-1,'0');
            
            %% Feedback Linearization and Null space controller
            disp('---------------------')
            i
        
            mu=[mu1;mu2;mu3] % Vector with the cartesian acceleration that the QPs compute
            pu(i,:)=[mu1,mu2,mu3];

            J=right.jacob0(q(i-1,:));
            J=J(1:3,:); % Cartesian Jacobian
        
            dJ=right.jacob_dot(q(i-1,:),dq(i-1,:));
            dJ=dJ(1:3);  % Cartesian derivative of the Jacobian
        
            u_null=(eye(7,7)-transpose(J)*pinv(transpose(J)))*(Gain*(q_init(1,:)'-q(i-1,:)')); % Null space controller
            u_null=[0;0;u_null(3);0;u_null(5);0;0;]; 
        
            u = C+K+(M*pinv(J)*(-dJ+mu+[ddp(i,1); ddp(i,2); ddp(i,3)]))+u_null; % Sum the CLF/CBF controller and the Null space controller
        
            %% Simulating the robot dynamics
            state=[q(i-1,:),dq(i-1,:)];
            [~,xx] = ode45(@baxter_dynamics, [0 dt], [state, u(1), u(2), u(3), u(4), u(5), u(6), u(7)], [], right);
        
            %% Next joint state
            q(i,:)=xx(end, 1:7);
            dq(i,:)=xx(end, 8:14);
            ddq(i,:)=(dq(i,:)-dq(i-1,:))/dt;
        
            %% Next cartesian state
            p(i,:)=transl(right.fkine(q(i,:)));
        
            J=right.jacob0(q(i,:));
            J=J(1:3,:);
        
            temp=right.jacob_dot(q(i,:),dq(i,:));
            dJ=temp(1:3);
        
            dp(i,:)=(J*dq(i,:)')';
            ddp(i,:)=dJ+J*ddq(i,:)';
        
        end

        cnt_data.p=p;
        cnt_data.pu=pu;
        results{cnt}=cnt_data;
        cnt=cnt+1;

    end

end

%% Store the data
matname = append("Data\",date,"\results5.mat");
save(matname,'results')

%% Forward Dynamics

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

%% CLF/CBF Controller

function u =clf_cbf(alphaE,betaE,Gam,F,G,P,eps,psc, gamma,p,dp,data,dim,id,cnt_id)
                
    % CLF
    P_eps=[1/eps 0;0 1]*P*[1/eps 0;0 1];
    eta=[p-data.(dim)(id,1);dp-data.(append('d',dim))(id,1)];
    psi0 = eta'*(F'*P_eps + P_eps*F)*eta + (gamma/eps)*eta'*P_eps*eta; 
    psi1 = 2*eta'*P_eps*G;
        
    Aclf=[psi1 -1];bclf=-psi0;
  
    if cnt_id=='0'
        % Safety sets
        h1_1 = p-data.(dim)(id,1)+ data.(dim)(id,3);
        h1_1D = dp-data.(append('d',dim))(id,1)+ data.(append('d',dim))(id,3);
        h2_1 = data.(dim)(id,2)-(p-data.(dim)(id,1));
        h2_1D = data.(append('d',dim))(id,2)-(dp-data.(append('d',dim))(id,1));
        
        % Two barrier functions
        B1_1 = -log(h1_1/(1+h1_1))+alphaE*betaE*h1_1D^2/(1+betaE*h1_1D^2);
        B2_1 = -log(h2_1/(1+h2_1))+alphaE*betaE*h2_1D^2/(1+betaE*h2_1D^2);
         
        % Two Barrier conditions
        phi01_1 = -h1_1D/(h1_1*(1+h1_1))+2*alphaE*betaE*h1_1D/(1+betaE*h1_1D^2)^2*data.(append('dd',dim))(id,3)-Gam/B1_1
        phi11_1=2*alphaE*betaE*h1_1D/(1+betaE*h1_1D^2)^2;
        
        phi02_1 = -h2_1D/(h2_1*(1+h2_1))+2*alphaE*betaE*h2_1D/(1+betaE*h2_1D^2)^2*data.(append('dd',dim))(id,2)-Gam/B2_1
        phi12_1=-2*alphaE*betaE*h2_1D/(1+betaE*h2_1D^2)^2;
        
        Acbf11=[phi11_1 0];bcbf11=-phi01_1;
        Acbf12=[phi12_1 0];bcbf12=-phi02_1;

        A = [Aclf;Acbf11;Acbf12];
        b = [bclf;bcbf11;bcbf12];
    end

    if cnt_id=='1'
        A = [Aclf];
        b = [bclf];
    end
    
    H = [1 0;0 psc];f = [0;0];
    
    % QP
    options = optimoptions('quadprog','Algorithm','interior-point-convex','Display','off');
    u = quadprog(H,f,A,b,[],[],[],[],[],options);
    
    if  isempty( u )
        A = [Aclf];
        b = [bclf];
        options = optimoptions('quadprog','Algorithm','interior-point-convex','Display','off');
        u = quadprog(H,f,A,b,[],[],[],[],[],options)
        u=u(1);

    else
        u=u(1);
    end
end

