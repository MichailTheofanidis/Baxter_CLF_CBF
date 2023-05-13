%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% CBF-Based ProMP Controller %%%%%%%%%%%%%%
clc;
clear all;
close all


%% Load Baxter robot
mdl_baxter

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

%date = '2_15_23';
date = "5_3_23\Set_2";
num_joints = 7;

for i=1:num_joints
    path = append("Data\",date,"\joint_data\baxter_promp_joint_",num2str(i),".txt");
    joints{i} = importdata(path);
end

%% Sampling time
dt=.01;

%% Reading the ProMP data (mean and standard deviation)
f1=joints{1,1}(1,:); f2=joints{1,2}(1,:); f3=joints{1,3}(1,:); f4=joints{1,4}(1,:); f5=joints{1,5}(1,:); f6=joints{1,6}(1,:); f7=joints{1,7}(1,:);
f1_bar=joints{1,1}(3,:)-f1; f2_bar=joints{1,2}(3,:)-f2; f3_bar=joints{1,3}(3,:)-f3; f4_bar=joints{1,4}(3,:)-f4; f5_bar=joints{1,5}(3,:)-f5; f6_bar=joints{1,6}(3,:)-f6; f7_bar=joints{1,7}(3,:)-f7;
f1_und=f1-joints{1,1}(2,:); f2_und=f2-joints{1,2}(2,:); f3_und=f3-joints{1,3}(2,:); f4_und=f4-joints{1,4}(2,:); f5_und=f5-joints{1,5}(2,:); f6_und=f6-joints{1,6}(2,:); f7_und=f7-joints{1,7}(2,:);

plot(f1); hold on; plot(f1+f1_bar); plot(f1-f1_und);
figure; plot(f2); hold on; plot(f2+f2_bar); plot(f2-f2_und);
figure; plot(f3); hold on; plot(f3+f3_bar); plot(f3-f3_und);
figure; plot(f4); hold on; plot(f4+f4_bar); plot(f4-f4_und);
figure; plot(f5); hold on; plot(f5+f5_bar); plot(f5-f5_und);
figure; plot(f6); hold on; plot(f6+f6_bar); plot(f6-f6_und);

%% Initial conditions; 7 joint positions and 7 velocities
x1=[f1(1) f2(1) f3(1) f4(1) f5(1) f6(1) f7(1) 0 0 0 0 0 0 0];

%% Calculation of the first and second derivatives of the ProMP mean and standard deviation
fD1=[];fD2=[];fD3=[];fD4=[];fD5=[];fD6=[];fD7=[];
f1_barD=[]; f1_undD=[]; f2_barD=[]; f2_undD=[]; f3_barD=[]; f3_undD=[]; f4_barD=[]; f4_undD=[]; f5_barD=[]; f5_undD=[]; f6_barD=[]; f6_undD=[]; f7_barD=[]; f7_undD=[];

for i=1:length(f1)-1
 %t=(i-1)*dt;
 df1=(f1(i+1)-f1(i))/dt;
 fD1=[fD1 df1];
 df1_bar=(f1_bar(i+1)-f1_bar(i))/dt;
 f1_barD=[f1_barD df1_bar];
 df1_und=(f1_und(i+1)-f1_und(i))/dt;
 f1_undD=[f1_undD df1_und];
 %%
 df2=(f2(i+1)-f2(i))/dt;
 fD2=[fD2 df2];
 df2_bar=(f2_bar(i+1)-f2_bar(i))/dt;
 f2_barD=[f2_barD df2_bar];
 df2_und=(f2_und(i+1)-f2_und(i))/dt;
 f2_undD=[f2_undD df2_und];
 %%
 df3=(f3(i+1)-f3(i))/dt;
 fD3=[fD3 df3];
 df3_bar=(f3_bar(i+1)-f3_bar(i))/dt;
 f3_barD=[f3_barD df3_bar];
 df3_und=(f3_und(i+1)-f3_und(i))/dt;
 f3_undD=[f3_undD df3_und];
 %%
 df4=(f4(i+1)-f4(i))/dt;
 fD4=[fD4 df4];
 df4_bar=(f4_bar(i+1)-f4_bar(i))/dt;
 f4_barD=[f4_barD df4_bar];
 df4_und=(f4_und(i+1)-f4_und(i))/dt;
 f4_undD=[f4_undD df4_und];
 %%
 df5=(f5(i+1)-f5(i))/dt;
 fD5=[fD5 df5];
 df5_bar=(f5_bar(i+1)-f5_bar(i))/dt;
 f5_barD=[f5_barD df5_bar];
 df5_und=(f5_und(i+1)-f5_und(i))/dt;
 f5_undD=[f5_undD df5_und];
 %%
 df6=(f6(i+1)-f6(i))/dt;
 fD6=[fD6 df6];
 df6_bar=(f6_bar(i+1)-f6_bar(i))/dt;
 f6_barD=[f6_barD df6_bar];
 df6_und=(f6_und(i+1)-f6_und(i))/dt;
 f6_undD=[f6_undD df6_und];
 %%
 df7=(f7(i+1)-f7(i))/dt;
 fD7=[fD7 df7];
 df7_bar=(f7_bar(i+1)-f7_bar(i))/dt;
 f7_barD=[f7_barD df7_bar];
 df7_und=(f7_und(i+1)-f7_und(i))/dt;
 f7_undD=[f7_undD df7_und];
end

fDD1=[];fDD2=[];fDD3=[];fDD4=[];fDD5=[];fDD6=[];fDD7=[];
f1_barDD=[];f1_undDD=[]; f2_barDD=[];f2_undDD=[];f3_barDD=[];f3_undDD=[]; f4_barDD=[];f4_undDD=[]; f5_barDD=[];f5_undDD=[]; f6_barDD=[];f6_undDD=[]; f7_barDD=[];f7_undDD=[];

for i=1:length(f1)-2
    ddf1=(fD1(i+1)-fD1(i))/dt;
    fDD1=[fDD1 ddf1];
    ddf1_bar=(f1_barD(i+1)-f1_barD(i))/dt;
    f1_barDD=[f1_barDD ddf1_bar];
    ddf1_und=(f1_undD(i+1)-f1_undD(i))/dt;
    f1_undDD=[f1_undDD ddf1_und];
    %%
    ddf2=(fD2(i+1)-fD2(i))/dt;
    fDD2=[fDD2 ddf2];
    ddf2_bar=(f2_barD(i+1)-f2_barD(i))/dt;
    f2_barDD=[f2_barDD ddf2_bar];
    ddf2_und=(f2_undD(i+1)-f2_undD(i))/dt;
    f2_undDD=[f2_undDD ddf2_und];
    %%
    ddf3=(fD3(i+1)-fD3(i))/dt;
    fDD3=[fDD3 ddf3];
    ddf3_bar=(f3_barD(i+1)-f3_barD(i))/dt;
    f3_barDD=[f3_barDD ddf3_bar];
    ddf3_und=(f3_undD(i+1)-f3_undD(i))/dt;
    f3_undDD=[f3_undDD ddf3_und];
    %%
    ddf4=(fD4(i+1)-fD4(i))/dt;
    fDD4=[fDD4 ddf4];
    ddf4_bar=(f4_barD(i+1)-f4_barD(i))/dt;
    f4_barDD=[f4_barDD ddf4_bar];
    ddf4_und=(f4_undD(i+1)-f4_undD(i))/dt;
    f4_undDD=[f4_undDD ddf4_und];
    %%
    ddf5=(fD5(i+1)-fD5(i))/dt;
    fDD5=[fDD5 ddf5];
    ddf5_bar=(f5_barD(i+1)-f5_barD(i))/dt;
    f5_barDD=[f5_barDD ddf5_bar];
    ddf5_und=(f5_undD(i+1)-f5_undD(i))/dt;
    f5_undDD=[f5_undDD ddf5_und];
    %%
    ddf6=(fD6(i+1)-fD6(i))/dt;
    fDD6=[fDD6 ddf6];
    ddf6_bar=(f6_barD(i+1)-f6_barD(i))/dt;
    f6_barDD=[f6_barDD ddf6_bar];
    ddf6_und=(f6_undD(i+1)-f6_undD(i))/dt;
    f6_undDD=[f6_undDD ddf6_und];
    %%
    ddf7=(fD7(i+1)-fD7(i))/dt;
    ddf7=(fD7(i+1)-fD7(i))/dt;
    fDD7=[fDD7 ddf7];
    ddf7_bar=(f7_barD(i+1)-f7_barD(i))/dt;
    f7_barDD=[f7_barDD ddf7_bar];
    ddf7_und=(f7_undD(i+1)-f7_undD(i))/dt;
    f7_undDD=[f7_undDD ddf7_und];
end

%% Designing the controller and obtaining the robot trajectories
for i=1:length(f1)-2
    t=(i-1)*dt;   

    y1=[f1(i) f2(i) f3(i) f4(i) f5(i) f6(i) f7(i) fD1(i) fD2(i) fD3(i) fD4(i) fD5(i) fD6(i) fD7(i) fDD1(i) fDD2(i) fDD3(i) fDD4(i) fDD5(i) fDD6(i) fDD7(i)];
    z1=[f1_bar(i) f1_barD(i) f1_barDD(i) f1_und(i) f1_undD(i) f1_undDD(i) f2_bar(i) f2_barD(i) f2_barDD(i) f2_und(i) f2_undD(i) f2_undDD(i)];
    z2=[f3_bar(i) f3_barD(i) f3_barDD(i) f3_und(i) f3_undD(i) f3_undDD(i) f4_bar(i) f4_barD(i) f4_barDD(i) f4_und(i) f4_undD(i) f4_undDD(i)];
    z3=[f5_bar(i) f5_barD(i) f5_barDD(i) f5_und(i) f5_undD(i) f5_undDD(i) f6_bar(i) f6_barD(i) f6_barDD(i) f6_und(i) f6_undD(i) f6_undDD(i)];
    z4=[f7_bar(i) f5_barD(i) f7_barDD(i) f7_und(i) f7_undD(i) f7_undDD(i)];

    tic
    [pos1,pos2,pos3,pos4,pos5,pos6,pos7,vel1,vel2,vel3,vel4,vel5,vel6,vel7,u,TttT] = QP_CLFCBF(x1,y1,z1,z2,z3,z4,dt,right);
    Ttt(i)=toc;
    Totaltime(i,:)=[TttT];
    
    %% Saving the data for analysis and plot
    x1=[pos1 pos2 pos3 pos4 pos5 pos6 pos7 vel1 vel2 vel3 vel4 vel5 vel6 vel7];
    
    control(i,:) = u;
    simdata1(i,:) = [t x1(1) x1(2) x1(3) x1(4) x1(5) x1(6) x1(7) x1(8) x1(9) x1(10) x1(11) x1(12) x1(13) x1(14)]; 
    simdata2(i,:) = [t y1(1) y1(2) y1(3) y1(4) y1(5) y1(6) y1(7) y1(8) y1(9) y1(10) y1(11) y1(12) y1(13) y1(14) y1(15) y1(16) y1(17) y1(18) y1(19) y1(20) y1(21)];    
    simdata3(i,:) = [t z1(1) z1(7) z2(1) z2(7) z3(1) z3(7) z4(1)];    
end

% % %%% A number of statistical tests %%%%%%%%%%%%%%%%%%%%%%%%%%%
% kstest2(Totaltime(1,:),Totaltime(2,:));
% stdT=std(Totaltime);
% maxT=max(Totaltime);
% meanT=mean(Totaltime);
% boxplot(Totaltime);
% %profile viewer
% 
%%% Plotting %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time=simdata1(:,1);
position1=simdata1(:,2); positionD1=simdata2(:,2);
position2=simdata1(:,3); positionD2=simdata2(:,3);
position3=simdata1(:,4); positionD3=simdata2(:,4);
position4=simdata1(:,5); positionD4=simdata2(:,5);
position5=simdata1(:,6); positionD5=simdata2(:,6);
position6=simdata1(:,7); positionD6=simdata2(:,7);
position7=simdata1(:,8); positionD7=simdata2(:,8);
% velocity1=simdata1(:,4);
% velocity2=simdata1(:,5);

% %%%%%%%%%%%%%%%%%%%%%%%% Plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure
plot(time,position1,'linewidth',2);hold on; grid on
plot(time,positionD1,'--','linewidth',2);hold on;
plot(time, positionD1+simdata3(:,2),'k:')
plot(time, positionD1-simdata3(:,2),'k:')
xlabel('Time')
ylabel('Joint 1 Pos')
figure
plot(time,position2,'linewidth',2);hold on; grid on
plot(time,positionD2,'--','linewidth',2);hold on;
plot(time, positionD2+simdata3(:,3),'k:')
plot(time, positionD2-simdata3(:,3),'k:')
xlabel('Time')
ylabel('Joint 2 Pos')
figure
plot(time,position3,'linewidth',2);hold on; grid on
plot(time,positionD3,'--','linewidth',2);hold on;
plot(time, positionD3+simdata3(:,4),'k:')
plot(time, positionD3-simdata3(:,4),'k:')
xlabel('Time')
ylabel('Joint 3 Pos')
figure
plot(time,position4,'linewidth',2);hold on; grid on
plot(time,positionD4,'--','linewidth',2);hold on;
plot(time, positionD4+simdata3(:,5),'k:')
plot(time, positionD4-simdata3(:,5),'k:')
xlabel('Time')
ylabel('Joint 4 Pos')
figure
plot(time,position5,'linewidth',2);hold on; grid on
plot(time,positionD5,'--','linewidth',2);hold on;
plot(time, positionD5+simdata3(:,6),'k:')
plot(time, positionD5-simdata3(:,6),'k:')
xlabel('Time')
ylabel('Joint 5 Pos')
figure
plot(time,position6,'linewidth',2);hold on; grid on
plot(time,positionD6,'--','linewidth',2);hold on;
plot(time, positionD6+simdata3(:,7),'k:')
plot(time, positionD6-simdata3(:,7),'k:')
xlabel('Time')
ylabel('Joint 6 Pos')
figure
plot(time,position7,'linewidth',2);hold on; grid on
plot(time,positionD7,'--','linewidth',2);hold on;
plot(time, positionD7+simdata3(:,8),'k:')
plot(time, positionD7-simdata3(:,8),'k:')
xlabel('Time')
ylabel('Joint 7 Pos')
