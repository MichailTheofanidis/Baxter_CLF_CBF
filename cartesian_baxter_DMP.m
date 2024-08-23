%% Compute DMP polices for the Baxter Robot
%% Michail Theofanidis

close all
clear all
clc

%% Load Baxter robot
mdl_baxter
wd=[0.001 0.001 0.001 0.002]; % Variable to widen the boundaries of the cbf

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
start_demo=1;
num_demo = 10;
date = "8_31_23";
path = append("Data\",date,"\demo.mat");
save_path = append("Data\",date,"\Results\");
load(path)

index=700;

% Store the cartesian data
for i=start_demo:num_demo

    data.l{i}=length(cartesian_data{i}(index:end,1));
    data.time{i}=abs(linspace(joint_data{i}(index,1)-joint_data{i}(index,1),joint_data{i}(end,1)-joint_data{i}(index,1),data.l{i})');

    data.position{i}=[smooth(cartesian_data{i}(index:end,1)), smooth(cartesian_data{i}(index:end,2)), smooth(cartesian_data{i}(index:end,3))];
    data.dposition{i}=[vel(data.position{i}(:,1),data.time{i}), vel(data.position{i}(:,2),data.time{i}), vel(data.position{i}(:,3),data.time{i})];
    data.ddposition{i}=[vel(data.dposition{i}(:,1),data.time{i}), vel(data.dposition{i}(:,2),data.time{i}), vel(data.dposition{i}(:,3),data.time{i})];
 
end

%% Environment Coordinates
table_coord = [...
    0.357 -1.077 -0.08;
    0.105 -0.765 -0.08;
    0.780 0.034 -0.08;
    1.04 -0.265 -0.08;
    0.357 -1.077 -0.12;
    0.105 -0.765 -0.12;
    0.780 0.034 -0.12;
    1.04 -0.265 -0.12;];

idx = [4 8 5 1 4; 1 5 6 2 1; 2 6 7 3 2; 3 7 8 4 3; 5 8 7 6 5; 1 4 3 2 1]';

x_table = table_coord(:,1);
y_table = table_coord(:,2);
z_table = table_coord(:,3);

obs_coord = [...
    0.59 -0.64 -0.08;
    0.46 -0.49 -0.08;
    0.55 -0.39 -0.08;
    0.67 -0.54 -0.08;
    0.59 -0.64 0.08;
    0.46 -0.49 0.08;
    0.55 -0.39 0.08;
    0.67 -0.54 0.08;];

x_obs = obs_coord(:,1);
y_obs = obs_coord(:,2);
z_obs = obs_coord(:,3);

%% Learn the DMPs
for i=start_demo:num_demo

    dmp=Dmp();
    
    %% Calculate the phase of the dmp
    data.s{i}=dmp.phase(data.time{i});
    
    %% Calculate the spread out gaussians
    psV=dmp.distributions(data.s{i});
    
    %% Perform immitation learning on all cartesian dimensions
    ftarget=zeros(data.l{i},1);
    w=zeros(dmp.ng,3);

    [ftarget,w]=dmp.immitate(data.position{i},data.dposition{i},data.ddposition{i},data.time{i},data.s{i},psV);

    data.w{i}=w;

    %% Perform forward pass of the DMP
    position_r=zeros(data.l{i},3);

    position_r=dmp.generate(data.w{i},data.position{i}(1,:),data.position{i}(end,:),data.time{i},data.s{i},psV);
    
    data.position_r{i}=position_r;
    
end

% Plot the learned DMPs

fig(1)=figure(1);
set(0, 'defaultTextInterpreter', 'latex');
right.plot([0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50],'nowrist','noarrow','noname','view',[19,26],'workspace',[-0.1 1 -1 0.4 -0.5 1]);
patch(x_table(idx), y_table(idx), z_table(idx), 'w', 'facealpha', 0.6);
for i=start_demo:num_demo
    axis equal
    hold on
    h1=plot3(data.position{i}(1,1),data.position{i}(1,2),data.position{i}(1,3),'g*','LineWidth',1);
    h2=plot3(data.position{i}(:,1),data.position{i}(:,2),data.position{i}(:,3),'b','LineWidth',2);
    h3=plot3(data.position_r{i}(:,1),data.position_r{i}(:,2),data.position_r{i}(:,3),'--r','LineWidth',2);
    legend([h1 h2 h3],'Starting Position','Trajectory Demonstrations','DMP Trajectories')
    grid on
end
view(125,11)
xlabel('X (m)') 
ylabel('Y (m)')
zlabel('Z (m)')
f = gcf;
exportgraphics(f,append(save_path,"Learning_XYZ.png"),'Resolution',200)
hold off

%% Adapt the DMPs
[M,I]=max([data.l{:}]);
up_boundary=0.01;
low_boundary=0.01;
dmp_pad=0.5;

% Inital and Final conditions for different scenarions
% Scenario 1
conditions{1}.start=[0.81,-0.22, -0.03];
conditions{1}.end=[0.37,-0.74,-0.03];
% Scenario 2
conditions{2}.start=[0.65,-0.15,-0.03];
conditions{2}.end=[0.37,-0.74,-0.03];

obstacle.origin=[0.5675,-0.5150,-0.04];
obstacle.radius=[0.3 0.4];
obstacle.gains=0.02;
padding_x=[0.05, 0.05];
padding_y=[0.06, 0.06];
padding_z=[0.01, 0.01];
padding_z_obs=[0.05, 0.05];

store_file="Scenario";

[M,I]=max([data.l{:}]);

%% Compute the distribution of primitives
w_x=zeros(dmp.ng,num_demo);
w_y=zeros(dmp.ng,num_demo);
w_z=zeros(dmp.ng,num_demo);

for i=1:num_demo
    w_x(:,i)=data.w{i}(:,1); 
    w_y(:,i)=data.w{i}(:,2);
    w_z(:,i)=data.w{i}(:,3);
end

w_x_dist=zeros(dmp.ng,3);
w_y_dist=zeros(dmp.ng,3);
w_z_dist=zeros(dmp.ng,3);

boundary=dmp(1).ng;
for i=1:dmp(1).ng

    w_x_dist(i,1)=mean(w_x(i,:));
    w_y_dist(i,1)=mean(w_y(i,:));
    w_z_dist(i,1)=mean(w_z(i,:));

    if i<boundary
        w_x_dist(i,2)=mean(w_x(i,:))-dmp_pad*mean(w_x(i,:)); 
        w_x_dist(i,3)=mean(w_x(i,:))+dmp_pad*mean(w_x(i,:)); 
    
        w_y_dist(i,2)=mean(w_y(i,:))-dmp_pad*mean(w_y(i,:)); 
        w_y_dist(i,3)=mean(w_y(i,:))+dmp_pad*mean(w_y(i,:));
    
        w_z_dist(i,2)=mean(w_z(i,:))-dmp_pad*mean(w_z(i,:)); 
        w_z_dist(i,3)=mean(w_z(i,:))+dmp_pad*mean(w_z(i,:));
    else
        w_x_dist(i,2)=mean(w_x(i,:));
        w_x_dist(i,3)=mean(w_x(i,:));
    
        w_y_dist(i,2)=mean(w_y(i,:));
        w_y_dist(i,3)=mean(w_y(i,:));
    
        w_z_dist(i,2)=mean(w_z(i,:));
        w_z_dist(i,3)=mean(w_z(i,:));
    end

end

cnt=1;
cnt_inner=0;
fig_cnt=2;

for j =1:length(conditions)

    %% Generate the tube without obs
    dmp=Dmp();
    psV=dmp.distributions(data.time{I});
    
    % Mean bounds
    p_mean=dmp.generate([w_x_dist(:,1) w_y_dist(:,1) w_z_dist(:,1)],conditions{j}.start,conditions{j}.end,data.time{I},data.s{I},psV);
    factor=[linspace(0,1,length(p_mean)/2) 1 linspace(1,0.2,(length(p_mean)/2))];

    % Lower bounds
    p_low=dmp.generate([w_x_dist(:,2) w_y_dist(:,2) w_z_dist(:,2)],conditions{j}.start,conditions{j}.end-[low_boundary,low_boundary,0],data.time{I},data.s{I},psV);
    p_low(1:end,1)=p_low(1:end,1)-factor'*padding_x(j);
    p_low(1:end,2)=p_low(1:end,2)-factor'*padding_y(j);
    p_low(1:end,3)=p_low(1:end,3)-factor'*padding_z(j);
  
    % Upper bounds
    p_up=dmp.generate([w_x_dist(:,3) w_y_dist(:,3) w_z_dist(:,3)],conditions{j}.start,conditions{j}.end+[up_boundary,up_boundary,0],data.time{I},data.s{I},psV);
    p_up(1:end,1)=p_up(1:end,1)+factor'*padding_x(j);
    p_up(1:end,2)=p_up(1:end,2)+factor'*padding_y(j);
    p_up(1:end,3)=p_up(1:end,3)+factor'*padding_z(j);

    %% Generate the tube with obs
    dmp=Dmp([obstacle.origin obstacle.gains obstacle.radius(j)]);
    % Mean bounds
    p_mean_obs=dmp.generate([w_x_dist(:,1) w_y_dist(:,1) w_z_dist(:,1)],conditions{j}.start,conditions{j}.end,data.time{I},data.s{I},psV);

    % Lower bounds
    p_low_obs=dmp.generate([w_x_dist(:,2) w_y_dist(:,2) w_z_dist(:,2)],conditions{j}.start,conditions{j}.end-[low_boundary,low_boundary,0],data.time{I},data.s{I},psV);
    p_low_obs(1:end,1)=p_low_obs(1:end,1)-factor'*padding_x(j);
    p_low_obs(1:end,2)=p_low_obs(1:end,2)-factor'*padding_y(j);
    p_low_obs(1:end,3)=p_mean_obs(1:end,3)-factor'*padding_z_obs(j);
  
    % Upper bounds
    p_up_obs=dmp.generate([w_x_dist(:,3) w_y_dist(:,3) w_z_dist(:,3)],conditions{j}.start,conditions{j}.end+[up_boundary,up_boundary,0],data.time{I},data.s{I},psV);
    p_up_obs(1:end,1)=p_up_obs(1:end,1)+factor'*padding_x(j);
    p_up_obs(1:end,2)=p_up_obs(1:end,2)+factor'*padding_y(j);
    p_up_obs(1:end,3)=p_mean_obs(1:end,3)+factor'*padding_z_obs(j);

    dmp_data{j+cnt_inner}.x=[p_mean(:,1)';p_low(:,1)';p_up(:,1)';];
    dmp_data{j+cnt_inner}.y=[p_mean(:,2)';p_low(:,2)';p_up(:,2)';];
    dmp_data{j+cnt_inner}.z=[p_mean(:,3)';p_low(:,3)';p_up(:,3)';]
    dmp_data{j+cnt_inner+1}.x=[p_mean_obs(:,1)';p_low_obs(:,1)';p_up_obs(:,1)';];
    dmp_data{j+cnt_inner+1}.y=[p_mean_obs(:,2)';p_low_obs(:,2)';p_up_obs(:,2)';];
    dmp_data{j+cnt_inner+1}.z=[p_mean_obs(:,3)';p_low_obs(:,3)';p_up_obs(:,3)';];
    cnt_inner=cnt_inner+1;

end



%% Function that calculates derivatives
function [dq]=vel(q,t)

dq=zeros(length(t),1);

    for i = 1:length(t)-1
        dq(i+1) = (q(i+1)-q(i))/(t(i+1)-t(i));
    end

end

%% Function that adds parabolic blends to the joint trajectories
function [traj]=PolyTraj(q,dq,t,blends)

traj=zeros(length(q),1);
window=floor(length(q)/blends);

up=1;
down=window;

for i=1:blends
    
    if i==blends
        pad=(length(q)-down);
        window=window+pad;
        down=down+pad;
    end
    
    theta_s=q(up);
    theta_f=q(down);
    
    theta_dot_s=dq(up);
    theta_dot_f=dq(down);
    
    c = MyPoly3(theta_s,theta_f,t(window),theta_dot_s,theta_dot_f);
    dummy= Traj(c,t(1:window));
    
    traj(down-window+1:down)=dummy;
    
    up=down+1;
    down=down+window;
    
end

end

%% Polynomial function coefficients
function alpha = MyPoly3(theta_s,theta_f,time,theta_dot_s,theta_dot_f)

alpha(1)=theta_s;
alpha(2)=theta_dot_s;
alpha(3)=3*(theta_f-theta_s)/time^2-2*(theta_dot_s)/time-1*(theta_dot_f)/time;
alpha(4)=-2*(theta_f-theta_s)/time^3+(theta_dot_f+theta_dot_s)/time^2;

end

%% Polynomial fitting
function traj = Traj(alpha,time)

traj=zeros(1,length(time));

for i=1:length(time)
    
    traj(i)=alpha(1)+alpha(2)*time(i)+alpha(3)*time(i)^2+alpha(4)*time(i)^3;
    
end

end
