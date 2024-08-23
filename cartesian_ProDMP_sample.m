%% Compute ProDMP polices
%% Michail Theofanidis

close all
clear all
clc

%% Load Baxter robot
mdl_baxter

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

%Plot the learned DMPs
figure(1);
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
hold off

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

for i=1:dmp(1).ng

    w_x_dist(i,1)=mean(w_x(i,:));
    w_y_dist(i,1)=mean(w_y(i,:));
    w_z_dist(i,1)=mean(w_z(i,:));

    w_x_dist(i,2)=mean(w_x(i,:))-2*std(w_x(i,:)); 
    w_x_dist(i,3)=mean(w_x(i,:))+2*std(w_x(i,:)); 

    w_y_dist(i,2)=mean(w_y(i,:))-2*std(w_y(i,:)); 
    w_y_dist(i,3)=mean(w_y(i,:))+2*std(w_y(i,:));

    w_z_dist(i,2)=mean(w_z(i,:))-2*std(w_z(i,:)); 
    w_z_dist(i,3)=mean(w_z(i,:))+2*std(w_z(i,:));

end

%% Inital and Final conditions for different scenarions
% Scenario 1
conditions{1}.start=[0.81,-0.22, -0.03];
conditions{1}.end=[0.37,-0.74,-0.03];
% Scenario 2
conditions{2}.start=[0.65,-0.15,-0.03];
conditions{2}.end=[0.37,-0.74,-0.03];

%% Compute Trajectory Zones
[M,I]=max([data.l{:}]);
for j =1:length(conditions)

    dmp=Dmp();
    psV=dmp.distributions(data.time{I});
    
    % Mean bounds
    p_mean=dmp.generate([w_x_dist(:,1) w_y_dist(:,1) w_z_dist(:,1)],conditions{j}.start,conditions{j}.end,data.time{I},data.s{I},psV);

    % Lower bounds
    p_low=dmp.generate([w_x_dist(:,2) w_y_dist(:,2) w_z_dist(:,2)],conditions{j}.start,conditions{j}.end,data.time{I},data.s{I},psV);
  
    % Upper bounds
    p_up=dmp.generate([w_x_dist(:,3) w_y_dist(:,3) w_z_dist(:,3)],conditions{j}.start,conditions{j}.end,data.time{I},data.s{I},psV);

    dmp_data{j}.x=[p_mean(:,1)';p_low(:,1)';p_up(:,1)';];
    dmp_data{j}.y=[p_mean(:,2)';p_low(:,2)';p_up(:,2)';];
    dmp_data{j}.z=[p_mean(:,3)';p_low(:,3)';p_up(:,3)';];

end

% Plot the zones for scenario 1
figure(2);
plot(dmp_data{1}.x(1,:))
hold on
plot(dmp_data{1}.x(2,:))
plot(dmp_data{1}.x(3,:))

figure(3);
plot(dmp_data{1}.y(1,:))
hold on
plot(dmp_data{1}.y(2,:))
plot(dmp_data{1}.y(3,:))

figure(4);
plot(dmp_data{1}.z(1,:))
hold on
plot(dmp_data{1}.z(2,:))
plot(dmp_data{1}.z(3,:))

% Plot the zones for scenario 2
figure(5);
plot(dmp_data{2}.x(1,:))
hold on
plot(dmp_data{2}.x(2,:))
plot(dmp_data{2}.x(3,:))

figure(6);
plot(dmp_data{2}.y(1,:))
hold on
plot(dmp_data{2}.y(2,:))
plot(dmp_data{2}.y(3,:))

figure(7);
plot(dmp_data{2}.z(1,:))
hold on
plot(dmp_data{2}.z(2,:))
plot(dmp_data{2}.z(3,:))

%% Function that calculates derivatives
function [dq]=vel(q,t)

dq=zeros(length(t),1);

    for i = 1:length(t)-1
        dq(i+1) = (q(i+1)-q(i))/(t(i+1)-t(i));
    end

end