%% Data pre processing script to analyze the trajectories of the Baxter Robot
%% Michail Theofanidis

clc
clear all
close all

%% Load Baxter robot
mdl_baxter

%% Load data
%date = "2_15_23";
date = "5_3_23\Set_2";
num_demo = 13;
desired_length=500;
padding=0.01*desired_length;
cmap = jet(num_demo);

for i=1:num_demo
    path = append("Data\",date,"\demos\demo",num2str(i),".txt");
    data = readtable(path);
    joint_data{i} = data{:,["time","right_s0","right_s1","right_e0","right_e1","right_w0","right_w1","right_w2"]};
    legend_cell{i} = append('demo ',num2str(i));

%     if i==12
%         joint_data{i}=flip(joint_data{i});
%     end
    
end

%% Resample data
for i=1:num_demo

    data=joint_data{i};
    interp_joint_positions=zeros(desired_length-2*padding,8);

    t1=data(:,1);
    fs=floor(length(t1)/(t1(end)-t1(1)));
    t2=linspace(t1(1),t1(2),desired_length-2*padding);

    interp_joint_positions(:,1)=t2(1:end);

    for j=1:7

        q1=data(:,j+1);
        q2 = resample(q1,desired_length,length(q1),5,20);
        interp_joint_positions(:,j+1)=q2(padding+1:end-padding);
    end

    % Force final joint to be zero
    %interp_joint_positions=[interp_joint_positions(:,1:7) zeros(length(q2(padding+1:end-padding)),1)];

    interp_joint_data{i}=interp_joint_positions;

end

%% Generate Cartesian Data
for i=1:num_demo

%     rotation=zeros(length(interp_joint_data{i}(:,2:end)),3);
% 
%     for j=1:length(interp_joint_data{i}(:,2:end))
% 
%         [theta,rot]=tr2angvec(right.fkine(interp_joint_data{i}(j,2:end)));
%         
%         rot(1)=abs(rot(1));
%         
%         rot(2)=abs(rot(2));
%         
%         rotation(j,:)=rot;
% 
%     end

%     gripper_data{i}=[transl(right.fkine(interp_joint_data{i}(:,2:end))) rotation];
    gripper_data{i}=transl(right.fkine(interp_joint_data{i}(:,2:end)));
    
end

%% Plot the data
figure(1);
axis equal
grid on

for j=1:7

    subplot(2, 4, j) 
    hold on
    for i=1:num_demo

        joint = joint_data{i}(:,j+1);
        plot(joint,'Color',cmap(i,:))
        xlabel("time (secs)")
        ylabel(append("joint ",num2str(j-1)," (rad)"))
       
    end
    hold off
end
legend(legend_cell)

figure(2);
axis equal
grid on

for j=1:7

    subplot(2, 4, j) 
    hold on
    for i=1:num_demo

        joint = interp_joint_data{i}(:,j+1);
        plot(joint,'Color',cmap(i,:))
        xlabel("time (secs)")
        ylabel(append("joint ",num2str(j-1)," (rad)"))
       
    end
    hold off
end
legend(legend_cell)

%% Plot Cartesian Data
figure(3)
axis equal
grid on
hold on
for i=1:num_demo
    plot3(gripper_data{i}(:,1),gripper_data{i}(:,2),gripper_data{i}(:,3),'Color',cmap(i,:))
end
hold off

%% Plot Cartesian Data individually
figure(4);
axis equal
grid on

for j=1:3

    subplot(1, 3, j) 
    hold on
    for i=1:num_demo

        gripper = gripper_data{i}(:,j);
        plot(gripper,'Color',cmap(i,:))
       
    end
    hold off
end
legend(legend_cell)

% figure(5);
% axis equal
% grid on

% for j=4:6
% 
%     subplot(1, 3, j-3) 
%     hold on
%     for i=1:num_demo
% 
%         gripper = gripper_data{i}(:,j);
%         plot(gripper,'Color',cmap(i,:))
%        
%     end
%     hold off
% end
% legend(legend_cell)

%% Save the data
matname = append("Data\",date,"\demo.mat");
save(matname,'interp_joint_data','gripper_data','joint_data')




