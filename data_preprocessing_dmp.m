%% Data pre processing script to compute dmps from cartesian trajectory data
%% Michail Theofanidis

clc
clear all
close all

%% Load Baxter robot
mdl_baxter

%% Load the data
date = "8_31_23";
num_demo = 10;
cmap = jet(num_demo);
%flip=[13,17,20,24,25,26,27,28,29,30,31,32,34];

counter=1;
for i=1:num_demo
    path = append("Data\",date,"\demo",num2str(i),".txt");
    data = readtable(path);

    if i==flip(counter) 
        data=flipud(data);
        counter=counter+1;
    end

    joint_data{i} = data{:,["time","right_s0","right_s1","right_e0","right_e1","right_w0","right_w1","right_w2"]};

end

%% Compute the cartesian data
for i=1:num_demo

    cartesian_data{i}=transl(right.fkine(joint_data{i}(:,2:end)));
    
end

%% Plot Data
figure(1);

for j=1:7

    subplot(2, 4, j) 
    hold on
    for i=1:num_demo

        joint = joint_data{i}(:,j+1);
        plot(joint,'Color',cmap(i,:))
        xlabel("samples")
        ylabel(append("joint ",num2str(j-1)," (rad)"))
       
    end
    hold off
    grid on

end

figure(2);
axis equal
grid on

for i=1:num_demo

    plot3(cartesian_data{i}(:,1),cartesian_data{i}(:,2),cartesian_data{i}(:,3),'Color',cmap(i,:))
    hold on
    axis equal
    grid on

end

%% Store the data
matname = append("Data\",date,"\demo.mat");
save(matname,'cartesian_data','joint_data')
