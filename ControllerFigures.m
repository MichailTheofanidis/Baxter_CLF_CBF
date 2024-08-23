%% Plot the response of the ProDMP CBF/CLF Controller
%% Michail Theofanidis

close all
clear all
clc

%% Load the data
mdl_baxter;
date = "8_31_23";
data = importdata(append("Data\",date,"\results2.mat"));
save_path = append("Data\",date,"\Results\");

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

% Inital and Final conditions for different scenarions
% Scenario 1
conditions{1}.start=[0.81,-0.22, -0.03];
conditions{1}.end=[0.37,-0.74,-0.03];
% Scenario 2
conditions{2}.start=[0.65,-0.15,-0.03];
conditions{2}.end=[0.37,-0.74,-0.03];

store_file="Scenario";
orng = [0.8500 0.3250 0.0980];

%% Plot the Results
fig_cnt=1;


t=data{1}.time;
counter=0;
for i =1:2

    set(0, 'defaultTextInterpreter', 'latex');
    figure(fig_cnt);
    right.plot([0.08, -1.0,  1.19, 1.94, -0.67, 1.03,  0.50],'nowrist','noarrow','noname','view',[19,26],'workspace',[-0.5 1 -1 0.6 -0.5 1]);
    patch(x_table(idx), y_table(idx), z_table(idx), 'w', 'facealpha', 0.2);
    patch(x_obs(idx), y_obs(idx), z_obs(idx), 'w', 'facealpha', 0.2);
    hold on
    axis equal
    h1=plot3(data{i+counter}.x(:,1),data{i+counter}.y(:,1),data{i+counter}.z(:,1),'b','LineWidth',2.0);
    h2=plot3(data{i+counter+1}.x(:,1),data{i+counter+1}.y(:,1),data{i+counter+1}.z(:,1),'r','LineWidth',2.0);
    h3=plot3(conditions{i}.start(1),conditions{i}.start(2),conditions{i}.start(3),'g*','LineWidth',3.0);
    h4=plot3(conditions{i}.end(1),conditions{i}.end(2),conditions{i}.end(3),'k*','LineWidth',3.0);
    grid on
    view(105,45)
    xlabel('X (m)') 
    ylabel('Y (m)')
    zlabel('Z (m)')
    view(121,25)
    hold off
    if i==2
        lgd =legend([h1 h2 h3 h4],'Mean Trajectory','Modified Mean Trajectory','Starting Location', 'Goal Location');
        lgd.Location='northeast';
    end
    f = gcf;
    exportgraphics(f,append(save_path,store_file,num2str(i),"\","Sc",num2str(i),".eps"),'Resolution',100)
    %saveas(fig(fig_cnt),append(save_path,store_file,num2str(i),"\","Sc",num2str(i),".png"))
    fig_cnt=fig_cnt+1;

    fig(fig_cnt)=figure(fig_cnt);
    hold on
    %plot(data{1}.time,data{i+counter}.x(:,1)-data{i+counter}.x(:,3),'--b','LineWidth',1.0)
    h1=plot(data{1}.time,data{i+counter}.x(:,1),'b','LineWidth',2.0);
    h2=plot(data{1}.time(1:end-2),data{i+counter}.p(1:end-2,1),'--c','LineWidth',2.0);;
    %plot(data{1}.time,data{i+counter}.x(:,1)+data{i+counter}.x(:,2),'--b','LineWidth',1.0)
    h3=patch([data{1}.time; flipud(data{1}.time)], [data{i+counter}.x(:,1)-data{i+counter}.x(:,3); flipud(data{i+counter}.x(:,1)+data{i+counter}.x(:,2))], 'c');;
    %plot(data{1}.time,data{i+counter+1}.x(:,1)-data{i+counter+1}.x(:,3),'--r','LineWidth',1.0);
    h4=plot(data{1}.time,data{i+counter+1}.x(:,1),'r','LineWidth',2.0);
    h5=plot(data{1}.time(1:end-2),data{i+counter+1}.p(1:end-2,1),'--','Color',[0.9290 0.6940 0.1250],'LineWidth',3.0);
    %plot(data{1}.time,data{i+counter+1}.x(:,1)+data{i+counter+1}.x(:,2),'--r','LineWidth',1.0)
    h6=patch([data{1}.time; flipud(data{1}.time)], [data{i+counter+1}.x(:,1)-data{i+counter+1}.x(:,3); flipud(data{i+counter+1}.x(:,1)+data{i+counter+1}.x(:,2))], 'r');
    alpha(0.2)
    h7=plot(t(1),conditions{i}.start(1),'g*','LineWidth',3.0);
    h8=plot(t(end),conditions{i}.end(1),'k*','LineWidth',3.0);
    grid on
    xlabel('time (sec)') 
    ylabel('X (m)')
    hold off
    if i==2
        lgd =legend([h3 h2 h6 h5],'Cartesian Zone', 'CLF/CBF Response','Modified Cartesian Zone', 'Modified CLF/CBF Response');
        lgd.Location='northeast';
    end
    f = gcf;
    exportgraphics(f,append(save_path,store_file,num2str(i),"\","Sc",num2str(i),"_X.eps"),'Resolution',100)
    %saveas(fig(fig_cnt),append(save_path,store_file,num2str(i),"\","Sc",num2str(i),"_X.png"))
    fig_cnt=fig_cnt+1;

    fig(fig_cnt)=figure(fig_cnt);
    hold on
    %plot(data{1}.time,data{i+counter}.y(:,1)-data{i+counter}.y(:,3),'--b','LineWidth',1.0)
    h1=plot(data{1}.time,data{i+counter}.y(:,1),'b','LineWidth',2.0);
    h2=plot(data{1}.time(1:end-2),data{i+counter}.p(1:end-2,2),'--c','LineWidth',2.0);
    %plot(data{1}.time,data{i+counter}.y(:,1)+data{i+counter}.y(:,2),'--b','LineWidth',1.0)
    h3=patch([data{1}.time; flipud(data{1}.time)], [data{i+counter}.y(:,1)-data{i+counter}.y(:,3); flipud(data{i+counter}.y(:,1)+data{i+counter}.y(:,2))], 'c');;
    %plot(data{1}.time,data{i+counter+1}.y(:,1)-data{i+counter+1}.y(:,3),'--r','LineWidth',1.0)
    h4=plot(data{1}.time,data{i+counter+1}.y(:,1),'r','LineWidth',2.0);
    h5=plot(data{1}.time(1:end-2),data{i+counter+1}.p(1:end-2,2),'--','Color',[0.9290 0.6940 0.1250],'LineWidth',3.0);
    %plot(data{1}.time,data{i+counter+1}.y(:,1)+data{i+counter+1}.y(:,2),'--r','LineWidth',1.0)
    h6=patch([data{1}.time; flipud(data{1}.time)], [data{i+counter+1}.y(:,1)-data{i+counter+1}.y(:,3); flipud(data{i+counter+1}.y(:,1)+data{i+counter+1}.y(:,2))], 'r');
    alpha(0.2)
    plot(t(1),conditions{i}.start(2),'g*','LineWidth',3.0);
    plot(t(end),conditions{i}.end(2),'k*','LineWidth',3.0);
    grid on
    xlabel('time (sec)') 
    ylabel('Y (m)')
    hold off
%     lgd =legend([h3 h1 h2 h6 h4 h5],'Cartesian Zone', 'Mean Trajectory', 'CLF/CBF Response','Modiefied Cartesian Zone', 'Modified Mean Trajectory', 'CLF/CBF Response');
%     lgd.Location='northeast';
    f = gcf;
    exportgraphics(f,append(save_path,store_file,num2str(i),"\","Sc",num2str(i),"_Y.eps"),'Resolution',100)
    %saveas(fig(fig_cnt),append(save_path,store_file,num2str(i),"\","Sc",num2str(i),"_Y.png"))
    fig_cnt=fig_cnt+1;

    fig(fig_cnt)=figure(fig_cnt);
    hold on
    %plot(data{1}.time,data{i+counter}.z(:,1)-data{i+counter}.z(:,3),'--b','LineWidth',1.0)
    plot(data{1}.time,data{i+counter}.z(:,1),'b','LineWidth',2.0)
    plot(data{1}.time(1:end-2),data{i+counter}.p(1:end-2,3),'--c','LineWidth',2.0)
    %plot(data{1}.time,data{i+counter}.z(:,1)+data{i+counter}.z(:,2),'--b','LineWidth',1.0)
    patch([data{1}.time; flipud(data{1}.time)], [data{i+counter}.z(:,1)-data{i+counter}.z(:,3); flipud(data{i+counter}.z(:,1)+data{i+counter}.z(:,2))], 'c')
    %plot(data{1}.time,data{i+counter+1}.z(:,1)-data{i+counter+1}.z(:,3),'--r','LineWidth',1.0)
    plot(data{1}.time,data{i+counter+1}.z(:,1),'r','LineWidth',2.0)
    plot(data{1}.time(1:end-2),data{i+counter+1}.p(1:end-2,3),'--','Color',[0.9290 0.6940 0.1250],'LineWidth',3.0)
    %plot(data{1}.time,data{i+counter+1}.z(:,1)+data{i+counter+1}.z(:,2),'--r','LineWidth',1.0)
    patch([data{1}.time; flipud(data{1}.time)], [data{i+counter+1}.z(:,1)-data{i+counter+1}.z(:,3); flipud(data{i+counter+1}.z(:,1)+data{i+counter+1}.z(:,2))], 'r')
    alpha(0.2)
    plot(t(1),conditions{i}.start(3),'g*','LineWidth',3.0);
    plot(t(end),conditions{i}.end(3),'k*','LineWidth',3.0);
    grid on
    xlabel('time (sec)') 
    ylabel('Z (m)')
    hold off
    %lgd =legend([h1 h2 h3 h4],'Mean Trajectory','Modified Mean Trajectory','Starting Location', 'Goal Location');
    %lgd.Location='northeast';
    f = gcf;
    exportgraphics(f,append(save_path,store_file,num2str(i),"\","Sc",num2str(i),"_Z.eps"),'Resolution',100)
    %saveas(fig(fig_cnt),append(save_path,store_file,num2str(i),"\","Sc",num2str(i),"_Z.png"))
    fig_cnt=fig_cnt+1;



    counter=counter+1;
end
