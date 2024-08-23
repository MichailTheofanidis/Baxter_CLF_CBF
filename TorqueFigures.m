%% Plot the response of the ProDMP CBF/CLF Controller
%% Michail Theofanidis



%% Load the data
date = "8_31_23";
data1 = importdata(append("Data\",date,"\results1.mat"));
data2 = importdata(append("Data\",date,"\results2.mat"));

t=data1{1}.time;

accel_1=zeros(length(t),2);
accel_2=zeros(length(t),2);
accel_3=zeros(length(t),2);
accel_4=zeros(length(t),2);

for j=1:length(t)
    accel_1(j,:)=[norm(data1{1}.pu(j,:)),norm(data2{1}.pu(j,:))];
    accel_2(j,:)=[norm(data1{2}.pu(j,:)),norm(data2{2}.pu(j,:))];
    accel_3(j,:)=[norm(data1{3}.pu(j,:)),norm(data2{3}.pu(j,:))];
    accel_4(j,:)=[norm(data1{4}.pu(j,:)),norm(data2{4}.pu(j,:))];
end

figure(1)
plot(t,accel_1(:,1),"--c",'LineWidth',2.0)
hold on
plot(t,accel_1(:,2),"b",'LineWidth',2.0)
legend("CBF priority","CLF priority")
hold off
sum_1=cumsum(accel_1(:,1));
sum_2=cumsum(accel_1(:,2));

sum_1(end)
sum_2(end)

set(0, 'defaultTextInterpreter', 'latex');
figure(2)
plot(t,accel_2(:,1),'--','Color',[0.9290 0.6940 0.1250],'LineWidth',2.0)
hold on
plot(t,accel_2(:,2),"r",'LineWidth',2.0)
xlabel('time (sec)') 
ylabel('control effort $$(m^{2}/s)$$')
legend("CBF priority","CLF priority")
hold off
f = gcf;
exportgraphics(f,append(save_path,"Energy.eps"),'Resolution',200)

sum_1=cumsum(accel_2(:,1));
sum_2=cumsum(accel_2(:,2));

sum_1(end)
sum_2(end)

figure(3)
plot(t,accel_3(:,1),"--c",'LineWidth',2.0)
hold on
plot(t,accel_3(:,2),"b",'LineWidth',2.0)
legend("CBF priority","CLF priority")
hold off

sum_1=cumsum(accel_3(:,1));
sum_2=cumsum(accel_3(:,2));

sum_1(end)
sum_2(end)

figure(4)
plot(t,accel_4(:,1),"--y",'LineWidth',2.0)
hold on
plot(t,accel_4(:,2),"r",'LineWidth',2.0)
legend("CBF priority","CLF priority")
hold off

sum_1=cumsum(accel_4(:,1));
sum_2=cumsum(accel_4(:,2));

sum_1(end)
sum_2(end)
