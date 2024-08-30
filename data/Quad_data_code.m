close all;


filename{1} = 'data_FL.csv';
filename{2} = 'data_FR.csv';
filename{3} = 'data_RL.csv';
filename{4} = 'data_RR.csv';

for i = 1:1:4
    Arr_Leg{i} = table2array(readtable(filename{i}));
end

t = Arr_Leg{1}(:,1);

for i = 1:1:4

    r_des{i} = Arr_Leg{i}(:,2);
    r_act{i} = Arr_Leg{i}(:,3);
    th_des{i} = Arr_Leg{i}(:,4);
    th_act{i} = Arr_Leg{i}(:,5);

    dr_des{i} = Arr_Leg{i}(:,6);
    dr_act{i} = Arr_Leg{i}(:,7);
    dth_des{i} = Arr_Leg{i}(:,8);
    dth_act{i} = Arr_Leg{i}(:,9);

    tau_HAA{i} = Arr_Leg{i}(:,10);
    tau_HFE{i} = Arr_Leg{i}(:,11);
    tau_KFE{i} = Arr_Leg{i}(:,12);

end


Ts = t(2,1)-t(1,1);
for i = 1:length(r_des)
    t(i,1) = (i-1)*Ts;
end


 %%%%DATA PLOT %%%%%%%%%%%%%%%%%%%

%Plotting Parameter
lw =1;   %Line Width
FT = 7; %Title Fonte Size
sgT= 18; % subtitle plot title
Faxis = 12.5; %Axis Fonte Size
fl =10 ; % Legend Fonte Size
Ms = 3 ; %Mark Size


%Need to change total plot
figure(1)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,r_des{i},'b-','LineWidth', lw);
    hold on
    plot(t,r_act{i},'r-','LineWidth',lw);
    grid on;
    legend('ref','act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    ylabel('r (m)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
end
sgtitle('r tracking ','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(2)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,dr_des{i},'b-','LineWidth', lw);
    hold on
    plot(t,dr_act{i},'r-','LineWidth',lw);
    grid on;
    legend('ref','act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    ylabel('$\dot{r}$ (m/s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
end
sgtitle('$\dot{r}$ tracking ','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(3)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,th_des{i},'b-','LineWidth', lw);
    hold on
    plot(t,th_act{i},'r-','LineWidth',lw);
    grid on;
    legend('ref','act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    ylabel('$\theta$ (rad)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
end
sgtitle('$\theta$ tracking ','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(4)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,dth_des{i},'b-','LineWidth', lw);
    hold on
    plot(t,dth_act{i},'r-','LineWidth',lw);
    grid on;
    legend('ref','act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    ylabel('$\dot{\theta}$ (rad/s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
end
sgtitle('$\dot{\theta}$ tracking ','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(5)
for i = 1:1:4
    subplot(2,2,i)
    plot(t,tau_HAA{i},'b-','LineWidth', lw);
    hold on
    plot(t,tau_HFE{i},'r-','LineWidth', lw);
    hold on
    plot(t,tau_KFE{i},'g-','LineWidth', lw);
    grid on
    legend('HAA','HFE','KFE','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    ylabel('$\tau$ (rad/s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
end
sgtitle('Actual Torque ','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');









