close all;

filename{1} = 'data_FL.csv';
filename{2} = 'data_FR.csv';
filename{3} = 'data_RL.csv';
filename{4} = 'data_RR.csv';

for i = 1:1:4
    Arr_Leg{i} = table2array(readtable(filename{i}));
end
[m,n] = size(Arr_Leg{1});
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

    phase{i} = Arr_Leg{i}(:,13);
    event{i} = Arr_Leg{i}(:,14);
    touch{i} = Arr_Leg{i}(:,15);

    LO_state_indices = find(event{i} == 4);
    TD_state_indices = find(event{i} == 3);
    TD_state_indices = TD_state_indices(2:end); % First Data is no use

    r_des_TD{i} = Arr_Leg{i}(TD_state_indices,16);
    dr_des_TD{i} = Arr_Leg{i}(TD_state_indices,17);
    th_des_TD{i} = Arr_Leg{i}(TD_state_indices,18) + pi/2;
    dth_des_TD{i} = Arr_Leg{i}(TD_state_indices,19);
    t_flight_des{i} = Arr_Leg{i}(TD_state_indices,20);

    r_TD{i} = Arr_Leg{i}(TD_state_indices,21);
    dr_TD{i} = Arr_Leg{i}(TD_state_indices,22);
    th_TD{i} = Arr_Leg{i}(TD_state_indices,23);
    dth_TD{i} = Arr_Leg{i}(TD_state_indices,24);
    t_TD{i} = Arr_Leg{i}(TD_state_indices,25);

    r_LO{i} = Arr_Leg{i}(LO_state_indices,26);
    dr_LO{i} = Arr_Leg{i}(LO_state_indices,27);
    th_LO{i} = Arr_Leg{i}(LO_state_indices,28);
    dth_LO{i} = Arr_Leg{i}(LO_state_indices,29);
    t_LO{i} = Arr_Leg{i}(LO_state_indices,30);
    
%     [fl_m{i},fl_n{i}] = size(t_flight_des{i});
    
%     t_flight_des{i} = t_flight_des{i} + t_LO{i}(1:fl_n{i});

  

    
    

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


%%%Need to change total plot

figure(1)
for i = 1:1:4
    subplot(2,2,i);
    plot(t,r_des{i},'b-','LineWidth', lw);
    hold on
    plot(t,r_act{i},'r-','LineWidth',lw);
    grid on;
    legend('ref','act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    ylabel('r (m)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
    ylim([0 0.5]);
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
    plot(t,th_des{i}-th_act{i},'b-','LineWidth', lw);
    hold on
    plot(t,dth_des{i}-dth_act{i},'r-','LineWidth', lw); 
    grid on
    legend('th_error','dth_error','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    ylabel('$\tau$ (rad/s)','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
end
sgtitle('ERROR ','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');



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

figure(6)
for i = 1:1:4
    subplot(2,2,i)
    plot(t,phase{i},'b-','LineWidth', lw);
    hold on
    plot(t,event{i},'r-','LineWidth', lw); 
    grid on
    legend('phase','event','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    ylabel('status','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
end
sgtitle('Status','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

figure(7)
for i = 1:1:4
    subplot(2,2,i)
    plot(t,touch{i},'b-','LineWidth', lw);
    grid on;
    legend('touch','event','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
    ylabel('N','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
end
sgtitle('Touch Sensor','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');

% figure(8)
% for i = 1:1:4
%     subplot(2,2,i)
%     plot(t_TD{i},r_des_TD{i},'b-o','LineWidth', lw, 'MarkerSize', Ms);
%     hold on;
%     plot(t_TD{i},r_TD{i},'r-o','LineWidth', lw, 'MarkerSize', Ms);
%     grid on;
%     legend('Des','Act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
%     ylabel('m','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
% end
% sgtitle('Compare TD r','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');
% 
% figure(9)
% for i = 1:1:4
%     subplot(2,2,i)
%     plot(t_TD{i},dr_des_TD{i},'b-o','LineWidth', lw, 'MarkerSize', Ms);
%     hold on;
%     plot(t_TD{i},dr_TD{i},'r-o','LineWidth', lw, 'MarkerSize', Ms);
%     grid on;
%     legend('Des','Act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
%     ylabel('m/s','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
% end
% sgtitle('Compare TD dr','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');
% 
% figure(10)
% for i = 1:1:4
%     subplot(2,2,i)
%     plot(t_TD{i},th_des_TD{i},'b-o','LineWidth', lw, 'MarkerSize', Ms);
%     hold on;
%     plot(t_TD{i},th_TD{i},'r-o','LineWidth', lw, 'MarkerSize', Ms);
%     grid on;
%     legend('Des','Act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
%     ylabel('rad','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
% end
% sgtitle('Compare TD th','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');
% 
% figure(11)
% for i = 1:1:4
%     subplot(2,2,i)
%     plot(t_TD{i},dth_des_TD{i},'b-o','LineWidth', lw, 'MarkerSize', Ms);
%     hold on;
%     plot(t_TD{i},dth_TD{i},'r-o','LineWidth', lw, 'MarkerSize', Ms);
%     grid on;
%     legend('Des','Act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
%     ylabel('rad/s','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
% end
% sgtitle('Compare TD dth','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');
% 
% figure(12)
% for i = 1:1:4
%     subplot(2,2,i)
%     plot(t_flight_des{i},'b-o','LineWidth', lw, 'MarkerSize', Ms);
%     hold on;
%     plot(t_TD{i},'r-o','LineWidth', lw, 'MarkerSize', Ms);
%     grid on;
%     legend('Des','Act','FontName','Times New Roman','location','northeast','FontSize',fl,'Interpreter', 'latex')
%     ylabel('rad','FontName','Times New Roman','FontSize', Faxis,'Interpreter', 'latex'); % y축 레이블
% end
% sgtitle('Compare time','FontName','Times New Roman','FontSize',sgT,'Interpreter', 'latex');
% 


setFigurePositions(4);







