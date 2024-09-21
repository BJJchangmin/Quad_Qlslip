
T = 2;

r_LO = 0.4;
th_LO = -0.1;
dr_LO = -0.0524;
dth_LO = -1.2134;

des_r_TD = 0.4;
des_th_TD = 0.3;
des_dr_TD = -0.3;
des_dth_TD = -1;

r_des_top = 0.3;
th_des_top = 0;

%position
% [r*sin(th), r*cos(th)]

%Velocity
% [dr*sin(th) + r*cos(th)*dth , dr*cos(th)-r*sin(th)*dth]
%% 2-order bezier curve
p0 = [r_LO*sin(th_LO), r_LO*cos(th_LO) ];
p2 = [des_r_TD*sin(des_th_TD), des_r_TD*cos(des_th_TD)];
p1 = 2*([r_des_top*sin(th_des_top), r_des_top*cos(th_des_top)] - 0.25*p0 - 0.25*p2);

% t 값 범위 (0에서 1까지, 곡선 생성)
t = linspace(0, 1, 100);

% 2차 베지어 곡선을 각 좌표(x와 y)별로 나눠서 계산
% x 좌표 계산 (2차)
bx_2 = (1-t).^2 * p0(1) + 2*(1-t) .* t * p1(1) + t.^2 * p2(1);
% y 좌표 계산 (2차)
by_2 = (1-t).^2 * p0(2) + 2*(1-t) .* t * p1(2) + t.^2 * p2(2);

b_2 = [bx_2; by_2];

r = sqrt(bx_2.^2+by_2.^2);
th_r = atan(bx_2./by_2);

% 그래프 그리기
figure;
hold on;

% 2차 베지어 곡선 그리기
plot(b_2(1, :), b_2(2, :), 'g-', 'LineWidth', 2); % 2차 곡선
plot([p0(1), p1(1), p2(1)], [p0(2), p1(2), p2(2)], 'go--'); % 2차 제어점


%% 4 order bezier curve

p0 = [r_LO*sin(th_LO), r_LO*cos(th_LO) ];
p1 = 0.25 *([dr_LO*sin(th_LO) + r_LO*cos(th_LO)*dth_LO , dr_LO*cos(th_LO)-r_LO*sin(th_LO)*dth_LO]-4*p0);
p4 = [des_r_TD*sin(des_th_TD), des_r_TD*cos(des_th_TD)];
p3 = (4*p4 - [des_dr_TD*sin(des_th_TD) + des_r_TD*cos(des_th_TD)*des_dth_TD , ...
    des_dr_TD*cos(des_th_TD)-des_r_TD*sin(des_th_TD)*des_dth_TD])/(4);
p2 = ([r_des_top*sin(th_des_top), r_des_top*cos(th_des_top)] - 1/16*p0 - 4/16*p1 - ...
    4/16*p3 - 1/16*p4)*16/6;

% t 값 범위 (0에서 1까지, 곡선 생성)
t = linspace(0, 1, 100);

% 베지어 곡선을 각 좌표(x와 y)별로 나눠서 계산
% x 좌표 계산
bx_4 = (1-t).^4 * p0(1) + 4*(1-t).^3 .* t * p1(1) + 6*(1-t).^2 .* t.^2 * p2(1) + 4*(1-t) .* t.^3 * p3(1) + t.^4 * p4(1);

% y 좌표 계산
by_4 = (1-t).^4 * p0(2) + 4*(1-t).^3 .* t * p1(2) + 6*(1-t).^2 .* t.^2 * p2(2) + 4*(1-t) .* t.^3 * p3(2) + t.^4 * p4(2);

% x와 y 좌표를 합쳐서 곡선을 완성
b_4 = [bx_4; by_4];


% 그래프 그리기
figure;
hold on;
% 4차 베지어 곡선
plot(b_4(:,1), b_4(:,2)-0.4, 'b-', 'LineWidth', 2); % 곡선
plot([p0(1), p1(1), p2(1), p3(1), p4(1)], [p0(2), p1(2), p2(2), p3(2), p4(2)], 'bo--'); % 제어점