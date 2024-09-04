
des_r = 0.4;
des_thr = 80*pi/180;
l = 0.25;

% th2 = acos(0.4/(2*0.25))*2;

th_b = des_thr+acos(des_r/(2*l));
th_m = 2*des_thr - th_b;

th1 = th_m;
th2 = th_b-th_m;

disp(th_b);
disp(th_m);
disp(th1);
disp(th2);