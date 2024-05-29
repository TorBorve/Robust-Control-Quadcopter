%% LQR shown in midterm
Q = diag([1e2, 1e2, 1e3, 1e1, 1e3, 1e2, 1e0, 1e0]);
R = diag([1e0, 1e0]);
r_observer = 15;
init_simulink();

[A, B, C, D] = linmod("LQR_controller");
K_lqr_ss = ss(A, B, C, D);

fname = mfilename;
fpath = mfilename('fullpath');
dpath = strrep(fpath, fname, '');
dpath = strcat(dpath, "/generated");
[status, msg, msgID] = mkdir(dpath);
save(strcat(dpath, "/K_lqr_controller.mat"), "K_lqr_ss");

%% Retuned LQR shown in final report
clear all;
% Q = diag([1e2, 1e3, 1e3, 1e2, 1e2, 1e3, 1e2, 1e2]);
% R = diag([1e2, 1e2]);

Q = diag([1e2, 1e3, 1e3, 1e2, 1e2, 1e3, 1e1, 1e1]);
R = diag([3e1, 3e1]);
% Q = diag([1e3, 1e3, 1e3, 1e2, 1e4, 1e3, 1e1, 1e1]);
% R = diag([1e1, 1e1]);
% % Q = diag([1e0, 1e0, 1e2, 1e1, 1e3, 1e3, 1e2, 1e2]);
% % R = diag([1e3, 1e3]);
r_observer = 15;
init_simulink();
[A, B, C, D] = linmod("LQR_controller");

% LQR_V2();
% [A, B, C, D] = linmod("LQR_controller_V2");
K_lqr_ss_retuned = ss(A, B, C, D);

fname = mfilename;
fpath = mfilename('fullpath');
dpath = strrep(fpath, fname, '');
dpath = strcat(dpath, "/generated");
save(strcat(dpath, "/K_lqr_controller_retuned.mat"), "K_lqr_ss_retuned");