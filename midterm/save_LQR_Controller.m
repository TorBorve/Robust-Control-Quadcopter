init_simulink();

[A, B, C, D] = linmod("LQR_controller");
K_lqr_ss = ss(A, B, C, D);

fname = mfilename;
fpath = mfilename('fullpath');
dpath = strrep(fpath, fname, '');
dpath = strcat(dpath, "/generated");
[status, msg, msgID] = mkdir(dpath);
save(strcat(dpath, "/K_lqr_controller.mat"), "K_lqr_ss");