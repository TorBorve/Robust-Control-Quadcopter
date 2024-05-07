close all; clear all; clc;
%% Calc settings
num_iter = 2;

%% Init model and Weights
init_robust_simulink();


%% Plot Weights
figure("Name", "Weights");
hold on;
addWeightToPlot(W_perf_e, omega, "W_perf_e");
addWeightToPlot(W_perf_p, omega, "W_perf_p");
addWeightToPlot(W_perf_uf, omega, "W_perf_uf");
addWeightToPlot(W_perf_ub, omega, "W_perf_ub");
addWeightToPlot(W_act, omega, "W_act");
addWeightToPlot(W_noise_e, omega, "W_noise_e");
addWeightToPlot(W_noise_p, omega, "W_noise_p");
addWeightToPlot(W_ref_e, omega, "W_ref_e");
addWeightToPlot(W_ref_p, omega, "W_ref_p");

yscale log;
xscale log;
grid on;
legend();
pause(0.1);

%% Model
% Linear model
[A_P, B_P, C_P, D_P] = linmod("robust_model");
P = ss(A_P, B_P, C_P, D_P);

nmeas = size(Iy, 1);
nctrl = size(Iu, 1);

nz = size(Iz, 1);
ne = size(Ie, 1);
nv = size(Iv, 1);
nw = size(Iw, 1);

%% Start DK-iteration
fprintf("Find nominal H_inf controller\n");
K_inf = K_iteration_nominal(P, Ie, Iy, Iw, Iu);
fprintf("Plot Info about nominal controller\n");
% 
% load("K_lqr_controller.mat");
% muinfo = PlotInfoController(P, K_lqr_ss, Iz, Ie, Iv, Iw, RS_blk, RP_blk, omega, 10);
muinfo = PlotInfoController(P, K_inf, Iz, Ie, Iv, Iw, RS_blk, RP_blk, omega, 0);
pause(0.1);
% error("Test");

Dl = tf(eye(6));
Dr = tf(eye(6));


for i=1:num_iter
    fprintf("\nDK-iteration %i\n", i);
    N_inf = lft(P, K_inf);
    fprintf("Choose D scales\n");
    [Dl_new, Dr_new] = chooseDscales(muinfo, N_inf, RP_blk, omega, Dl, nmeas, nctrl);
    % Dl = Dl_new * Dl;
    % Dr = Dr * Dr_new;
    Dl = Dl_new;
    Dr = Dr_new;
    fprintf("Find controller for D scaled plant\n");
    try
        K_inf = K_iteration(P, Dl, Dr, nz, ne, nmeas, nv, nw, nctrl);
    catch e
        fprintf("[Error]: %s\n", e.message);
        break;
    end

    fprintf("Plot Info about controller\n");
    muinfo = PlotInfoController(P, K_inf, Iz, Ie, Iv, Iw, RS_blk, RP_blk, omega, i);
    pause(0.1);
end

fprintf("Saving results to file\n");
saveControllerToFile(K_inf);
fprintf("Done\n");

function saveControllerToFile(K_inf)
    fname = mfilename;
    fpath = mfilename('fullpath');
    dpath = strrep(fpath, fname, '');
    dpath = strcat(dpath, "/generated");
    [status, msg, msgID] = mkdir(dpath);
    save(strcat(dpath, "/K_inf_controller.mat"), "K_inf");
end

function addWeightToPlot(weight, freqs, name)
    H = squeeze(freqresp(weight, freqs));
    if name ~= ""
        plot(freqs, abs(H), "DisplayName", name);
    else
        plot(freqs, abs(H));
    end
end

function muinfoRP = plotSSV(N, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk, iter)
    N_w = frd(N, omega);
    muRS = mussv(N_w(Iz, Iv), RS_blk);
    muNP = svd(N_w(Ie, Iw));
    [muRP, muinfoRP] = mussv(N_w, RP_blk);
    
    u_muRS = muRS(1, 1, :);
    u_muNP = muNP(1, :, :);
    u_muRP = muRP(1, 1, :);
    
    figure("Name", sprintf("Robustness iter: %i", iter));
    hold on;
    grid on;
    plot(u_muRS);
    plot(u_muNP);
    plot(u_muRP);
    legend(["RS", "NP", "RP"]);
    xscale log;

    worst_muRP = max(u_muRP.ResponseData(1, 1, :));
    fprintf("Worst muRP upper bound: %f, iter: %i\n", worst_muRP, iter);
end

function [Dl, Dr] = chooseDscales(muInfo, N_inf, RP_blk, omega, Dl_prev, nmeas, nctrl)
    % Ddata = muInfo.dvec;
    % sens = muInfo.sens;
    % blk = muInfo.blk;
    % [Dl, Dr] = musynflp(Dl_prev, Ddata, sens, blk, nmeas, nctrl);
    % return;

    [Dl0, Dr0] = mussvunwrap(muInfo); % Extract D-scales
    
    D0_perf = Dl0(3, 3);
    D0_1 = Dl0(1, 1)/D0_perf;
    D0_2 = Dl0(2, 2)/D0_perf;
    
    D0_1a = fitfrd(genphase(D0_1), 0);
    D0_1b = fitfrd(genphase(D0_1), 1);
    D0_1c = fitfrd(genphase(D0_1), 2);
    D0_1d = fitfrd(genphase(D0_1), 3);

    D0_2a = fitfrd(genphase(D0_2), 0);
    D0_2b = fitfrd(genphase(D0_2), 1);
    D0_2c = fitfrd(genphase(D0_2), 2);
    D0_2d = fitfrd(genphase(D0_2), 3);
    
    fig_val = figure;
    hold on;
    plot(D0_1);
    plot(abs(frd(D0_1a, omega)));
    plot(abs(frd(D0_1b, omega)));
    plot(abs(frd(D0_1c, omega)));
    plot(abs(frd(D0_1d, omega)));
    legend(["D0_1", "0", "1", "2", "3"]);
    % plot(D0_2);
    % plot(D0_perf);
    xscale log;
    yscale log;
    
    D_scale_base = [D0_1, zeros(1, 5);
                0, D0_2, zeros(1, 4);
                zeros(4, 2), eye(4)];
    
    D_scale_inv_base = [1/D0_1, zeros(1, 5);
                     0, 1/D0_2, zeros(1, 4);
                     zeros(4, 2), eye(4)];

    
    D_scale_a = D_scale_base;
    D_scale_a(1, 1) = D0_1a;
    D_scale_inv_a = D_scale_inv_base;
    D_scale_inv_a(1, 1) = 1/D0_1a;
    N_D_a = D_scale_a * N_inf * D_scale_inv_a;

    D_scale_b = D_scale_base;
    D_scale_b(1, 1) = D0_1b;
    D_scale_inv_b = D_scale_inv_base;
    D_scale_inv_b(1, 1) = 1/D0_1b;
    N_D_b = D_scale_b * N_inf * D_scale_inv_b;

    D_scale_c = D_scale_base;
    D_scale_c(1, 1) = D0_1c;
    D_scale_inv_c = D_scale_inv_base;
    D_scale_inv_c(1, 1) = 1/D0_1c;
    N_D_c = D_scale_c * N_inf * D_scale_inv_c;

    D_scale_d = D_scale_base;
    D_scale_d(1, 1) = D0_1d;
    D_scale_inv_d = D_scale_inv_base;
    D_scale_inv_d(1, 1) = 1/D0_1d;
    N_D_d = D_scale_d * N_inf * D_scale_inv_d;

    N_inf_w = frd(N_inf, omega);
    % muRP = mussv(N_inf_w, RP_blk);
    % 
    % % N_D_a_w = frd(N_D_a, omega);
    % muRPa = mussv(N_D_a, RP_blk);
    % 
    % % N_D_b_w = frd(N_D_b, omega);
    % muRPb = mussv(N_D_b, RP_blk);
    % 
    % % N_D_c_w = frd(N_D_c, omega);
    % muRPc = mussv(N_D_c, RP_blk);
    % 
    % % N_D_d_w = frd(N_D_d, omega);
    % muRPd = mussv(N_D_d, RP_blk);
    % 
    % fig1 = figure();
    % hold on;
    % plot(muRP(1, 1, :));
    % plot(muRPa(1, 1, :));
    % plot(muRPb(1, 1, :));
    % plot(muRPc(1, 1, :));
    % plot(muRPd(1, 1, :));
    % legend(["Exact", "0", "1", "2", "3"]);
    % xscale log;

    % figure(fig1);
    figure(fig_val);
    choosenOrder = input("Choose the order of the D scale. i.e 0, 1, 2, 3: ");
    % close(fig1);
    close(fig_val);
    if choosenOrder == 0
        fprintf("Order 0 chosen\n")
        D1_chosed = D0_1a;
        D_scale_base = D_scale_a;
        D_scale_inv_base = D_scale_inv_a;
    elseif choosenOrder == 1
        fprintf("Order 1 chosen\n")
        D1_chosed = D0_1b;
        D_scale_base = D_scale_b;
        D_scale_inv_base = D_scale_inv_b;
    elseif choosenOrder == 2
        fprintf("Order 2 chosen\n")
        D1_chosed = D0_1c;
        D_scale_base = D_scale_c;
        D_scale_inv_base = D_scale_inv_c;
    elseif choosenOrder == 3
        fprintf("Order 3 chosen\n")
        D1_chosed = D0_1d;
        D_scale_base = D_scale_d;
        D_scale_inv_base = D_scale_inv_d;
    else
        error("invalid order chosen for D scales");
    end
    
    D_scale_a = D_scale_base;
    D_scale_a(2, 2) = D0_2a;
    D_scale_inv_a = D_scale_inv_base;
    D_scale_inv_a(2, 2) = 1/D0_2a;
    N_D_a = D_scale_a * N_inf * D_scale_inv_a;

    D_scale_b = D_scale_base;
    D_scale_b(2, 2) = D0_2b;
    D_scale_inv_b = D_scale_inv_base;
    D_scale_inv_b(2, 2) = 1/D0_2b;
    N_D_b = D_scale_b * N_inf * D_scale_inv_b;

    D_scale_c = D_scale_base;
    D_scale_c(2, 2) = D0_2c;
    D_scale_inv_c = D_scale_inv_base;
    D_scale_inv_c(2, 2) = 1/D0_2c;
    N_D_c = D_scale_c * N_inf * D_scale_inv_c;

    D_scale_d = D_scale_base;
    D_scale_d(2, 2) = D0_2d;
    D_scale_inv_d = D_scale_inv_base;
    D_scale_inv_d(2, 2) = 1/D0_2d;
    N_D_d = D_scale_d * N_inf * D_scale_inv_d;

    N_inf_w = frd(N_inf, omega);
    % muRP = mussv(N_inf_w, RP_blk);
    % 
    % % N_D_a_w = frd(N_D_a, omega);
    % muRPa = mussv(N_D_a, RP_blk);
    % 
    % % N_D_b_w = frd(N_D_b, omega);
    % muRPb = mussv(N_D_b, RP_blk);
    % 
    % % N_D_c_w = frd(N_D_c, omega);
    % muRPc = mussv(N_D_c, RP_blk);
    % 
    % % N_D_d_w = frd(N_D_d, omega);
    % muRPd = mussv(N_D_d, RP_blk);
    % 
    % fig2 = figure;
    % hold on;
    % plot(muRP(1, 1, :));
    % plot(muRPa(1, 1, :));
    % plot(muRPb(1, 1, :));
    % plot(muRPc(1, 1, :));
    % plot(muRPd(1, 1, :));
    % legend(["Exact", "0", "1", "2", "3"]);
    % xscale log;

    fig2_val = figure;
    hold on;
    plot(D0_2);
    plot(abs(frd(D0_2a, omega)));
    plot(abs(frd(D0_2b, omega)));
    plot(abs(frd(D0_2c, omega)));
    plot(abs(frd(D0_2d, omega)));
    legend(["D0_2", "0", "1", "2", "3"]);
    % plot(D0_2);
    % plot(D0_perf);
    xscale log;
    yscale log;

    % figure(fig2);
    figure(fig2_val);
    choosenOrder = input("Choose the order of the D scale. i.e 0, 1, 2, 3: ");
    % close(fig2);
    close(fig2_val);
    if choosenOrder == 0
        fprintf("Order 0 chosen\n")
        D2_chosed = D0_2a;
        D_scale_base(2, 2) = D_scale_a(2, 2);
        D_scale_inv_base(2, 2) = D_scale_inv_a(2, 2);
    elseif choosenOrder == 1
        fprintf("Order 1 chosen\n")
        D2_chosed = D0_2b;
        D_scale_base(2, 2) = D_scale_b(2, 2);
        D_scale_inv_base(2, 2) = D_scale_inv_b(2, 2);
    elseif choosenOrder == 2
        fprintf("Order 2 chosen\n")
        D2_chosed = D0_2c;
        D_scale_base(2, 2) = D_scale_c(2, 2);
        D_scale_inv_base(2, 2) = D_scale_inv_c(2, 2);
    elseif choosenOrder == 3
        fprintf("Order 3 chosen\n")
        D2_chosed = D0_2d;
        D_scale_base(2, 2) = D_scale_d(2, 2);
        D_scale_inv_base(2, 2) = D_scale_inv_d(2, 2);
    else
        error("invalid order chosen for D scales");
    end

    
    Dl = [D1_chosed, zeros(1, 5);
          0, D2_chosed, zeros(1, 4);
          zeros(4, 2), eye(4);];
    Dr = [1/D1_chosed, zeros(1, 5);
          0, 1/D2_chosed, zeros(1, 4);
          zeros(4, 2), eye(4);];
end

function K_inf = K_iteration_nominal(P, Ie, Iy, Iw, Iu)
    nmeas = size(Iy, 1);
    nctrl = size(Iu, 1);
    
    
    P_nom = P([Ie;Iy], [Iw;Iu]);
    P_nom = P;
    [K_inf, N_inf_unused, gamma, info] = hinfsyn(P_nom, nmeas, nctrl); %, ...
                                          % "METHOD", "ric", ...
                                          % "TOLGAM", 0.1); 
    if isnan(gamma)
        msg = sprintf("Failed to find H_inf controller\n");
        error(msg);
    end    
end

function K_inf = K_iteration(P, Dl, Dr, nz, ne, nmeas, nv, nw, nctrl)
    Pmu1design = [Dl, zeros(nz+ne, nmeas);
                  zeros(nmeas, nz+ne), eye(nmeas)] ...
                  * P * ...
                 [Dr, zeros(nv+nw, nctrl);
                 zeros(nctrl, nv+nw), eye(nctrl)];
    
    [K_inf, N_inf_unused, gamma, info] = hinfsyn(Pmu1design, nmeas, nctrl); %, ...
                                          % "METHOD", "lmi", ...
                                          % "TOLGAM", 0.1);
    if isinf(gamma) || isnan(gamma)
        msg = sprintf("Failed to find H_inf controller\n");
        error(msg);
    end
end

function muinfoRP = PlotInfoController(P, K, Iz, Ie, Iv, Iw, RS_blk, RP_blk, omega, iter)
    K = K; % Used in simulink
    % [A_Pc, B_Pc, C_Pc, D_Pc] = linmod("robust_model_closed_loop");
    % Pc = ss(A_Pc, B_Pc, C_Pc, D_Pc);
    % 
    % Icz = (1:2)';
    % Icw = (3:6)';
    % 
    % Icv = (1:2)';
    % Ice = (3:4)';
    % Icy = (5:6)';
    % 
    % y_r = Pc(Icy(2), Icw(2));
    % y_r = minreal(y_r);
    % figure;
    % step(y_r);
    % 
    % u_r = Pc(Ice(2), Icw(2));
    % u_r = minreal(u_r);
    % figure;
    % step(u_r);
    
    % Singular vlues
    N_inf = lft(P, K);

    muinfoRP = plotSSV(N_inf, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk, iter);
end
