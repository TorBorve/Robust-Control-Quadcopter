close all; clear all; clc;
%% Calc settings
num_iter = 5;
Dscale_order = -1;
gammaRange = [0.9, 10];

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
K_inf = K_iteration_nominal(P, Ie, Iy, Iw, Iu, gammaRange);
fprintf("Plot Info about nominal controller\n");
% 
% load("K_lqr_controller.mat");
% muinfo = PlotInfoController(P, K_lqr_ss, Iz, Ie, Iv, Iw, RS_blk, RP_blk, omega, 10);
[muinfo, gammaRP_nom] = PlotInfoController(P, K_inf, Iz, Ie, Iv, Iw, RS_blk, RP_blk, omega, 0);
pause(0.1);
% error("Test");

K_inf_best = K_inf;
gammaRP_best = gammaRP_nom;

Dl = tf(eye(nz+ne));
Dr = tf(eye(nz+ne));
P_D = DscaleP(P, Dl, Dr, nz, ne, nmeas, nw, nv, nctrl);


for i=1:num_iter
    fprintf("\nDK-iteration %i\n", i);
    N_inf = lft(P, K_inf);
    fprintf("Choose D scales\n");
    [Dl_new, Dr_new] = chooseDscales(Dl, muinfo, omega, Dscale_order);
    % Dl = Dl_new * Dl;
    % Dr = Dr * Dr_new;
    Dl = Dl_new;
    Dr = Dr_new;
    fprintf("Find controller for D scaled plant\n");
    try
        K_inf = K_iteration(P_D, Dl, Dr, nz, ne, nmeas, nv, nw, nctrl, gammaRange);
    catch e
        fprintf("[Error]: %s\n", e.message);
        break;
    end
    P_D = DscaleP(P, Dl, Dr, nz, ne, nmeas, nw, nv, nctrl);

    fprintf("Plot Info about controller\n");
    [muinfo, gammaRP] = PlotInfoController(P_D, K_inf, Iz, Ie, Iv, Iw, RS_blk, RP_blk, omega, i);
    if gammaRP < gammaRP_best
        fprintf("Better controller found\n");
        gammaRP_best = gammaRP;
        K_inf_best = K_inf;
    end
    pause(0.1);
end

fprintf("Saving results to file\n");
saveControllerToFile(K_inf_best);
fprintf("Best gammaRP = %f\n", gammaRP_best);
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

function [muinfoRP, gammaRP] = plotSSV(N, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk, iter)
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
    gammaRP = worst_muRP;
end

function [Dl, Dr] = chooseDscales(Dl_prev, muInfo, omega, order)
    % Ddata = muInfo.dvec;
    % sens = muInfo.sens;
    % blk = muInfo.blk;
    % [Dl, Dr] = musynflp(Dl_prev, Ddata, sens, blk, nmeas, nctrl);
    % return;

    [Dl_w, Dr_w] = mussvunwrap(muInfo); % Extract D-scales
    % Dl_prev_w = frd(Dl_prev, omega);
    % Dl_w = Dl_w * Dl_prev_w;
    
    D_perf = Dl_w(size(Dl_w, 1), size(Dl_w, 2));
    D_1 = Dl_w(1, 1);
    D_2 = Dl_w(2, 2);
    D_3 = Dl_w(3, 3);

    D_1_fit = fitAndChooseDScale(D_1, omega, order);
    D_2_fit = fitAndChooseDScale(D_2, omega, order);
    D_3_fit = fitAndChooseDScale(D_3, omega, order);
    D_p_fit = fitAndChooseDScale(D_perf, omega, order);

    Dl = [D_1_fit, 0, 0, 0, 0, 0, 0;
          0, D_2_fit, 0, 0, 0, 0, 0;
          0, 0, D_3_fit, 0, 0, 0, 0;
          0, 0, 0, D_p_fit, 0, 0, 0;
          0, 0, 0, 0, D_p_fit, 0, 0;
          0, 0, 0, 0, 0, D_p_fit, 0;
          0, 0, 0, 0, 0, 0, D_p_fit];
    Dr = [1/D_1_fit, 0, 0, 0, 0, 0, 0;
          0, 1/D_2_fit, 0, 0, 0, 0, 0;
          0, 0, 1/D_3_fit, 0, 0, 0, 0;
          0, 0, 0, 1/D_p_fit, 0, 0, 0;
          0, 0, 0, 0, 1/D_p_fit, 0, 0;
          0, 0, 0, 0, 0, 1/D_p_fit, 0;
          0, 0, 0, 0, 0, 0, 1/D_p_fit];

    % Dl = [D_1_fit, 0, 0, 0, 0, 0;
    %       0, D_2_fit, 0, 0, 0, 0;
    %       0, 0, D_p_fit, 0, 0, 0;
    %       0, 0, 0, D_p_fit, 0, 0;
    %       0, 0, 0, 0, D_p_fit, 0;
    %       0, 0, 0, 0, 0, D_p_fit];
    % Dr = [1/D_1_fit, 0, 0, 0, 0, 0;
    %       0, 1/D_2_fit, 0, 0, 0, 0;
    %       0, 0, 1/D_p_fit, 0, 0, 0;
    %       0, 0, 0, 1/D_p_fit, 0, 0;
    %       0, 0, 0, 0, 1/D_p_fit, 0;
    %       0, 0, 0, 0, 0, 1/D_p_fit];

    % Dl = [D_1_fit, 0, 0, 0, 0, 0;
    %       0, D_2_fit, 0, 0, 0, 0;
    %       zeros(4, 2), eye(4)];
    % Dr = [1/D_1_fit, 0, 0, 0, 0, 0;
    %       0, 1/D_2_fit, 0, 0, 0, 0;
    %       zeros(4, 2), eye(4)];
    % Dl = [D_1_fit, 0, 0, 0, 0, 0, 0;
    %       0, D_2_fit, 0, 0, 0, 0, 0;
    %       0, 0, D_3_fit, 0, 0, 0, 0;
    %       zeros(4, 3), eye(4)];
    % Dr = [1/D_1_fit, 0, 0, 0, 0, 0, 0;
    %       0, 1/D_2_fit, 0, 0, 0, 0, 0;
    %       0, 0, 1/D_3_fit, 0, 0, 0, 0;
    %       zeros(4, 3), eye(4)];
end

function D_i = fitAndChooseDScale(D_frd, omega, order)
    D_a = fitfrd(genphase(D_frd), 0);
    D_b = fitfrd(genphase(D_frd), 1);
    D_c = fitfrd(genphase(D_frd), 2);
    D_d = fitfrd(genphase(D_frd), 3);
    
    if order == -1
        fig_val = figure;
        hold on;
        plot(D_frd);
        plot(abs(frd(D_a, omega)));
        plot(abs(frd(D_b, omega)));
        plot(abs(frd(D_c, omega)));
        plot(abs(frd(D_d, omega)));
        legend(["Exact", "0", "1", "2", "3"]);
        xscale log;
        yscale log;
        
        figure(fig_val);
    
        choosenOrder = input("Choose the order of the D scale. i.e 0, 1, 2, 3: ");
        close(fig_val);
    else
        choosenOrder = order;
    end
    % close(fig1);
    
    if choosenOrder == 0
        fprintf("Order 0 chosen\n")
        D_i = D_a;
    elseif choosenOrder == 1
        fprintf("Order 1 chosen\n")
        D_i = D_b;
    elseif choosenOrder == 2
        fprintf("Order 2 chosen\n")
        D_i = D_c;
    elseif choosenOrder == 3
        fprintf("Order 3 chosen\n")
        D_i = D_d;
    elseif choosenOrder > 3
        D_i = fitfrd(genphase(D_frd), choosenOrder);
        fprintf("Order %i chosen\n", choosenOrder);
    else
        error("invalid order chosen for D scales");
    end

end

function P_D = DscaleP(P, Dl, Dr, nz, ne, nmeas, nw, nv, nctrl)
    P_D = [Dl, zeros(nz+ne, nmeas);
                  zeros(nmeas, nz+ne), eye(nmeas)] ...
                  * P * ...
                 [Dr, zeros(nv+nw, nctrl);
                 zeros(nctrl, nv+nw), eye(nctrl)];
end

function K_inf = K_iteration_nominal(P, Ie, Iy, Iw, Iu, gammaRange)
    nmeas = size(Iy, 1);
    nctrl = size(Iu, 1);
    
    
    P_nom = P([Ie;Iy], [Iw;Iu]);
    P_nom = P;
    options = hinfsynOptions("Display", "on", "Method", "RIC");
    [K_inf, N_inf_unused, gamma, info] = hinfsyn(P_nom, nmeas, nctrl, options); %, ...
                                          % "METHOD", "ric", ...
                                          % "TOLGAM", 0.1); 
    if isnan(gamma)
        msg = sprintf("Failed to find H_inf controller\n");
        error(msg);
    end    
end

function K_inf = K_iteration(P, Dl, Dr, nz, ne, nmeas, nv, nw, nctrl, gammaRange)
    % Pmu1design = [Dl, zeros(nz+ne, nmeas);
    %               zeros(nmeas, nz+ne), eye(nmeas)] ...
    %               * P * ...
    %              [Dr, zeros(nv+nw, nctrl);
    %              zeros(nctrl, nv+nw), eye(nctrl)];
    Pmu1design = DscaleP(P, Dl, Dr, nz, ne, nmeas, nw, nv, nctrl);
    options = hinfsynOptions("Display", "on", "Method", "RIC");
    [K_inf, N_inf_unused, gamma, info] = hinfsyn(Pmu1design, nmeas, nctrl, options); %, ...
                                          % "METHOD", "lmi", ...
                                          % "TOLGAM", 0.1);
    if isinf(gamma) || isnan(gamma)
        msg = sprintf("Failed to find H_inf controller\n");
        error(msg);
    end
end

function [muinfoRP, gammaRP] = PlotInfoController(P, K, Iz, Ie, Iv, Iw, RS_blk, RP_blk, omega, iter)
    N_inf = lft(P, K);

    [muinfoRP, gammaRP] = plotSSV(N_inf, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk, iter);
end
