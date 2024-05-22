%% Calc settings
clear all; close all; clc;
num_iter = 10;
Dscale_orders_seq = [1, 3, 2, 1;
                 1, 1, 2, 1];

% Dscale_orders_seq = [];
% for i1=0:4
%     for i2=0:4
%         for i3=0:4
%             for i4=0:4
%                 Dscale_orders_seq = [Dscale_orders_seq;
%                                      i4, i3, i2, i1];
%             end
%         end
%     end
% end
% external_monitor = 1;
% 
% std_dims = [50 800 600 500];
% if external_monitor == 1
%     std_dims(2) = 1200;
% end

%% Init model and Weights
init_robust_simulink();


%% Plot Weights
fig = figure("Name", "Weights");
weight_plot(W_perf_e, W_perf_p, W_perf_uf, W_act, W_noise_e, W_noise_p, W_ref_e, W_ref_p, omega)
% set(fig, "renderer", "painters", "position", std_dims, "PaperPositionMode", "auto");

% exportgraphics(fig,"./figures/perf_weight.pdf",'ContentType','vector');
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

[muinfo, gammaRP_nom] = calculate_and_plot_ssv(P, K_inf, Iz, Ie, Iv, Iw, RS_blk, RP_blk, omega, 0);
pause(0.1);

K_inf_iters = {K_inf};
K_inf_best = K_inf;
gammaRP_best = gammaRP_nom;

% if exist("Dscale_orders_seq", "var")
%     num_iter = size(Dscale_orders_seq, 1);
% end
js = [];

for i=1:num_iter
    fprintf("\nDK-iteration %i\n", i);
    N_inf = lft(P, K_inf);
    fprintf("Choose D scales\n");
    if exist("Dscale_orders_seq", "var") && size(Dscale_orders_seq, 2) >= i
        Dscale_orders = Dscale_orders_seq(i, :);
    else
        Dscale_orders = [-1, -1, -1, -1];
    end
    [Dl, Dr] = chooseDscales(muinfo, omega, Dscale_orders);
    P_D = DscaleP(P, Dl, Dr, nz, ne, nmeas, nw, nv, nctrl);
    fprintf("Find controller for D scaled plant\n");

    try
        K_inf = K_iteration(P_D, nmeas, nctrl);
    catch e
        fprintf("[Error]: %s\n", e.message);
        continue;
        % break;
    end

    fprintf("Plot Info about controller\n");
    [muinfo, gammaRP] = calculate_and_plot_ssv(P, K_inf, Iz, Ie, Iv, Iw, RS_blk, RP_blk, omega, i);
    
    if gammaRP < gammaRP_best
        fprintf("Better controller found\n");
        K_inf_iters = [K_inf_iters ; {K_inf}];
        gammaRP_best = gammaRP;
        K_inf_best = K_inf;
    end
    K_inf = K_inf_best;
    pause(0.1);
end

fprintf("Saving results to file\n");
saveControllerToFile(K_inf_best, K_inf_iters);
fprintf("Best gammaRP = %f\n", gammaRP_best);
fprintf("Done\n");

%% Function Definitions

function saveControllerToFile(K_inf, K_inf_iters)
    fname = mfilename;
    fpath = mfilename('fullpath');
    dpath = strrep(fpath, fname, '');
    dpath = strcat(dpath, "/generated");
    [status, msg, msgID] = mkdir(dpath);
    save(strcat(dpath, "/K_inf_controller.mat"), "K_inf", "K_inf_iters");
end

function [Dl, Dr] = chooseDscales(muInfo, omega, orders)
    if nargin < 3
        orders = [-1, -1, -1, -1];
    end
    [Dl_w, Dr_w] = mussvunwrap(muInfo); % Extract D-scales

    D_perf = Dl_w(size(Dl_w, 1), size(Dl_w, 2));
    D_1 = Dl_w(1, 1);
    D_2 = Dl_w(2, 2);
    D_3 = Dl_w(3, 3);
    % D_1 = Dl_w(1, 1)/D_perf;
    % D_2 = Dl_w(2, 2)/D_perf;
    % D_3 = Dl_w(3, 3)/D_perf;

    D_1_fit = fitAndChooseDScale(D_1, omega, orders(1));
    D_2_fit = fitAndChooseDScale(D_2, omega, orders(2));
    D_3_fit = fitAndChooseDScale(D_3, omega, orders(3));
    D_p_fit = fitAndChooseDScale(D_perf, omega, orders(4));
    % D_p_fit = tf(1);

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
end

function D_i = fitAndChooseDScale(D_frd, omega, order)
    D_a = fitfrd(genphase(D_frd), 0);
    D_b = fitfrd(genphase(D_frd), 1);
    D_c = fitfrd(genphase(D_frd), 2);
    D_d = fitfrd(genphase(D_frd), 3);
    D_e = fitfrd(genphase(D_frd), 4);
    
    if order == -1
        fig_val = figure;
        hold on;
        plot(D_frd);
        plot(abs(frd(D_a, omega)));
        plot(abs(frd(D_b, omega)));
        plot(abs(frd(D_c, omega)));
        plot(abs(frd(D_d, omega)));
        plot(abs(frd(D_e, omega)));
        legend(["Exact", "0", "1", "2", "3", "4"]);
        xscale log;
        yscale log;
        
        figure(fig_val);
    
        choosenOrder = input("Choose the order of the D scale. i.e 0, 1, 2, 3, 4: ");
        close(fig_val);
    else
        choosenOrder = order;
    end
    
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
    elseif choosenOrder == 4
        fprintf("Order 4 chosen\n")
        D_i = D_e;
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

function K_inf = K_iteration_nominal(P, Ie, Iy, Iw, Iu)
    nmeas = size(Iy, 1);
    nctrl = size(Iu, 1);
    
    
    P_nom = P([Ie;Iy], [Iw;Iu]);
    P_nom = P;
    options = hinfsynOptions("Display", "off", "Method", "RIC");
    [K_inf, N_inf_unused, gamma, info] = hinfsyn(P_nom, nmeas, nctrl, options); %, ...
                                          % "METHOD", "ric", ...
                                          % "TOLGAM", 0.1); 
    if isnan(gamma)
        msg = sprintf("Failed to find H_inf controller\n");
        error(msg);
    end    
end

function K_inf = K_iteration(P_D, nmeas, nctrl)
    options = hinfsynOptions("Display", "off", "Method", "RIC");
    [K_inf, N_inf_unused, gamma, info] = hinfsyn(P_D, nmeas, nctrl, options);
    if isinf(gamma) || isnan(gamma)
        msg = sprintf("Failed to find H_inf controller\n");
        error(msg);
    end
end

function [muinfoRP, gammaRP] = calculate_and_plot_ssv(P, K, Iz, Ie, Iv, Iw, RS_blk, RP_blk, omega, iter)
    N_inf = lft(P, K);
    fig = figure("Name", sprintf("SSV Iter: %i", iter));
    [muinfoRP, gammaRP] = plot_ssv(N_inf, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk, [1, 1, 1]);
    fprintf("Worst case mu-RP: %f, iter: %i\n", gammaRP, iter);
end