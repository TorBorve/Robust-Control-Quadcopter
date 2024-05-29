%% Calc settings
clear all; close all;

% num_iter = 5;

% Orders which worked best
Dscale_orders_seq = [0, 1, 2, 0;
                     0, 2, 2, 3;
                     0, 3, 3, 0;
                     1, 1, 2, 0];
num_iter = size(Dscale_orders_seq, 1);

%% Init model and Weights
init_robust_simulink();

%% Model
fprintf("Extract linear system with pertubations and disturbances\n");
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
fprintf("Find initial H_inf controller\n");
K_inf = K_iteration_initial(P, Ie, Iy, Iw, Iu);
fprintf("Plot Info about initial controller\n");

[muinfo, gammaRP_nom] = calculate_and_plot_ssv(P, K_inf, Iz, Ie, Iv, Iw, RS_blk, RP_blk, omega, 0);
pause(0.1);

K_inf_iters = {K_inf};
K_inf_best = K_inf;
muinfo_best = muinfo;
gammaRP_best = gammaRP_nom;

for i=1:num_iter
    fprintf("\nDK-iteration %i\n", i);
    muinfo = muinfo_best;
    fprintf("Choose D scales\n");
    if exist("Dscale_orders_seq", "var") && size(Dscale_orders_seq, 1) >= i
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
    end

    fprintf("Plot info about controller\n");
    [muinfo, gammaRP] = calculate_and_plot_ssv(P, K_inf, Iz, Ie, Iv, Iw, RS_blk, RP_blk, omega, i);
    
    if gammaRP < gammaRP_best
        fprintf("Better controller found\n");
        K_inf_iters = [K_inf_iters ; {K_inf}];
        gammaRP_best = gammaRP;
        muinfo_best = muinfo;
        K_inf_best = K_inf;
    end
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

    D_1_fit = fitAndChooseDScale(D_1, omega, orders(1));
    D_2_fit = fitAndChooseDScale(D_2, omega, orders(2));
    D_3_fit = fitAndChooseDScale(D_3, omega, orders(3));
    D_p_fit = fitAndChooseDScale(D_perf, omega, orders(4));

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

function K_inf = K_iteration_initial(P, Ie, Iy, Iw, Iu)
    nmeas = size(Iy, 1);
    nctrl = size(Iu, 1);
    
    
    % P_nom = P([Ie;Iy], [Iw;Iu]);
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
    [u_muNP, u_muRS, u_muRP, muinfoRP] = calculate_ssv(N_inf, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk);

    fig = figure("Name", sprintf("SSV Iter: %i", iter));
    whichToPlot = [1, 1, 1];

    hold on;
    grid on;
    legends = [];
    if whichToPlot(1) == 1
        plot(u_muRS);
        legends = [legends, "RS"];
    end
    if whichToPlot(2) == 1
        plot(u_muNP);
        legends = [legends, "NP"];
    end
    if whichToPlot(3) == 1
        plot(u_muRP);
        legends = [legends, "RP"];
    end

    legend(legends, "Interpreter", "latex");
    xscale log;

    worst_muRP = max(u_muRP.ResponseData(1, 1, :));
    gammaRP = worst_muRP;
    fprintf("Worst case mu-RP: %f, iter: %i\n", gammaRP, iter);
end