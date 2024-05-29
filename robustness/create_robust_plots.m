%% Init Model and weights
close all; clear all;

external_monitor = 0;
figs = {};

std_dims = [50 800 500 400];
legend_size = 10;
title_size = 12;

if external_monitor == 1
    std_dims(2) = 1200;
end

FIG_PATH = "./figures";
if ~exist(FIG_PATH, "dir")
    mkdir(FIG_PATH);
end
init_robust_simulink();

%% Load controllers
fprintf("Load controllers\n");
load("K_inf_controller.mat");
load("K_lqr_controller.mat");
K_lqr = K_lqr_ss;

fprintf("Extract closed loop system for the controllers\n");
% K_inf closed loop
K_sim = K_inf;
[A, B, C, D] = linmod("robust_model_closed_loop");
P_inf = ss(A, B, C, D); 

% LQR closed loop
K_sim = K_lqr_ss;
[A, B, C, D] = linmod("robust_model_closed_loop");
P_lqr = ss(A, B, C, D);


P_inf_nom = P_inf(Ie, Iw);
P_lqr_nom = P_lqr(Ie, Iw);

% Open loop
[A_o, B_o, C_o, D_o] = linmod("robust_model");
P_o = ss(A_o, B_o, C_o, D_o);

%% Weights
fprintf("Plot uncertainty and performance weights\n");
fig = figure("Name", "Weights");
figs = {figs, fig};
weight_plot(W_perf_e, W_perf_p, W_perf_uf, W_act, p_Jp, W_noise_e, W_noise_p, W_ref_e, W_ref_p, omega)
weight_dims = [std_dims(1:3), 300];
set(fig, "renderer", "painters", "position", weight_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/perf_weight.pdf",'ContentType','vector');
%% SSV Plots
fprintf("Plot SSV evolution for mu-controller\n");
fig = figure("Name", "SSV comparison");
figs = {figs, fig};
hold on;
grid on;
xscale log;
line_style_iter = [repmat(["--"], 1, size(K_inf_iters, 1)-1), "-"];
for i=1:size(K_inf_iters, 1)
    N_infi = lft(P_o, K_inf_iters{i});
    [muNP_infi, muRS_infi, muRP_infi] = calculate_ssv(N_infi, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk);

    line_style = line_style_iter(i);
    plot(omega, squeeze(muRP_infi.ResponseData), "LineStyle", line_style, "DisplayName", sprintf("$RP_%i$", i-1));
end
legend("Interpreter", "latex", "FontSize", legend_size);
xlabel("Frequency [rad/s]", "Interpreter", "latex");
ylabel("SSV", "Interpreter", "latex");
title("Structured Singular Value for $\mu$-controllers", "Interpreter", "latex", "FontSize", title_size);

ssv_dims = [std_dims(1:3), 300];
set(fig, "renderer", "painters", "position", ssv_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/ssv_h_inf.pdf",'ContentType','vector');

fprintf("Plot SSV comparison for mu- and LQR controller\n");
N_inf = lft(P_o, K_inf);
N_lqr = lft(P_o, K_lqr);
fig = figure("Name", "SSV LQR and mu");
figs = {figs, fig};
hold on;
grid on;
xscale log;

[muNP_inf, muRS_inf, muRP_inf] = calculate_ssv(N_inf, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk);
[muNP_lqr, muRS_lqr, muRP_lqr] = calculate_ssv(N_lqr, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk);

plot(omega, squeeze(muNP_inf.ResponseData), "Color", "#77AC30", "LineStyle", "--", "DisplayName", "$\mu, NP$");
plot(omega, squeeze(muRS_inf.ResponseData), "Color", "#D95319", "LineStyle", "--", "DisplayName", "$\mu, RS$");
plot(omega, squeeze(muRP_inf.ResponseData), "Color", "#0072BD", "LineStyle", "--", "DisplayName", "$\mu, RP$");

plot(omega, squeeze(muNP_lqr.ResponseData), "Color", "#77AC30", "LineStyle", "-", "DisplayName", "$LQR, NP$");
plot(omega, squeeze(muRS_lqr.ResponseData), "Color", "#D95319", "LineStyle", "-", "DisplayName", "$LQR, RS$");
plot(omega, squeeze(muRP_lqr.ResponseData), "Color", "#0072BD", "LineStyle", "-", "DisplayName", "$LQR, RP$");

legend("Interpreter", "latex", "FontSize", legend_size, "Location", "northwest");
xlabel("Frequency [rad/s]", "Interpreter", "latex");
ylabel("SSV", "Interpreter", "latex");
title("Comparing SSV for LQR and $\mu$-controller", "Interpreter", "latex", "FontSize", title_size);

ssv_dims = [std_dims(1:3), 300];
set(fig, "renderer", "painters", "position", ssv_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/ssv_h_inf_vs_lqr.pdf",'ContentType','vector');



%% Step Responses
fprintf("Simulate and plot nominal step responses for LQR and mu-controller\n");
t = 0:0.1:20;
step_dims = [std_dims(1:3), 500];

fig = figure("Name", "Elevation Step Nom");
figs = {figs, fig};
plotElevationStep(P_inf_nom, t, "\mu", 0, 0, 1);
plotElevationStep(P_lqr_nom, t, "LQR", 1, 0, 1);
formatElevationRespFig("Nominal Elevation Step Response", 1);
set(fig, "renderer", "painters", "position", step_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/elev_step_nom.pdf",'ContentType','vector');



fig = figure("Name", "Pitch Step Nom");
figs = {figs, fig};
plotPitchStep(P_inf_nom, t, "\mu", 0, 0, 1);
plotPitchStep(P_lqr_nom, t, "LQR", 1, 0, 1);
formatPitchRespFig("Nominal Pitch Step Response", 1);
set(fig, "renderer", "painters", "position", step_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/pitch_step_nom.pdf",'ContentType','vector');
pause(0.1);

%% Worst case
fprintf("Simulate and plot worst case perturbation\n");
[Delta_wc_inf, maxmu_inf, maxmuRS_inf] = getWorstCasePerturbation(P_o, K_inf, omega, RP_blk, Iz, Iv, "RS");
[Delta_wc_lqr, maxmu_lqr, maxmuRS_lqr] = getWorstCasePerturbation(P_o, K_lqr_ss, omega, RP_blk, Iz, Iv, "RS");

fprintf("Max muRP inf: %f,\nMax muRP lqr: %f\n", maxmu_inf, maxmu_lqr);
fprintf("Max muRS inf: %f,\nMax muRS lqr: %f\n", maxmuRS_inf, maxmuRS_lqr);

P_inf_pert = lft(Delta_wc_inf, P_inf);
P_lqr_pert = lft(Delta_wc_lqr, P_lqr);

fig = figure("Name", "Elev perturbed");
figs = {figs, fig};
plotElevationStep(P_inf_pert, t, "\mu", 0, 0, 0);
plotElevationStep(P_lqr_pert, t, "LQR", 1, 0, 0);
formatElevationRespFig("Worst Case Elevation Step Response", 0);
ylim([-5, 6]);
half_step_dims = [std_dims(1:3), 250];
set(fig, "renderer", "painters", "position", half_step_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/elev_step_wc.pdf",'ContentType','vector');


fig = figure("Name", "Pitch perturbed");
figs = {figs, fig};
plotPitchStep(P_inf_pert, t, "\mu", 0, 0, 0);
plotPitchStep(P_lqr_pert, t, "LQR", 1, 0, 0);
formatPitchRespFig("Worst Case Pitch Step Response", 0);
ylim([-45, 135]);
set(fig, "renderer", "painters", "position", half_step_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/pitch_step_wc.pdf",'ContentType','vector');

%% Comparision with Retuned LQR
fprintf("Comparing with retuned LQR\n");
load("K_lqr_controller_retuned.mat");
K_lqr_rt = K_lqr_ss_retuned;

% LQR closed loop
K_sim = K_lqr_rt;
[A, B, C, D] = linmod("robust_model_closed_loop");
P_lqr_rt = ss(A, B, C, D);

fprintf("Plot SSV for retuned LQR\n");
N_lqr_rt = lft(P_o, K_lqr_rt);
[muNP_lqr_rt, muRS_lqr_rt, muRP_lqr_rt] = calculate_ssv(N_lqr_rt, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk);
[muNP_inf, muRS_inf, muRP_inf] = calculate_ssv(N_inf, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk);

fig = figure("Name", "SSV retuned LQR and mu");
figs = {figs, fig};
hold on;
grid on;
xscale log;

plot(omega, squeeze(muNP_inf.ResponseData), "Color", "#77AC30", "LineStyle", "--", "DisplayName", "$\mu, NP$");
plot(omega, squeeze(muRS_inf.ResponseData), "Color", "#D95319", "LineStyle", "--", "DisplayName", "$\mu, RS$");
plot(omega, squeeze(muRP_inf.ResponseData), "Color", "#0072BD", "LineStyle", "--", "DisplayName", "$\mu, RP$");

plot(omega, squeeze(muNP_lqr_rt.ResponseData), "Color", "#77AC30", "LineStyle", "-", "DisplayName", "$LQR, NP$");
plot(omega, squeeze(muRS_lqr_rt.ResponseData), "Color", "#D95319", "LineStyle", "-", "DisplayName", "$LQR, RS$");
plot(omega, squeeze(muRP_lqr_rt.ResponseData), "Color", "#0072BD", "LineStyle", "-", "DisplayName", "$LQR, RP$");

legend("Interpreter", "latex", "FontSize", legend_size, "Location", "northwest");
xlabel("Frequency [rad/s]", "Interpreter", "latex");
ylabel("SSV", "Interpreter", "latex");
title("Comparing SSVs for retuned LQR and $\mu$-controller", "Interpreter", "latex", "FontSize", title_size);

set(fig, "renderer", "painters", "position", ssv_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/ssv_h_inf_vs_retuned_lqr.pdf",'ContentType','vector');

%% Nominal Retuned
% Nominal response for retuned LQR. Not included in report.
% P_lqr_rt_nom = P_lqr_rt(Ie, Iw);
% 
% noise = 0;
% 
% fig = figure("Name", "Elevation Step Nom");
% figs = {figs, fig};
% plotElevationStep(P_inf_nom, t, "\mu", 0, noise);
% plotElevationStep(P_lqr_rt_nom, t, "LQR", 1, noise);
% formatElevationRespFig("Nominal Retuned LQR Step Response");
% set(fig, "renderer", "painters", "position", step_dims, "PaperPositionMode", "auto");
% exportgraphics(fig,"./figures/elev_step_nom_retuned.pdf",'ContentType','vector');
% 
% fig = figure("Name", "Pitch Step Nom");
% figs = {figs, fig};
% plotPitchStep(P_inf_nom, t, "\mu", 0, noise);
% plotPitchStep(P_lqr_rt_nom, t, "LQR", 1, noise);
% formatPitchRespFig("Nominal Retuned LQR Step Response");
% set(fig, "renderer", "painters", "position", step_dims, "PaperPositionMode", "auto");
% exportgraphics(fig,"./figures/pitch_step_nom_retuned.pdf",'ContentType','vector');
% pause(0.1);


%% Worst case retuned
fprintf("Finding worst case perturbation for retuned LQR\n");
[Delta_wc_inf, maxmuRP_inf, maxmuRS_inf] = getWorstCasePerturbation(P_o, K_inf, omega, RP_blk, Iz, Iv, "RP");
[Delta_wc_lqr_rt, maxmuRP_lqr_rt, maxmuRS_lqr_rt] = getWorstCasePerturbation(P_o, K_lqr_rt, omega, RP_blk, Iz, Iv, "RP");
Delta_wc_lqr_rt = Delta_wc_inf; % Use the same pertubation for comparison

fprintf("Max muRP retuned LQR: %f\n", maxmuRP_lqr_rt);
fprintf("Max muRS retuned LQR: %f\n", maxmuRS_lqr_rt);

P_inf_pert = lft(Delta_wc_inf, P_inf);
P_lqr_rt_pert = lft(Delta_wc_lqr_rt, P_lqr_rt);

fprintf("Plot worst case for retuned LQR without noise\n");

noise = 0;
plot_acutation = 1;

fig = figure("Name", "Elev pert");
figs = {figs, fig};
plotElevationStep(P_inf_pert, t, "\mu", 0, noise, plot_acutation);
plotElevationStep(P_lqr_rt_pert, t, "LQR", 1, noise, plot_acutation);
formatElevationRespFig("Worst Case Retuned LQR", plot_acutation);

set(fig, "renderer", "painters", "position", step_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/elev_step_wc_retuned.pdf",'ContentType','vector');


fig = figure("Name", "Pitch pert");
figs = {figs, fig};
plotPitchStep(P_inf_pert, t, "\mu", 0, noise, plot_acutation);
plotPitchStep(P_lqr_rt_pert, t, "LQR", 1, noise, plot_acutation);
formatPitchRespFig("Worst Case Retuned LQR", plot_acutation);
subplot(2, 1, 2);
ylim([-10, 40]);

set(fig, "renderer", "painters", "position", step_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/pitch_step_wc_retuned.pdf",'ContentType','vector');

fprintf("Plot worst case for retuned LQR with noise\n");
noise = 1;
fig = figure("Name", "Elev pert");
figs = {figs, fig};
plotElevationStep(P_inf_pert, t, "\mu", 0, noise, plot_acutation);
plotElevationStep(P_lqr_rt_pert, t, "LQR", 1, noise, plot_acutation);
formatElevationRespFig("Worst Case Retuned LQR with Noise", plot_acutation);
subplot(2, 1, 2);
ylim([-10, 25])

set(fig, "renderer", "painters", "position", step_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/elev_step_wc_retuned_noise.pdf",'ContentType','vector');


fig = figure("Name", "Pitch pert");
plotPitchStep(P_inf_pert, t, "\mu", 0, noise, plot_acutation);
plotPitchStep(P_lqr_rt_pert, t, "LQR", 1, noise, plot_acutation);
formatPitchRespFig("Worst Case Retuned LQR with Noise", plot_acutation);
subplot(2, 1, 2);
ylim([-20, 50]);

set(fig, "renderer", "painters", "position", step_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/pitch_step_wc_retuned_noise.pdf",'ContentType','vector');

%% Save Workspace
% Save workspace so that the variables are available even using a different
% MATLAB VERSION
fig = -1;
fname = mfilename;
fpath = mfilename('fullpath');
dpath = strrep(fpath, fname, '');
dpath = strcat(dpath, "/generated");
[status, msg, msgID] = mkdir(dpath);
save(strcat(dpath,"/plot_workspace.mat"));


%% Function definitions

function [Delta_wc, maxmuRP, maxmuRS] = getWorstCasePerturbation(P, K, omega, RP_blk, Iz, Iv, type)
    N = lft(P, K);
    N_w = frd(N, omega);
    [muRP, muinfoRP] = mussv(N_w, RP_blk);
    RS_blk = RP_blk(1:end-1, :);
    [muRS, muinfoRS] = mussv(N_w(Iz, Iv), RS_blk);
    muRSdata = frdata(muRS);
    maxmuRS = max(muRSdata(1, 1, :));

    muRPdata = frdata(muRP);
    maxmuRP = max(muRPdata(1, 1, :));

    if type == "RP" % Choose RS or RP worst case
        mudata = muRPdata;
        Delta0 = mussvunwrap(muinfoRP);
    elseif type == "RS"
        mudata = muRSdata;
        Delta0 = mussvunwrap(muinfoRS);
    else
        error("invalid perturbation type");
    end
    maxmu = max(mudata(1, 1, :));
    maxidx = find(maxmu == mudata(1, 1, :));
    maxidx = maxidx(1);
    
    
    Delta0data = frdata(Delta0);
    Delta0data_wc = Delta0data(:, :, maxidx);

    s = tf("s");
    
    % Fit data
    Delta0_wc = ss(zeros(size(RP_blk, 1) - 1));
    
    for i = 1:size(Delta0_wc, 1)
        delta_i = Delta0data_wc(i, i);
        gamma = abs(delta_i);
        if imag(delta_i) > 0
            delta_i = -delta_i;
            gamma = -gamma;
        end
        x = real(delta_i)/abs(gamma);
        tau = 2*omega(maxidx)*(sqrt((1+x)/(1-x)));
        if isnan(tau) || isinf(tau)
            Delta0_wc(i, i) = gamma;
        else
            Delta0_wc(i, i) = gamma*(-s+tau/2)/(s+tau/2);
        end
    end
    nDelta = norm(Delta0data_wc);
    Delta0_wc = 1/nDelta*Delta0_wc;
    Delta_wc = Delta0_wc;
end

function plotElevationStep(P_c, t, name_prefix, plot_ref, use_noise, plot_actuation)
    u_ref = [ones(1, size(t, 2));
             zeros(1, size(t, 2))];
    if use_noise == 1
        u_noise = generateNoise(size(t, 2));
    else
        u_noise = [zeros(2, size(t, 2))];
    end
    u = [u_ref;
         u_noise];
    y = lsim(P_c, u, t);

    e = y(:, 3);
    u_f = y(:, 1);
    u_b = y(:, 2);
    if plot_actuation == 1
        subplot(2, 1, 1);
    end
    hold on;
    plot(t, e, "DisplayName", sprintf("$%s: e$", name_prefix));
    if plot_ref == 1
        plot(t, u(1, :), "k--", "DisplayName", "$e_{ref}$");
    end
    if plot_actuation == 1
        subplot(2, 1, 2);
        hold on;
        plot(t, u_f, "DisplayName", sprintf("$%s: u_f$", name_prefix));
        plot(t, u_b, "DisplayName", sprintf("$%s: u_b$", name_prefix));
    end
end

function plotPitchStep(P_c, t, name_prefix, plot_ref, use_noise, plot_acutation)
    u_ref = [zeros(1, size(t, 2));
            deg2rad(45)*ones(1, size(t, 2))];
    if use_noise == 1
        u_noise = generateNoise(size(t, 2));
    else
        u_noise = [zeros(2, size(t, 2))];
    end
    u = [u_ref;
         u_noise];
    y = lsim(P_c, u, t);

    p = rad2deg(y(:, 4));
    u_f = y(:, 1);
    u_b = y(:, 2);
    if plot_acutation == 1
        subplot(2, 1, 1);
    end
    hold on;
    plot(t, p, "DisplayName", sprintf("$%s: p$", name_prefix));
    if plot_ref == 1
        plot(t, rad2deg(u(2, :)), "k--", "DisplayName", "$p_{ref}$");
    end
    if plot_acutation == 1
        subplot(2, 1, 2);
        hold on;
        plot(t, u_f, "DisplayName", sprintf("$%s: u_f$", name_prefix));
        plot(t, u_b, "DisplayName", sprintf("$%s: u_b$", name_prefix));
    end
end

function [y_noise] = generateNoise(N)
    init_robust_simulink;
    % Asume uniform noise
    % e_noise = 2*rand(1, N) - 1;
    % p_noise = 2*rand(1, N) - 1;
    e_noise = 2*(randi([0, 1], 1, N) - 1/2);
    p_noise = 2*(randi([0, 1], 1, N) - 1/2);
    e_noise = max_noise_e*e_noise;
    p_noise = max_noise_p*p_noise;
    assert(max(abs(p_noise)) <= max_noise_p);
    assert(max(abs(e_noise)) <= max_noise_e);
    y_noise = [e_noise;
               p_noise];
end

function formatPitchRespFig(mainTitle, plot_actuation)
    formatCommonRespFig(mainTitle, plot_actuation);
    if plot_actuation == 1
        subplot(2, 1, 1);
        title("Pitch", "Interpreter", "latex");
    else 
        title_size = evalin("caller", "title_size");
        title(mainTitle, "interpreter", "latex", "FontSize", title_size);
    end
    
    ylabel("Pitch [deg]", "Interpreter", "latex");
    if plot_actuation == 1
        subplot(2, 1, 2);
        legend_size = evalin("caller", "legend_size");
        legend("Interpreter", "latex", "Location", "southeast", "FontSize", legend_size);
    end
end

function formatElevationRespFig(mainTitle, plot_actuation)
    formatCommonRespFig(mainTitle, plot_actuation);
    if plot_actuation == 1
        subplot(2, 1, 1);
        title("Elevation", "Interpreter", "latex")
    else 
        title_size = evalin("caller", "title_size");
        title(mainTitle, "interpreter", "latex", "FontSize", title_size);
    end
    ylabel("Elevation [m]", "Interpreter", "latex");
    % subplot(2, 1, 2);
end

function formatCommonRespFig(mainTitle, plot_actuation)
    if plot_actuation == 1
        subplot(2, 1, 1);
    end
    grid on;
    % legend_size = evalin("caller", "legend_size");
    legend("Interpreter", "latex", "Location", "southeast", "FontSize", 10);
    xlabel("Time [s]", "Interpreter", "latex");
    if plot_actuation == 1
        subplot(2, 1, 2);
        title("Acutation Effort", "Interpreter", "latex");
        grid on;
        legend("Interpreter", "latex", "FontSize", 10);
        xlabel("Time [s]", "Interpreter", "latex");
        ylabel("Actuation [N]", "Interpreter", "latex");
        sgtitle(mainTitle,  "interpreter", "latex");
    end
end

