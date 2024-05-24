%% Init Model and weights
close all; clear all;
external_monitor = 1;

std_dims = [50 800 600 500];
if external_monitor == 1
    std_dims(2) = 1200;
end

FIG_PATH = "./figures";
if ~exist(FIG_PATH, "dir")
    mkdir(FIG_PATH);
end
init_robust_simulink();

%% Load controllers
load("K_inf_controller.mat");
load("K_lqr_controller.mat");
K_lqr = K_lqr_ss;

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

%% SSV Plots

% Plot SSV for each iteration
assert(size(K_inf_iters, 1) == 2);
fig = figure;
hold on;
grid on;
xscale log;
line_style_iter = ["--", "-"];
for i=1:2
    N_infi = lft(P_o, K_inf_iters{i});
    [muNP_infi, muRS_infi, muRP_infi] = calculate_ssv(N_infi, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk);

    line_style = line_style_iter(i);
    plot(omega, squeeze(muNP_infi.ResponseData), "Color", "#77AC30", "LineStyle", line_style, "DisplayName", sprintf("$NP_%i$", i));
    plot(omega, squeeze(muRS_infi.ResponseData), "Color", "#D95319", "LineStyle", line_style, "DisplayName", sprintf("$RS_%i$", i));
    plot(omega, squeeze(muRP_infi.ResponseData), "Color", "#0072BD", "LineStyle", line_style, "DisplayName", sprintf("$NP_%i$", i));
end
legend("Interpreter", "latex", "FontSize", 10);
xlabel("Frequency [rad/s]", "Interpreter", "latex");
ylabel("$\mu$", "Interpreter", "latex");
title("Structured Singular Value for $\mu$-controllers", "Interpreter", "latex");

dims = [std_dims(1:2), 600, 400];
set(fig, "renderer", "painters", "position", dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/ssv_h_inf.pdf",'ContentType','vector');

N_inf = lft(P_o, K_inf);
N_lqr = lft(P_o, K_lqr);
fig = figure;
hold on;
grid on;
xscale log;

[muNP_inf, muRS_inf, muRP_inf] = calculate_ssv(N_inf, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk);
[muNP_lqr, muRS_lqr, muRP_lqr] = calculate_ssv(N_lqr, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk);

plot(omega, squeeze(muNP_inf.ResponseData), "Color", "#77AC30", "LineStyle", "--", "DisplayName", "$H_\infty, NP$");
plot(omega, squeeze(muRS_inf.ResponseData), "Color", "#D95319", "LineStyle", "--", "DisplayName", "$H_\infty, RS$");
plot(omega, squeeze(muRP_inf.ResponseData), "Color", "#0072BD", "LineStyle", "--", "DisplayName", "$H_\infty, RP$");

plot(omega, squeeze(muNP_lqr.ResponseData), "Color", "#77AC30", "LineStyle", "-", "DisplayName", "$LQR, NP$");
plot(omega, squeeze(muRS_lqr.ResponseData), "Color", "#D95319", "LineStyle", "-", "DisplayName", "$LQR, RS$");
plot(omega, squeeze(muRP_lqr.ResponseData), "Color", "#0072BD", "LineStyle", "-", "DisplayName", "$LQR, RP$");

legend("Interpreter", "latex", "FontSize", 10, "Location", "northwest");
xlabel("Frequency [rad/s]", "Interpreter", "latex");
ylabel("$\mu$", "Interpreter", "latex");
title("Comparing SSV for LQR and $\mu$-controller", "Interpreter", "latex");

dims = [std_dims(1:2), 600, 500];
set(fig, "renderer", "painters", "position", dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/ssv_h_inf_vs_lqr.pdf",'ContentType','vector');



%% Step Responses
t = 0:0.1:20;
step_dims = [std_dims(1:2), 600, 600];

fig = figure("Name", "Elevation Step Nom");
plotElevationStep(P_inf_nom, t, "\mu", 0, 0);
plotElevationStep(P_lqr_nom, t, "LQR", 1, 0);
formatElevationRespFig("Nominal Elevation Step Response");
set(fig, "renderer", "painters", "position", step_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/elev_step_nom.pdf",'ContentType','vector');



fig = figure("Name", "Pitch Step Nom");
plotPitchStep(P_inf_nom, t, "\mu", 0, 0);
plotPitchStep(P_lqr_nom, t, "LQR", 1, 0);
formatPitchRespFig("Nominal Pitch Step Response");
set(fig, "renderer", "painters", "position", step_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/pitch_step_nom.pdf",'ContentType','vector');
pause(0.1);

%% Worst case

[Delta_wc_inf, maxmu_inf, maxmuRS_inf] = getWorstCasePerturbation(P_o, K_inf, omega, RP_blk, Iz, Iv, "RS");
[Delta_wc_lqr, maxmu_lqr, maxmuRS_lqr] = getWorstCasePerturbation(P_o, K_lqr_ss, omega, RP_blk, Iz, Iv, "RS");

fprintf("Max muRP inf: %f,\nMax muRP lqr: %f\n", maxmu_inf, maxmu_lqr);
fprintf("Max muRS inf: %f,\nMax muRS lqr: %f\n", maxmuRS_inf, maxmuRS_lqr);

P_inf_pert = lft(Delta_wc_inf, P_inf);
P_lqr_pert = lft(Delta_wc_lqr, P_lqr);

fig = figure("Name", "Elev pert");
plotElevationStep(P_inf_pert, t, "\mu", 0, 0);
plotElevationStep(P_lqr_pert, t, "LQR", 1, 0);
formatElevationRespFig("Worst Case Elevation Step Response");
subplot(2, 1, 1);
ylim([-5, 6]);
subplot(2, 1, 2);
ylim([-50, 50]);
set(fig, "renderer", "painters", "position", step_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/elev_step_wc.pdf",'ContentType','vector');


fig = figure("Name", "Pitch pert");
plotPitchStep(P_inf_pert, t, "\mu", 0, 0);
plotPitchStep(P_lqr_pert, t, "LQR", 1, 0);
formatPitchRespFig("Worst Case Pitch Step Response");
subplot(2, 1, 1);
ylim([-45, 135]);
subplot(2, 1, 2);
ylim([-60, 120]);
set(fig, "renderer", "painters", "position", step_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/pitch_step_wc.pdf",'ContentType','vector');

%% Comparision with Retuned LQR
load("K_lqr_controller_retuned.mat");
K_lqr_rt = K_lqr_ss_retuned;

% LQR closed loop
K_sim = K_lqr_rt;
[A, B, C, D] = linmod("robust_model_closed_loop");
P_lqr_rt = ss(A, B, C, D);

% [muNP_inf, muRS_inf, muRP_inf] = calculate_ssv(N_inf, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk);
N_lqr_rt = lft(P_o, K_lqr_rt);
[muNP_lqr_rt, muRS_lqr_rt, muRP_lqr_rt] = calculate_ssv(N_lqr_rt, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk);

fig = figure;
hold on;
grid on;
xscale log;



% Duplicate
[muNP_inf, muRS_inf, muRP_inf] = calculate_ssv(N_inf, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk);

plot(omega, squeeze(muNP_inf.ResponseData), "Color", "#77AC30", "LineStyle", "--", "DisplayName", "$H_\infty, NP$");
plot(omega, squeeze(muRS_inf.ResponseData), "Color", "#D95319", "LineStyle", "--", "DisplayName", "$H_\infty, RS$");
plot(omega, squeeze(muRP_inf.ResponseData), "Color", "#0072BD", "LineStyle", "--", "DisplayName", "$H_\infty, RP$");

plot(omega, squeeze(muNP_lqr_rt.ResponseData), "Color", "#77AC30", "LineStyle", "-", "DisplayName", "$LQR, NP$");
plot(omega, squeeze(muRS_lqr_rt.ResponseData), "Color", "#D95319", "LineStyle", "-", "DisplayName", "$LQR, RS$");
plot(omega, squeeze(muRP_lqr_rt.ResponseData), "Color", "#0072BD", "LineStyle", "-", "DisplayName", "$LQR, RP$");

legend("Interpreter", "latex", "FontSize", 10, "Location", "northwest");
xlabel("Frequency [rad/s]", "Interpreter", "latex");
ylabel("$\mu$", "Interpreter", "latex");
title("Comparing $\mu$ for retuned LQR and $H_\infty$", "Interpreter", "latex");

dims = [std_dims(1:2), 600, 500];
set(fig, "renderer", "painters", "position", dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/ssv_h_inf_vs_retuned_lqr.pdf",'ContentType','vector');

%% Nominal Retuned

P_lqr_rt_nom = P_lqr_rt(Ie, Iw);

noise = 0;

fig = figure("Name", "Elevation Step Nom");
plotElevationStep(P_inf_nom, t, "\mu", 0, noise);
plotElevationStep(P_lqr_rt_nom, t, "LQR", 1, noise);
formatElevationRespFig("Nominal Retuned LQR Step Response");
set(fig, "renderer", "painters", "position", step_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/elev_step_nom_retuned.pdf",'ContentType','vector');



fig = figure("Name", "Pitch Step Nom");
plotPitchStep(P_inf_nom, t, "\mu", 0, noise);
plotPitchStep(P_lqr_rt_nom, t, "LQR", 1, noise);
formatPitchRespFig("Nominal Retuned LQR Step Response");
set(fig, "renderer", "painters", "position", step_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/pitch_step_nom_retuned.pdf",'ContentType','vector');
pause(0.1);


%% Worst case retuned

[Delta_wc_inf, maxmuRP_inf, maxmuRS_inf] = getWorstCasePerturbation(P_o, K_inf, omega, RP_blk, Iz, Iv, "RP");
[Delta_wc_lqr_rt, maxmuRP_lqr_rt, maxmuRS_lqr_rt] = getWorstCasePerturbation(P_o, K_lqr_rt, omega, RP_blk, Iz, Iv, "RP");
Delta_wc_lqr_rt = Delta_wc_inf;

fprintf("Max muRP retuned LQR: %f\n", maxmuRP_lqr_rt);
fprintf("Max muRS retuned LQR: %f\n", maxmuRS_lqr_rt);

P_inf_pert = lft(Delta_wc_inf, P_inf);
P_lqr_rt_pert = lft(Delta_wc_lqr_rt, P_lqr_rt);

noise = 0;

fig = figure("Name", "Elev pert");
plotElevationStep(P_inf_pert, t, "\mu", 0, noise);
plotElevationStep(P_lqr_rt_pert, t, "LQR", 1, noise);
formatElevationRespFig("Worst Case Retuned LQR");
% subplot(2, 1, 1);
% ylim([-5, 6]);
% subplot(2, 1, 2);
% ylim([-50, 50]);
set(fig, "renderer", "painters", "position", step_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/elev_step_wc_retuned.pdf",'ContentType','vector');


fig = figure("Name", "Pitch pert");
plotPitchStep(P_inf_pert, t, "\mu", 0, noise);
plotPitchStep(P_lqr_rt_pert, t, "LQR", 1, noise);
formatPitchRespFig("Worst Case Retuned LQR");
% subplot(2, 1, 1);
% ylim([-45, 135]);
% subplot(2, 1, 2);
% ylim([-60, 120]);
set(fig, "renderer", "painters", "position", step_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/pitch_step_wc_retuned.pdf",'ContentType','vector');

noise = 1;
fig = figure("Name", "Elev pert");
plotElevationStep(P_inf_pert, t, "\mu", 0, noise);
plotElevationStep(P_lqr_rt_pert, t, "LQR", 1, noise);
formatElevationRespFig("Worst Case Retuned LQR with Noise");
% subplot(2, 1, 1);
% ylim([-5, 6]);
% subplot(2, 1, 2);
% ylim([-50, 50]);
set(fig, "renderer", "painters", "position", step_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/elev_step_wc_retuned_noise.pdf",'ContentType','vector');


fig = figure("Name", "Pitch pert");
plotPitchStep(P_inf_pert, t, "\mu", 0, noise);
plotPitchStep(P_lqr_rt_pert, t, "LQR", 1, noise);
formatPitchRespFig("Worst Case Retuned LQR with Noise");
% subplot(2, 1, 1);
% ylim([-45, 135]);
% subplot(2, 1, 2);
% ylim([-60, 120]);
set(fig, "renderer", "painters", "position", step_dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/pitch_step_wc_retuned_noise.pdf",'ContentType','vector');



%% Functions

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

function plotElevationStep(P_c, t, name_prefix, plot_ref, use_noise)
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
    subplot(2, 1, 1);
    hold on;
    plot(t, e, "DisplayName", sprintf("$%s: e$", name_prefix));
    if plot_ref == 1
        plot(t, u(1, :), "k--", "DisplayName", "$e_{ref}$");
    end
    subplot(2, 1, 2);
    hold on;
    plot(t, u_f, "DisplayName", sprintf("$%s: u_f$", name_prefix));
    plot(t, u_b, "DisplayName", sprintf("$%s: u_b$", name_prefix));
end

function plotPitchStep(P_c, t, name_prefix, plot_ref, use_noise)
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
    subplot(2, 1, 1);
    hold on;
    plot(t, p, "DisplayName", sprintf("$%s: p$", name_prefix));
    if plot_ref == 1
        plot(t, rad2deg(u(2, :)), "k--", "DisplayName", "$p_{ref}$");
    end
    subplot(2, 1, 2);
    hold on;
    plot(t, u_f, "DisplayName", sprintf("$%s: u_f$", name_prefix));
    plot(t, u_b, "DisplayName", sprintf("$%s: u_b$", name_prefix));
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

function formatPitchRespFig(mainTitle)
    formatCommonRespFig(mainTitle);
    subplot(2, 1, 1);
    grid on;
    title("Pitch", "Interpreter", "latex");
    ylabel("Pitch [deg]", "Interpreter", "latex");
    subplot(2, 1, 2);
end

function formatElevationRespFig(mainTitle)
    formatCommonRespFig(mainTitle);
    subplot(2, 1, 1);
    title("Elevation", "Interpreter", "latex")
    ylabel("Elevation [m]", "Interpreter", "latex");
    subplot(2, 1, 2);
end

function formatCommonRespFig(mainTitle)
    subplot(2, 1, 1);
    grid on;
    legend("Interpreter", "latex", "Location", "southeast");
    xlabel("Time [s]", "Interpreter", "latex");
    subplot(2, 1, 2);
    title("Acutation Effort", "Interpreter", "latex");
    grid on;
    legend("Interpreter", "latex");
    xlabel("Time [s]", "Interpreter", "latex");
    ylabel("Actuation [N]", "Interpreter", "latex");
    sgtitle(mainTitle,  "interpreter", "latex");
end

