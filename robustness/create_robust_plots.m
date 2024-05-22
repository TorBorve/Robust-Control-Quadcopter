%% Init Model and weights
close all; clear all;
FIG_PATH = "./figures";
if ~exist(FIG_PATH, "dir")
    mkdir(FIG_PATH);
end
init_robust_simulink();

%% Load controller
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

%% SSV Plots


%% Step ref elevation;
t = 0:0.1:40;
figure("Name", "Elev Inf nom");
plotElevationStep(P_inf_nom, t);
figure("Name", "Elev LQR nom");
plotElevationStep(P_lqr_nom, t);
figure("Name", "Pitch Inf nom");
plotPitchStep(P_inf_nom, t);
figure("Name", "Pitch LQR nom");
plotPitchStep(P_lqr_nom, t);
pause(0.1);

%% Worst case

% Open loop
[A_o, B_o, C_o, D_o] = linmod("robust_model");
P_o = ss(A_o, B_o, C_o, D_o);

N_inf = lft(P_o, K_inf);
N_lqr = lft(P_o, K_lqr);
fig = figure;
plot_ssv(N_inf, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk, [1, 1, 1]);
fig = figure;
plot_ssv(N_lqr, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk, [1, 1, 1]);

[Delta_wc_inf, maxmu_inf, maxmuRS_inf] = getWorstCasePerturbation(P_o, K_inf, omega, RP_blk, Iz, Iv);
[Delta_wc_lqr, maxmu_lqr, maxmuRS_lqr] = getWorstCasePerturbation(P_o, K_lqr_ss, omega, RP_blk, Iz, Iv);

fprintf("Max muRP inf: %f,\nMax muRP lqr: %f\n", maxmu_inf, maxmu_lqr);
fprintf("Max muRS inf: %f,\nMax muRS lqr: %f\n", maxmuRS_inf, maxmuRS_lqr);

P_inf_pert = lft(Delta_wc_inf, P_inf);
P_lqr_pert = lft(Delta_wc_lqr, P_lqr);

figure("Name", "Elev inf pert");
plotElevationStep(P_inf_pert, t);
figure("Name", "Elev LQR pert");
plotElevationStep(P_lqr_pert, t);
figure("Name", "Pitch inf pert");
plotPitchStep(P_inf_pert, t);
figure("Name", "Pitch LQR pert");
plotPitchStep(P_lqr_pert, t);

function [Delta_wc, maxmuRP, maxmuRS] = getWorstCasePerturbation(P, K, omega, RP_blk, Iz, Iv)
    N = lft(P, K);
    N_w = frd(N, omega);
    [muRP, muinfoRP] = mussv(N_w, RP_blk);
    RS_blk = RP_blk(1:end-1, :);
    muRS = mussv(N_w(Iz, Iv), RS_blk);
    muRSdata = frdata(muRS);
    maxmuRS = max(muRSdata(1, 1, :));

    muRPdata = frdata(muRP);
    maxmuRP = max(muRPdata(1, 1, :));

    % mudata = muRPdata;
    mudata = muRSdata;
    maxmu = max(mudata(1, 1, :));
    maxidx = find(maxmu == mudata(1, 1, :));
    maxidx = maxidx(1);
    
    Delta0 = mussvunwrap(muinfoRP);
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

function plotElevationStep(P_c, t)
    u_ref = [ones(1, size(t, 2));
             zeros(1, size(t, 2))];
    u_noise = [zeros(2, size(t, 2))];
    u = [u_ref;
         u_noise];
    y = lsim(P_c, u, t);

    e = y(:, 3);
    u_f = y(:, 1);
    u_b = y(:, 2);
    subplot(2, 1, 1);
    hold on;
    plot(t, e, "DisplayName", "e");
    plot(t, u(1, :), "k--");
    subplot(2, 1, 2);
    hold on;
    plot(t, u_f, "DisplayName", "u_f");
    plot(t, u_b, "DisplayName", "u_b");
    formatStepRespFig();
end

function plotPitchStep(P_c, t)
    u_ref = [zeros(1, size(t, 2));
            ones(1, size(t, 2))];
    u_noise = [zeros(2, size(t, 2))];
    u = [u_ref;
         u_noise];
    y = lsim(P_c, u, t);

    p = y(:, 4);
    u_f = y(:, 1);
    u_b = y(:, 2);
    subplot(2, 1, 1);
    hold on;
    plot(t, p, "DisplayName", "p");
    plot(t, u(2, :), "k--");
    subplot(2, 1, 2);
    hold on;
    plot(t, u_f, "DisplayName", "u_f");
    plot(t, u_b, "DisplayName", "u_b");
    formatStepRespFig();
end

function formatStepRespFig()
    subplot(2, 1, 1);
    grid on;
    legend();
    subplot(2, 1, 2);
    grid on;
    legend();
end

