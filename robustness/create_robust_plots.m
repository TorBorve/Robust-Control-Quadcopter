close all; clear all; clc;

%% Init Model and weights
init_robust_simulink();

%% Load controller
load("K_inf_controller.mat");
load("K_lqr_controller.mat");

% K_inf closed loop
K_sim = K_inf;
[A, B, C, D] = linmod("robust_model_closed_loop");
P_inf = ss(A, B, C, D);

% LQR closed loop
K_sim = K_lqr_ss;
[A, B, C, D] = linmod("robust_model_closed_loop");
P_lqr = ss(A, B, C, D);

% Outputs
Iz = (1:2)'; % Connected to perturbation blocks
Ie = (3:6)'; % Error signals

% Inputs
Iv = (1:2)'; % Input Perturbation
Iw = (3:6)'; % Distrubrances and noise

P_inf_nom = P_inf(Ie, Iw);
P_lqr_nom = P_lqr(Ie, Iw);

%% Step ref elevation;
t = 0:0.1:40;
figure("Name", "Elev Inf nom");
plotElevationStep(P_inf_nom, t);
% figure("Name", "Elev LQR nom");
% plotElevationStep(P_lqr_nom, t);
figure("Name", "Pitch Inf nom");
plotPitchStep(P_inf_nom, t);
% figure("Name", "Pitch LQR nom");
% plotPitchStep(P_lqr_nom, t);
% u = [ones(1, size(t, 2));
%      zeros(3, size(t, 2))];
% 
% y_inf = lsim(P_inf_nom, u, t);
% y_lqr = lsim(P_lqr_nom, u, t);
% 
% e_inf = y_inf(:, 3);
% e_lqr = y_lqr(:, 3);
% figure("Name", "Nominal elevation step");
% % lsim(P_nom, u, t);
% plot(t, e_inf, "DisplayName", "Inf");
% hold on;
% plot(t, e_lqr, "DisplayName", "LQR");
% grid on;
% legend();
% 
% u = [zeros(1, size(t, 2));
%      max_p_ref*ones(1, size(t, 2));
%      zeros(2, size(t, 2))];
% 
% y_inf = lsim(P_inf_nom, u, t);
% y_lqr = lsim(P_lqr_nom, u, t);
% p_inf = y_inf(:, 4);
% p_lqr = y_lqr(:, 4);
% figure("Name", "Nominal pitch step");
% % lsim(P_nom, u, t);
% grid on;
% hold on;
% plot(t, p_inf, "DisplayName", "Inf");
% plot(t, p_lqr, "DisplayName", "LQR");
% plot(t, u(2, :), "k--", "DisplayName", "p_{ref}");
% legend();

%% Worst case

% Open loop
[A_o, B_o, C_o, D_o] = linmod("robust_model");
P_o = ss(A_o, B_o, C_o, D_o);

Delta_wc_inf = getWorstCasePerturbation(P_o, K_inf, omega, RP_blk);
Delta_wc_lqr = getWorstCasePerturbation(P_o, K_lqr_ss, omega, RP_blk);

P_inf_pert = lft(Delta_wc_inf, P_inf);
P_lqr_pert = lft(Delta_wc_lqr, P_lqr);

figure("Name", "Elev inf pert");
plotElevationStep(P_inf_pert, t);
% figure("Name", "Elev LQR pert");
% plotElevationStep(P_lqr_pert, t);
figure("Name", "Pitch inf pert");
plotPitchStep(P_inf_pert, t);
% figure("Name", "Pitch LQR pert");
% plotPitchStep(P_lqr_pert, t);

function [Delta_wc] = getWorstCasePerturbation(P, K, omega, RP_blk)
    N = lft(P, K);
    N_w = frd(N, omega);
    [muRP, muinfoRP] = mussv(N_w, RP_blk);

    mudata = frdata(muRP);
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
