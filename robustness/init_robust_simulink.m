N_freq_points = 250;
omega = logspace(-4, 2, N_freq_points);

s = tf("s");
load alternative_linearized_system.mat
M_inv_alt = inv(M_alt);

tau_act = 0.2;
G_act = 1/(s*tau_act+1); % Used in simulink

Je_inv = 1/M_alt(1, 1);
Q_Jp = lftForMInv(M_alt(2, 2), 0.2);

cutoff_freq_perf = 1e-2;
max_e_deav = 0.2;
max_p_deav = deg2rad(2);
max_u = 1000;
max_e_ref = 1;
max_p_ref = deg2rad(45);
W_perf_e = makeStepFilter(1/max_e_deav, cutoff_freq_perf, 1e-5);
W_perf_p = makeStepFilter(1/max_p_deav, cutoff_freq_perf, 1e-5);
W_perf_uf = tf(1/max_u);
W_perf_ub = W_perf_uf;
W_act = makeStepFilter(0.25, 2e1, 0.9);
max_noise_e = 0.1;
max_noise_p = deg2rad(1);
W_noise_e = tf(max_noise_e);
W_noise_p = tf(max_noise_p);
W_ref_e = tf(max_e_ref);
W_ref_p = tf(max_p_ref);

%% Know to work...
% Q_M = lftForMInv(M_alt, 0, 0.3);
% 
% cutoff_freq_perf = 1e-2;
% max_e_deav = 0.2;
% max_p_deav = deg2rad(0.5);
% max_u = 1000;
% max_e_ref = 1;
% max_p_ref = deg2rad(10);
% W_perf_e = makeStepFilter(1/max_e_deav, cutoff_freq_perf, 1e-5);
% W_perf_p = makeStepFilter(1/max_p_deav, cutoff_freq_perf, 1e-5);
% % W_perf_e = makeLowpassFilter(1e-1, 1e1);
% % W_perf_uf = makeStepFilter(1/max_u, cutoff_freq_perf*1e2, 1e0);
% W_perf_uf = tf(1/max_u);
% W_perf_ub = W_perf_uf;
% W_act = makeStepFilter(0.2, 1e2, 0.9);
% % W_act = tf(0.2);
% max_noise_e = 0.01;
% max_noise_p = deg2rad(0.1);
% W_noise_e = tf(max_noise_e);
% W_noise_p = tf(max_noise_p);
% W_ref_e = tf(max_e_ref);
% W_ref_p = tf(max_p_ref);


% Outputs
Iz = (1:3)'; % Connected to perturbation blocks
Ie = (4:7)'; % Error signals
Iy = (8:11)'; % Measurment signals for controller

% Inputs
Iv = (1:3)'; % Input Perturbation
Iw = (4:7)'; % Distrubrances and noise
Iu = (8:9)'; % actuation inputs

RS_blk = [1, 0;
          1, 0;
          -1, 0]; 
perf_blk = [size(Iw, 1), size(Ie, 1)];
RP_blk = [RS_blk;
          perf_blk];


function step_filter = makeStepFilter(start_gain, wc, end_gain)
    step_filter = makeweight(start_gain, [wc, (start_gain + end_gain)/2], end_gain);
end

function Q = lftForMInv(M_bar, p2)
    %% M = M(1 + P*Delta)
    %% M^-1 = Fu(Q, Delta), upper linear fractional transform

    M_p = M_bar*p2;    
    Q11 = -inv(M_bar)*M_p;
    Q12 = inv(M_bar);
    Q21 = -inv(M_bar)*M_p;
    Q22 = inv(M_bar);

    Q = [Q11, Q12;
         Q21, Q22];
end