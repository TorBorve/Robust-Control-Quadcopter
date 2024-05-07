N_freq_points = 250;
omega = logspace(-4, 2, N_freq_points);

s = tf("s");
% G_ex = 1/(0*s + 1)*[87.8, -86.4;
%                      108.2, -109.6];
load alternative_linearized_system.mat
M_inv_alt = inv(M_alt);
F_alt = 0*M_alt*diag([-0.1, -0.1]);

tau_act = 0.2;
G_act = 1/(s*tau_act+1); % Used in simulink

cutoff_freq_perf = 1e-2;
max_e_deav = 10;
max_p_deav = deg2rad(1);
max_u = 100;
max_e_ref = 1;
max_p_ref = deg2rad(25);
W_perf_e = makeStepFilter(1/max_e_deav, cutoff_freq_perf, 1e-5);
W_perf_p = makeStepFilter(1/max_p_deav, cutoff_freq_perf, 1e-5);
% W_perf_e = makeLowpassFilter(1e-1, 1e1);
% W_perf_uf = makeStepFilter(1/max_u, cutoff_freq_perf*1e2, 1e0);
W_perf_uf = tf(1/max_u);
W_perf_ub = W_perf_uf;
W_act = makeStepFilter(0.2, 1e1, 0.9);
max_noise_e = 0.1;
max_noise_p = deg2rad(5);
W_noise_e = tf(max_noise_e);
W_noise_p = tf(max_noise_p);
W_ref_e = tf(max_e_ref);
W_ref_p = tf(max_p_ref);


% Outputs
Iz = (1:2)'; % Connected to perturbation blocks
Ie = (3:6)'; % Error signals
Iy = (7:10)'; % Measurment signals for controller

% Inputs
Iv = (1:2)'; % Input Perturbation
Iw = (3:6)'; % Distrubrances and noise
Iu = (7:8)'; % actuation inputs

RS_blk = [-1, 0;
          -1, 0];
perf_blk = [size(Iw, 1), size(Ie, 1)];
RP_blk = [RS_blk;
          perf_blk];


function step_filter = makeStepFilter(start_gain, wc, end_gain)
    step_filter = makeweight(start_gain, [wc, (start_gain + end_gain)/2], end_gain);
end