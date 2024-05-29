close all;
% clear all;

load linearized_system.mat
init_robust_simulink();
K_act_f = 1;
K_act_b = 1;
noise_power = 0;

% tau_act = 0.2;
s = tf("s");
G_act = 1/(s*tau_act+1);
G_act = [G_act, 0;
         0, G_act];
load("K_inf_controller.mat");
load("K_lqr_controller.mat");
load("K_lqr_controller_retuned.mat");

% K_sim = K_lqr_ss;
% K_sim = K_lqr_ss_retuned;

%% References
t_end = 1000;
t_step = 1;
step_series = timeseries([0; 0; 1; 1], [0; t_step; t_step+0.01; t_end]);
zero_series = timeseries([0; 0], [0; t_end]);

% p_timeseries = zero_series;
p_timeseries = -pi/16*step_series;
% e_timeseries = step_series;
e_timeseries = zero_series;

t_sim = 20;
K_sim = K_lqr_ss;
sim_out = sim("nonlinear_closed_loop.slx", t_sim);

fig = figure;
subplot(2, 1, 1);
plot(sim_out.tout, sim_out.r.data(:, 1), "DisplayName", "$e_r$");
hold on;
plot(sim_out.tout, sim_out.x.data(:, 1), "DisplayName", "$INF$");
grid on;

subplot(2, 1, 2);
plot(sim_out.tout, sim_out.r.data(:, 2), "DisplayName", "$p_r$");
hold on;
plot(sim_out.tout, sim_out.x.data(:, 2), "DisplayName", "$INF$");
grid on;


K_sim = K_lqr_ss;
sim_out = sim("nonlinear_closed_loop.slx", t_sim);

subplot(2, 1, 1);
% plot(sim_out.tout, sim_out.r.data(:, 1), "DisplayName", "$e_r$");
hold on;
plot(sim_out.tout, sim_out.x.data(:, 1), "DisplayName", "$LQR$");
grid on;

subplot(2, 1, 2);
% plot(sim_out.tout, sim_out.r.data(:, 2), "DisplayName", "$p_r$");
hold on;
plot(sim_out.tout, sim_out.x.data(:, 2), "DisplayName", "$LQR$");


K_sim = K_lqr_ss_retuned;
sim_out = sim("nonlinear_closed_loop.slx", t_sim);

subplot(2, 1, 1);
% plot(sim_out.tout, sim_out.r.data(:, 1), "DisplayName", "$e_r$");
hold on;
plot(sim_out.tout, sim_out.x.data(:, 1), "DisplayName", "$RT$");
grid on;
title("Step Response Elevation", "Interpreter", "latex");
xlabel("t [s]", "Interpreter","latex");
ylabel("e [m]", "Interpreter", "latex");
legend("Interpreter", "latex", 'Location','southeast');

subplot(2, 1, 2);
% plot(sim_out.tout, sim_out.r.data(:, 2), "DisplayName", "$p_r$");
hold on;
plot(sim_out.tout, sim_out.x.data(:, 2), "DisplayName", "$RT$");
grid on;
title("Pitch Response", "Interpreter", "latex");
xlabel("t [s]", "Interpreter","latex");
ylabel("p [rad]", "Interpreter", "latex");
legend("Interpreter", "latex", 'Location','southeast');



