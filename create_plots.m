close all;
% clear all;
init_simulink();

fname = mfilename;
fpath = mfilename('fullpath');
dpath = strrep(fpath, fname, '');
dpath = strcat(dpath, "/figures");
[status, msg, msgID] = mkdir(dpath);

%% Plot constants

external_monitor = 1;

std_dims = [50 800 600 500];
if external_monitor == 1
    std_dims(2) = 1200;
end

%% Step Elevation

t_sim = 15;
noise_power = 0;
sim_nonlinear = 1;


e_timeseries = step_series;
p_timeseries = zero_series;

sim_out = sim("helicopter.slx", t_sim);

fig = figure;
subplot(2, 1, 1);
plot(sim_out.tout, sim_out.r.data(:, 1), "DisplayName", "$e_r$");
hold on;
plot(sim_out.tout, sim_out.x_drone.data(:, 1), "DisplayName", "$e$");
grid on;
title("Step Response Elevation", "Interpreter", "latex");
xlabel("t [s]", "Interpreter","latex");
ylabel("e [m]", "Interpreter", "latex");
legend("Interpreter", "latex", 'Location','southeast');

subplot(2, 1, 2);
plot(sim_out.tout, sim_out.r.data(:, 2), "DisplayName", "$p_r$");
hold on;
plot(sim_out.tout, sim_out.x_drone.data(:, 2), "DisplayName", "$p$");
grid on;
title("Pitch Response", "Interpreter", "latex");
xlabel("t [s]", "Interpreter","latex");
ylabel("p [rad]", "Interpreter", "latex");
legend("Interpreter", "latex", 'Location','southeast');

dims = std_dims;
set(fig, "renderer", "painters", "position", dims, "PaperPositionMode", "auto");
% export_fig("./figures/elevation_step", "-png", "-pdf");
exportgraphics(fig,"./figures/elevation_step.pdf",'ContentType','vector');


e_timeseries = zero_series;
p_timeseries = 0.2*step_series;

sim_out = sim("helicopter.slx", t_sim);

fig = figure;
subplot(2, 1, 1);
plot(sim_out.tout, sim_out.r.data(:, 1), "DisplayName", "$e_r$");
hold on;
plot(sim_out.tout, sim_out.x_drone.data(:, 1), "DisplayName", "$e$");
grid on;
title("Elevation Response", "Interpreter", "latex");
xlabel("t [s]", "Interpreter","latex");
ylabel("e [m]", "Interpreter", "latex");
legend("Interpreter", "latex", 'Location','southeast');

subplot(2, 1, 2);
plot(sim_out.tout, sim_out.r.data(:, 2), "DisplayName", "$p_r$");
hold on;
plot(sim_out.tout, sim_out.x_drone.data(:, 2), "DisplayName", "$p$");
grid on;
title("Step Response Pitch", "Interpreter", "latex");
xlabel("t [s]", "Interpreter","latex");
ylabel("p [rad]", "Interpreter", "latex");
legend("Interpreter", "latex", 'Location','southeast');

dims = std_dims;
set(fig, "renderer", "painters", "position", dims, "PaperPositionMode", "auto");
exportgraphics(fig,"./figures/pitch_step.pdf",'ContentType','vector');
% export_fig("./figures/pitch_step", "-png", "-pdf");

