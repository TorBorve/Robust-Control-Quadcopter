
% Update paths and working directory
init_workspace();
fprintf("Disclaimer: This MATLAB script has only been validated for MATLAB 2023b\n")

% Generate Linear model of nonlinear system and save to file
fprintf("\n******************************\n");
fprintf("\tGenerate model\n");
fprintf("******************************\n");
main_model();
clear all;

% Get model for LQR-plant and save to file
fprintf("\n******************************\n");
fprintf("\tCompute LQR\n");
fprintf("******************************\n");
save_LQR_Controller();
clear all;

% Compute mu-controller
fprintf("\n******************************\n");
fprintf("\tCompute mu-controller\n");
fprintf("******************************\n");
mu_controller();
close all; clear all;

% Generate plots
fprintf("\n******************************\n");
fprintf("\tCreate plots for final report\n");
fprintf("******************************\n");
create_robust_plots();
