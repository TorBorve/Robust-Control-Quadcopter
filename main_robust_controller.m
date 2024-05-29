
% Update paths and working directory
try
    init_workspace();
    fprintf("Disclaimer: This MATLAB script has only been validated for MATLAB 2023b\n");
    fprintf("If you are using another MATLAB version it will likely not work.\n");
    fprintf("If the script crashes then a workspace file will be loaded with all the plots contained\n");
    error("Test");
    
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
catch
    warning("Script crashed. Workspace will be loaded from file instead\n");
    % All variables saved such that the plots can be created even though the
    % simulink version might be wrong for example.
    try
        load("plot_workspace.mat");
    catch 
       warning("Could not load workspace file\n"); 
    end
end
