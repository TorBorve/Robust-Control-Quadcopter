function weight_plot(W_perf_e, W_perf_p, W_perf_u, W_act, p_Jp, W_noise_e, W_noise_p, W_ref_e, W_ref_p, omega)
    hold on;
    addWeightToPlot(W_perf_e, omega, "$W_{perf, e}$");
    addWeightToPlot(W_perf_p, omega, "$W_{perf, p}$");
    addWeightToPlot(W_perf_u, omega, "$W_{perf, u}$");
    addWeightToPlot(W_act, omega, "$p_{u_i}$");
    addWeightToPlot(p_Jp, omega, "$p_{J_p}$");
    addWeightToPlot(W_noise_e, omega, "$W_{e}$");
    addWeightToPlot(W_noise_p, omega, "$W_{p}$");
    addWeightToPlot(W_ref_e, omega, "$W_{e_r}$");
    addWeightToPlot(W_ref_p, omega, "$W_{p_r}$");
    
    yscale log;
    xscale log;
    grid on;
    try
        legend_size = evalin("caller", "legend_size");
        title_size = evalin("caller", "title_size");
        legend_size = legend_size + 2;
        legend("Interpreter", "latex", "Location","eastoutside", "FontSize", legend_size);
        title("Uncertainty and Performance Weights", "Interpreter", "latex", "FontSize", title_size);
        
    catch
        egend("Interpreter", "latex", "Location","eastoutside", "FontSize", 12);
        title("Uncertainty and Performance Weights", "Interpreter", "latex");
    end
    
    xlabel("Frequency [rad/s]", "Interpreter", "latex");
    ylabel("$Magnitude$", "Interpreter", "latex");
    

end

function addWeightToPlot(weight, freqs, name)
    H = squeeze(freqresp(weight, freqs));
    if name ~= ""
        plot(freqs, abs(H), "DisplayName", name);
    else
        plot(freqs, abs(H));
    end
end