function weight_plot(W_perf_e, W_perf_p, W_perf_u, W_act, W_noise_e, W_noise_p, W_ref_e, W_ref_p, omega)
    hold on;
    addWeightToPlot(W_perf_e, omega, "$W_{perf, e}$");
    addWeightToPlot(W_perf_p, omega, "$W_{perf, p}$");
    addWeightToPlot(W_perf_u, omega, "$W_{perf, u}$");
    addWeightToPlot(W_act, omega, "$W_{act}$");
    addWeightToPlot(W_noise_e, omega, "$W_{noise, e}$");
    addWeightToPlot(W_noise_p, omega, "$W_{noise, p}$");
    addWeightToPlot(W_ref_e, omega, "$W_{ref, e}$");
    addWeightToPlot(W_ref_p, omega, "$W_{ref, p}$");
    
    yscale log;
    xscale log;
    grid on;
    legend("Interpreter", "latex");
end

function addWeightToPlot(weight, freqs, name)
    H = squeeze(freqresp(weight, freqs));
    if name ~= ""
        plot(freqs, abs(H), "DisplayName", name);
    else
        plot(freqs, abs(H));
    end
end