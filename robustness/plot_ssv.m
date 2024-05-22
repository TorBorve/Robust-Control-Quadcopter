function [muinfoRP, gammaRP] = plot_ssv(N, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk, whichToPlot)
    if nargin < 9
        whichToPlot = [1, 1, 1];
    else
        assert(size(whichToPlot, 2) == 3);
    end
    N_w = frd(N, omega);
    hold on;
    grid on;
    legends = [];
    if whichToPlot(1) == 1
        muRS = mussv(N_w(Iz, Iv), RS_blk);
        u_muRS = muRS(1, 1, :);
        plot(u_muRS);
        legends = [legends, "RS"];
    end
    if whichToPlot(2) == 1
        muNP = svd(N_w(Ie, Iw));
        u_muNP = muNP(1, :, :);
        plot(u_muNP);
        legends = [legends, "NP"];
    end
    if whichToPlot(3) == 1
        [muRP, muinfoRP] = mussv(N_w, RP_blk);
        u_muRP = muRP(1, 1, :);
        plot(u_muRP);
        legends = [legends, "RP"];
    end
    
    legend(legends, "Interpreter", "latex");
    xscale log;

    worst_muRP = max(u_muRP.ResponseData(1, 1, :));
    gammaRP = worst_muRP;
end