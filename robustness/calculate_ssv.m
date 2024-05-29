function [muNP, muRS, muRP, muinfoRP] = calculate_ssv(N, omega, Iz, Ie, Iv, Iw, RS_blk, RP_blk)
    N_w = frd(N, omega);

    muRS = mussv(N_w(Iz, Iv), RS_blk);
    muRS = muRS(1, 1, :);

    muNP = svd(N_w(Ie, Iw));
    muNP = muNP(1, :, :);
    % muNP = nan;
    % muRS = nan;

    [muRP, muinfoRP] = mussv(N_w, RP_blk);
    muRP = muRP(1, 1, :);
end