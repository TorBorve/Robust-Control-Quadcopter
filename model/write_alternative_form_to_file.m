function write_alternative_form_to_file(M, E, B, params, params_val)
    % M ddq = Eq+Bu
    M_alt = double(subs(M, struct2cell(params), struct2cell(params_val)));
    E_alt = double(subs(E, struct2cell(params), struct2cell(params_val)));
    B_alt = double(subs(B, struct2cell(params), struct2cell(params_val)));
    
    fname = mfilename;
    fpath = mfilename('fullpath');
    dpath = strrep(fpath, fname, '');
    dpath = strcat(dpath, "/generated");
    [status, msg, msgID] = mkdir(dpath);
    save(strcat(dpath, "/alternative_linearized_system.mat"), "M_alt", "E_alt", "B_alt");
end