function write_system_to_file(A, B, C, D, x0, u0, params, params_val)
    A = double(subs(A, struct2cell(params), struct2cell(params_val)));
    B = double(subs(B, struct2cell(params), struct2cell(params_val)));
    C = double(subs(C, struct2cell(params), struct2cell(params_val)));
    D = double(subs(D, struct2cell(params), struct2cell(params_val)));
    x0 = double(subs(x0, struct2cell(params), struct2cell(params_val)));
    u0 = double(subs(u0, struct2cell(params), struct2cell(params_val)));
    
    fname = mfilename;
    fpath = mfilename('fullpath');
    dpath = strrep(fpath, fname, '');
    dpath = strcat(dpath, "/generated");
    [status, msg, msgID] = mkdir(dpath);
    save(strcat(dpath, "/linearized_system.mat"), "A", "B", "C", "D", "x0", "u0");
end