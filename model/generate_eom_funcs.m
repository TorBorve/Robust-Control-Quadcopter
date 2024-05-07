function generate_eom_funcs(eom, gen_cor, F_ext, params, params_val)
    M = eom.M;
    b = eom.b;
    g = eom.g;
    tau = eom.tau;
    hamiltonian = eom.H;

    M = subs(M, struct2cell(params), struct2cell(params_val));
    b = subs(b, struct2cell(params), struct2cell(params_val));
    g = subs(g, struct2cell(params), struct2cell(params_val));
    tau = subs(tau, struct2cell(params), struct2cell(params_val));
    hamiltonian = subs(hamiltonian, struct2cell(params), struct2cell(params_val));

    q = gen_cor.q;
    dq = gen_cor.dq;

    %% Generate matlab functions
    fname = mfilename;
    fpath = mfilename('fullpath');
    dpath = strrep(fpath, fname, '');
    dpath = strcat(dpath, "/generated");
    [status, msg, msgID] = mkdir(dpath);
    
    fprintf('Generating eom scripts... ');
    fprintf('M... ');
    matlabFunction(M, 'vars', {q}, 'file', strcat(dpath,'/M_fun'), 'Optimize', false);
    fprintf('g... ');
    matlabFunction(g, 'vars', {q}, 'file', strcat(dpath,'/g_fun'), 'Optimize', false);
    fprintf('b... ');
    matlabFunction(b, 'vars', {q, dq}, 'file', strcat(dpath,'/b_fun'), 'Optimize', false);
    fprintf('tau... ');
    matlabFunction(tau, 'vars', {q, F_ext}, 'file', strcat(dpath,'/tau_fun'), 'Optimize', false);
    fprintf('hamiltonian... ');
    matlabFunction(hamiltonian, 'vars', {q, dq}, 'file', strcat(dpath,'/hamiltonian_fun'), 'Optimize', false);
    fprintf("\n");
end