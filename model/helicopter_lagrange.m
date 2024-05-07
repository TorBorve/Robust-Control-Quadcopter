function [eom] = helicopter_lagrange(q, dq, F_ext, params)
    
    r_Ib = [0; 0; q(1)];
    C_Ib = [cos(q(2)), 0, -sin(q(2));
           0, 1, 0;
           sin(q(2)), 0, cos(q(2))];
    

    T_Ib = [C_Ib, r_Ib;
            0, 0, 0, 1];

    I_g = [0; 0; -params.g];

    r_b = T_Ib*[0; 0; 0; 1];
    r_b = r_b(1:3);
    dr_b = jacobian(r_b, q)*dq;

    r_mf = T_Ib*[params.l_p; 0; 0; 1];
    r_mf = r_mf(1:3);
    dr_mf = jacobian(r_mf, q)*dq;

    r_mb = T_Ib*[-params.l_p; 0; 0; 1];
    r_mb = r_mb(1:3);
    dr_mb = jacobian(r_mb, q)*dq;

    potEnergy = -params.m_b*r_b'*I_g -params.m_m*(r_mb'*I_g + r_mf'*I_g);
    kinEnergy = 1/2*(params.m_b*(dr_b'*dr_b) + params.m_m*((dr_mb'*dr_mb) + (dr_mf'*dr_mf)));

    kinEnergy = simplify(kinEnergy);
    potEnergy = simplify(potEnergy);

    L = simplify(kinEnergy - potEnergy);
    H = simplify(kinEnergy + potEnergy);
    % dt(dL/ddq) - dL/dq = tau
    dL_dq = simplify(jacobian(L, q)');
    dL_ddq = simplify(jacobian(L, dq)');
    
    M = jacobian(dL_ddq, dq)';
    M = simplify(M);
    b = jacobian(dL_ddq, q)'*dq - jacobian(kinEnergy, q)';
    b = simplify(b);
    g = jacobian(potEnergy, q)';
    g = simplify(g);

    %% External Forces

    r_ext_f = r_mf;
    I_Jp_ext_f = jacobian(r_ext_f, q);

    r_ext_b = r_mb;
    I_Jp_ext_b = jacobian(r_ext_b, q);

    I_F_f = T_Ib(1:3, 1:3)*[0; 0; F_ext(1)];
    I_F_b = T_Ib(1:3, 1:3)*[0; 0; F_ext(2)];

    tau = I_Jp_ext_f'*I_F_f + I_Jp_ext_b'*I_F_b;
    tau = simplify(tau);



    eom.g = g;
    eom.M = M;
    eom.b = b;
    eom.H = H;
    eom.tau = tau;
end