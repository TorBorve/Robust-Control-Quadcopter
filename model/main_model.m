close all; clear all; clc;

e = sym("e", "real");
p = sym("p", "real");
q = [e; p];

de = sym("de", "real");
dp = sym("dp", "real");
dq =  [de; dp];

F_1 = sym("F_f", "real");
F_2 = sym("F_r", "real");
F_ext = [F_1; F_2];

params.g = sym("g", "real");
params.l_p = sym("l", "real");
params.m_m = sym("m_m", "real");
params.m_b = sym("m_b", "real");

params_val.g = 9.81; % gravitational constant [m/s^2]
params_val.l_p = 0.2; % distance pitch axis to motor [m]
params_val.m_b = 2; % Body mass [kg]
params_val.m_m = 0.1; % Motor mass [kg]

gen_cor.q = q;
gen_cor.dq = dq;

fprintf("Compute Equations of Motion using Lagrange\n");
eom = helicopter_lagrange(q, dq, F_ext, params);
generate_eom_funcs(eom, gen_cor, F_ext, params, params_val);

fprintf("Compute x_dot = f(x, u)\n");
x = [q; dq];
y = q;
u = F_ext;
f = [dq;
    eom.M\(-eom.b-eom.g+eom.tau)];
f = simplify(f);

fprintf("Linearize system\n");
q0 = [0; pi/4];
dq0 = zeros(2, 1);
x0 = [q0; dq0];
u0 = find_equilibrium_input(x, x0, f, u);
[A, B, C, D] = linearize_model(f, x, y, u, x0, u0);
write_system_to_file(A, B, C, D, x0, u0, params, params_val);

fprintf("Create alternative form linearized system\n");
% Alternative form ddq = M\(Eq + tau_lin*u)
M_lin = subs(eom.M, q, q0);
E = A(size(q, 1)+1:end, 1:size(q, 1));
E = M_lin*E;
tau_lin = jacobian(eom.tau, F_ext);
tau_lin = subs(tau_lin, [x; u], [x0; u0]);

write_alternative_form_to_file(M_lin, E, tau_lin, params, params_val);




