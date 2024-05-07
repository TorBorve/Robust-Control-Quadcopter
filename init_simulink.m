%% Load Linearized system
load linearized_system.mat
load alternative_linearized_system.mat
M_inv_alt = inv(M_alt);

%%
noise_power = 1e-5;
sim_nonlinear = 0;

%% Reference Timeseries
t_end = 1000;
t_step = 1;
step_series = timeseries([0; 0; 1; 1], [0; t_step; t_step+0.01; t_end]);
zero_series = timeseries([0; 0], [0; t_end]);

% p_ref = [0; 0];
% p_t =   [0; t_end];
p_ref = [0; 0;  0.2; 0.2; 0.2; 0.2;    0;   0];
p_t =   [0; 2; 2.01; 2.5; 2.51; 10.00; 10.01;t_end];
p_timeseries = timeseries(p_ref, p_t);
% p_timeseries = step_series;

e_ref = [0; 0];
e_t = [0; t_end];
% e_ref = [0;   0;  0.1;  0.1;  -0.3; -0.3];
% e_t =   [0;   2; 2.01;   20; 20.01; t_end];
e_timeseries = timeseries(e_ref, e_t);
e_timeseries = step_series;

s = tf("s");

tau_ref = 0;
ref_filter = 1/(s*tau_ref + 1);
ref_filter = [ref_filter, 0;
              0, ref_filter];

%% Acutator Dynamics
tau_act = 0.2;
G_act = 1/(s*tau_act+1);
G_act = [G_act, 0;
         0, G_act];

tau_act_real = 0.2;
G_act_real = 1/(s*tau_act_real+1);
G_act_real = [G_act_real, 0;
         0, G_act_real];



K_act_f = 1;
K_act_b = 1;

G = ss(A, B, C, D);
G_new = G*G_act;
A_new = G_new.A;
B_new = G_new.B;
C_new = G_new.C;
D_new = G_new.D;

Ctrb_m = ctrb(A_new, B_new);
uncontr = length(A_new) - rank(Ctrb_m);

Obsv_m = obsv(A_new, C_new);
unobsv = length(A_new) - rank(Obsv_m);

if uncontr > 0 || unobsv > 0
    error("System is either unctonrollable or unobservable");
end

%% Controller Parameters

A_int = [zeros(2, 2), eye(2), zeros(2, 4);
        zeros(6, 2), A_new];
B_int = [zeros(2, 2);
        B_new];

A_lqr = A_int;
B_lqr = B_int;

Q = diag([1e2, 1e2, 1e3, 1e1, 1e3, 1e2, 1e0, 1e0]);
R = diag([1e0, 1e0]);
K_lqr = lqr(A_lqr, B_lqr, Q, R);

% Feedforward ensure steady state 0 = ((A-BK+BKf)r
K_lqr_no_int = K_lqr(:, 3:end);
r_u = sym("r_u", "real");
r_e = sym("r_e", "real");
r_p = sym("r_p", "real");
u_f_1 = sym("u_f_1", "real");
u_f_2 = sym("u_f_2", "real");
u_f = [u_f_1; u_f_2];

r = [r_e; r_p; 0; 0; r_u; r_u];
dx = (A_new-B_new*K_lqr_no_int)*r + B_new*u_f;
eq = dx(3) == 0;
r_u_sol = solve(eq, r_u);

dx = subs(dx, r_u, r_u_sol);

eq = dx == 0;
u_f_sol = solve(eq, u_f);
u_f_sol = [u_f_sol.u_f_1; u_f_sol.u_f_2];

eq = subs(eq, u_f, u_f_sol);

K_f_lqr = jacobian(u_f_sol, [r_e; r_p]);
K_f_lqr = double(K_f_lqr);

dx = (A_new - B_new*K_lqr_no_int)*[r_e; r_p; 0; 0; r_u_sol; r_u_sol] + B_new*K_f_lqr*[r_e; r_p];

K_f_lqr = [K_lqr(:, 1:2), K_f_lqr];

%% Observer
A_obs = A_new;
B_obs = B_new;
C_obs = C_new;

r = 15  ;
max_angle = 20*pi/180;
p = zeros(1, 0);
angles = linspace(-max_angle, max_angle, length(A_obs));
for angle=angles
    p = [p, -r*exp(angle*1i)];
end
L = place(A_obs', C_obs', p).';