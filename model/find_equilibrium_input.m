function [u0] = find_equilibrium_input(x, x0, f, u)
    eq = f == 0;
    eq = subs(eq, x, x0);
    u_sol = solve(eq, u);
    u0 = [u_sol.F_f; u_sol.F_r];
end