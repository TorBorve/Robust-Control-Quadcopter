function [A, B, C, D] = linearize_model(f, x, y, u, x0, u0)
    f_dx = jacobian(f, x);
    f_du = jacobian(f, u);
    A = subs(f_dx, [x; u], [x0; u0]);
    B = subs(f_du, [x; u], [x0; u0]);

    y_dx = jacobian(y, x);
    y_du = jacobian(y, u);
    C = subs(y_dx, [x; u], [x0; u0]);
    D = subs(y_du, [x; u], [x0; u0]);
    A = simplify(A);
    B = simplify(B);
    C = simplify(C);
    D = simplify(D);
end