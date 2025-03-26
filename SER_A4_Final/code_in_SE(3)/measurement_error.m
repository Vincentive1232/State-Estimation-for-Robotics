function res = measurement_error(y, D, T_c_v, p, T, fu, fv, cu, cv, b)
    p = [p; [1]];
    z = D' * T_c_v * T * p;
    g = zeros(4, 1);

    z_1 = 1/z(3,1);

    g(1,1) = z_1*fu*z(1,1) + cu;
    g(2,1) = z_1*fv*z(2,1) + cv;
    g(3,1) = z_1*fu*(z(1,1) - b) + cu;
    g(4,1) = z_1*fv*z(2,1) + cv;

    res = y - g;
end