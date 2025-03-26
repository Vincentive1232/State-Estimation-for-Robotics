function res = compute_G(p, T, T_c_v, D, fu, fv, b)
    p = [p; [1]];
    z = D' * T_c_v * T * p;
    dgdz = zeros(4, 3);

    dgdz(1, 1) = fu/z(3, 1);
    dgdz(1, 3) = -fu*z(1, 1)/(z(3, 1)^2);
    dgdz(2, 2) = fv/z(3, 1);
    dgdz(2, 3) = -fv*z(2, 1)/(z(3, 1)^2);
    dgdz(3, 1) = fu/z(3, 1);
    dgdz(3, 3) = -fu*(z(1, 1) - b)/(z(3, 1)^2);
    dgdz(4, 2) = fv/z(3, 1);
    dgdz(4, 3) = -fv*z(2, 1)/(z(3, 1)^2);

    dzdx = D'*T_c_v*Circle_dot(T*p);

    res = dgdz*dzdx;
end