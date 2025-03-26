function res = Adjoint_SE3(T)
    res = zeros(6, 6);

    res(1:3, 1:3) = T(1:3, 1:3);
    res(4:6, 4:6) = T(1:3, 1:3);
    res(1:3, 4:6) = v2skew(T(1:3, 4))*T(1:3, 1:3);
end