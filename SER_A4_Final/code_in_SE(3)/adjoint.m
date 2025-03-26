function res = adjoint(x)
    res = zeros(6, 6);
    res(1:3, 1:3) = v2skew(x(4:6,1));
    res(1:3, 4:6) = v2skew(x(1:3,1));
    res(4:6, 4:6) = v2skew(x(4:6,1));
end