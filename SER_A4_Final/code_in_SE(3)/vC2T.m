function res = vC2T(v, C)
    res = zeros(4, 4);

    res(1:3, 1:3) = C;
    res(1:3, 4) = -C*v;
    res(4, 4) = 1;
end