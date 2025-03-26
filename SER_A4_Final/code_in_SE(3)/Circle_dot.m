function res = Circle_dot(Tp)
    res = zeros(4, 6);
    res(1:3, 1:3) = Tp(4,1)*eye(3,3);
    res(1:3, 4:6) = -v2skew(Tp(1:3,1));
end