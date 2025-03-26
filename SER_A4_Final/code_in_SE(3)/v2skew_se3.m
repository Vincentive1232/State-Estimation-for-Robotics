function res = v2skew_se3(v)
    res = zeros(4, 4);

    % temp = v2skew(Theta);
    res(1:3, 1:3) = v2skew(v(4:6, 1));
    res(1:3, 4) = v(1:3, 1);
end