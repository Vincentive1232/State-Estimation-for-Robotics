function res = skew2v_se3(M)
    res = zeros(6, 1);
    res(4:6,:) = skew2v(M(1:3, 1:3));
    res(1:3,:) = M(1:3, 4);
end