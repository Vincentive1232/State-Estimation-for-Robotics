function res = v2skew_se3(translation, Theta)
    res = zeros(4, 4);

    temp = v2skew(Theta);
    res(1:3, 1:3) = temp;
    res(1:3, 4) = translation;
end