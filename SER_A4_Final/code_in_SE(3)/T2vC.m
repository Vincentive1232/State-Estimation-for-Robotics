function [v, C] = T2vC(T)
    v = T(1:3, 4);
    C = T(1:3, 1:3);
    C_transpose = permute(C, [2, 1, 3]);
    v = -C_transpose*v;
end