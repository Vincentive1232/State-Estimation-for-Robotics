function res = compute_F(T2, T, v, dt)
    res = Adjoint_SE3(T2/T);
end