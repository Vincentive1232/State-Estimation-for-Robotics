function res = compute_e_v(T2, T, v, dt)
    T_temp = motion_model(T, v, dt);
    res = skew2v_se3(logm(T_temp/T2));
end