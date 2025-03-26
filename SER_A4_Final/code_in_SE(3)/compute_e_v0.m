function res = compute_e_v0(T_prior, T)
    res = skew2v_se3(logm(T_prior/T));
end