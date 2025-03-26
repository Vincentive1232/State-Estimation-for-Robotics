function res = residual_with_IMU_stereo(param_vec, t, v_vk_vk_i, w_vk_vk_i, ...
    Q_k, y_k_j, rho_i_pj_i, C_c_v, rho_v_c_v, fu, fv, cu, cv, b, y_var)
    
    trans_op = param_vec(1:3, :);
    rots_op = param_vec(4:6, :);
    [trans, rots] = motion_model(trans_op, rots_op, v_vk_vk_i, w_vk_vk_i, t, 0);

    e_v = zeros(6, size(trans_op, 2));
    for i = 1:1:size(trans_op, 2)
        e_v(1:3, i) = trans(:, i) - trans_op(:, i);
        C_rots_op = expm(v2skew(rots_op(:, i)));
        C_rots = expm(v2skew(rots(:, i)));
        e_v(4:6, i) = skew2v(logm(C_rots/C_rots_op));
        e_v(:, i) = e_v(:, i)'/Q_k;
    end
    e_v = e_v(:);
    

    param_vec = [trans; rots];

    e_y = compute_pixel_residual(param_vec, y_k_j, ...
        rho_i_pj_i, C_c_v, rho_v_c_v, fu, fv, cu, cv, b, y_var);

    res = [e_v; e_y];
end