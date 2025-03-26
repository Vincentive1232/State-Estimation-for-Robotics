function res = compute_pixel_residual(param_vec, y_k_j, ...
    rho_i_pj_i, C_c_v, rho_v_c_v, fu, fv, cu, cv, b, y_var)

    trans = param_vec(1:3, :);
    rots = param_vec(4:6, :);

    res = residual_image_points(trans, rots, y_k_j, ...
        rho_i_pj_i, C_c_v, rho_v_c_v, fu, fv, cu, cv, b, y_var);
end