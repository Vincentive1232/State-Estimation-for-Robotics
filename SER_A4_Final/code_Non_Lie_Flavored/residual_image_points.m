function res = residual_image_points(trans, rots, y_k_j, ...
        rho_i_pj_i, C_c_v, rho_v_c_v, fu, fv, cu, cv, b, y_var)
    
    y_predict_in = predict_image_point(trans, rots, ...
        rho_i_pj_i, C_c_v, rho_v_c_v, fu, fv, cu, cv, b);

    % subtract the predict image points with the actual measurement
    y_error = y_predict_in - y_k_j;

    % weight each error by the std deviation
    y_error = y_error ./ sqrt(y_var);

    % only keep the error terms with reasonable large measurements
    y_error = y_error(y_k_j > 10);
    % change the error into a vector.
    res = y_error(:);
end