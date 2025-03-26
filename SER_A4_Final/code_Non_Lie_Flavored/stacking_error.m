function costf_value = stacking_error(state_op, v_vk_vk_i, w_vk_vk_i, y_k_j, ...
                                        C_c_v, rho_v_c_v, rho_i_pj_i, fu, fv, cu, cv, b, t, k1, k2, ...
                                        Q_k, R_k)
    interval = k2-k1+1;
    trans_op = state_op(1:3, :);
    rot_op = state_op(4:6, :);
    [e_v, trans, rot] = input_error(trans_op, rot_op, v_vk_vk_i(:, k1:k2), w_vk_vk_i(:, k1:k2), t(:, k1:k2), k1, k2);

    [e_y, ~, ~] = measurement_error(trans, rot, y_k_j(:, k1:k2, :), C_c_v, rho_v_c_v, rho_i_pj_i, fu, fv, cu, cv, b, k1, k2);

    % e_stacking = [e_v; e_y];

    % try to form a objective function(cost function)
    Q_k_inv = 1\Q_k;
    R_k_inv = 1\R_k;

    costf_value = 0;

    for i = 1:1:interval
        costf_value = costf_value + 0.5*e_v(:, i)'*Q_k_inv*e_v(:, i) + 0.5*e_y(:, i)'*R_k_inv*e_y(:, i);
    end
end