function [e_y, y_hat, p_ob] = measurement_error(trans, rot, y_k_j, C_c_v, rho_v_c_v, rho_i_pj_i, fu, fv, cu, cv, b, k1, k2)
    e_y = zeros(4, (k2-k1+1), 20);
    p_ob = zeros(3, (k2-k1+1), 20);
    y_hat = zeros(4, (k2-k1+1), 20);

    for i = 1:1:(k2-k1+1)
        C_vk_i = expm(v2skew(rot(:, i)));
        for j = 1:1:20
            p_ob(:, i, j) = C_c_v*(C_vk_i*(rho_i_pj_i(:, j) - trans(:, i)) - rho_v_c_v);
            y_hat(:, i, j) = measurement_model(p_ob(:, i, j), fu, fv, cu, cv, b);
            if (y_hat(1, i, j) ~= -1 && y_hat(2, i, j) ~= -1 && y_hat(3, i, j) ~= -1)
                e_y(:, i, j) = y_hat(:, i, j) - y_k_j(:, i, j);
            end
        end
    end
    
    e_y = reshape(e_y, 4*20, (k2-k1+1));
end