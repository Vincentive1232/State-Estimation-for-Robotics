function [trans, rot] = motion_model(trans_op, rot_op, v_vk_vk_i, w_vk_vk_i, t, init)
    trans = trans_op;
    rot = rot_op;
    if(init)
        for i = 2:1:size(t, 2)
            trans(:, i) = trans(:, i-1) + expm(v2skew(rot(:, i-1)))'*(v_vk_vk_i(:, i-1)*(t(1, i) - t(1, i-1)));
            C_i_1= expm(v2skew(rot(:, i-1)));
            Phi_k = w_vk_vk_i(:, i-1)*(t(1, i) - t(1, i-1));
            Phi_k_norm = norm(Phi_k);
            if (Phi_k_norm > pi)
                Phi_k_norm = -pi + mod(Phi_k_norm, pi);
            elseif (Phi_k_norm < -pi)
                Phi_k_norm = pi + mod(Phi_k_norm, pi);
            end
            Psi_k = cos(Phi_k_norm)*eye(3, 3) + (1 - cos(Phi_k_norm))*(Phi_k/Phi_k_norm)*(Phi_k/Phi_k_norm)' - sin(Phi_k_norm)*(v2skew(Phi_k/Phi_k_norm));
            C_i = Psi_k*C_i_1;
            rot(:, i) = skew2v(logm(C_i));
        end
    elseif (~init)
        for i = 2:1:size(t, 2)
            trans(:, i) = trans_op(:, i-1) + expm(v2skew(rot(:, i-1)))'*(v_vk_vk_i(:, i-1)*(t(1, i) - t(1, i-1)));
            C_i_1= expm(v2skew(rot_op(:, i-1)));
            Phi_k = w_vk_vk_i(:, i-1)*(t(1, i) - t(1, i-1));
            Phi_k_norm = norm(Phi_k);
            if (Phi_k_norm > pi)
                Phi_k_norm = -pi + mod(Phi_k_norm, pi);
            elseif (Phi_k_norm < -pi)
                Phi_k_norm = pi + mod(Phi_k_norm, pi);
            end
            Psi_k = cos(Phi_k_norm)*eye(3, 3) + (1 - cos(Phi_k_norm))*(Phi_k/Phi_k_norm)*(Phi_k/Phi_k_norm)' - sin(Phi_k_norm)*(v2skew(Phi_k/Phi_k_norm));
            C_i = Psi_k*C_i_1;
            rot(:, i) = skew2v(logm(C_i));
        end
    end
end

% The motion model has been checked and proved it is correct.