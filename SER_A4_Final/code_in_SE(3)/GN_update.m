function [eps, hat_T_vk_i, hat_P_k, LHS_k] = GN_update(T_c_v, hat_T_vk_i, init_T_vk_i, init_P, varpi_vk_i_vk, Q_k_inv, rho_i_pj_i, y_k_j, y_filter,...
        R_k_inv, dt, fu, fv, cu, cv, b, D, k1, k2)
    %% 1. First we calculate the first input error
    T = hat_T_vk_i(:, :, k1);
    T_prior = init_T_vk_i(:, :, k1);
    e_v0 = compute_e_v0(T_prior, T);

    %% 2. Second we initialize the jacobian
    H_v_0 = zeros(6, (k2-k1+1)*6);
    H_v_0(1:6, 1:6) = eye(6);

    %% 3. We initialize the covariance
    P0_inv = eye(6)/init_P(:, :, k1);

    %% 4. Then we fill in the rest input errors
    T = hat_T_vk_i(:, :, k1:k2-1);
    T2 = hat_T_vk_i(:, :, (k1+1):k2);
    v = varpi_vk_i_vk(:, k1:k2-1);
    d_t = dt(k1+1:k2);
    e_v = zeros(6, (k2-k1));
    for i = 1:1:(k2-k1)
        e_v(:, i) = compute_e_v(T2(:, :, i), T(:, :, i), v(:, i), d_t(i));
    end

    %% 5. compute the Jacobian, i.e. the upper part of H matrix
    F = zeros(6, 6, k2-k1);
    for i = 1:1:(k2-k1)
        F(:, :, i) = compute_F(T2(:, :, i), T(:, :, i), v(:, i), d_t(i));
    end
    H_v = zeros((k2-k1)*6, (k2-k1+1)*6);
    for i = 1:1:size(F, 3)
        H_v((6*(i-1)+1):(6*i), (6*(i-1)+1):(6*i)) = -F(:, :, i);
        H_v((6*(i-1)+1):(6*i), (6*i+1):(6*(i+1))) = eye(6);
    end

    %% 6. compute the covariance of the process noise
    Q_inv_temp = Q_k_inv;
    Q_inv_k1_k2 = zeros(6*(k2-k1), 6*(k2-k1));
    for i = 1:1:(k2-k1)
        Q_inv_k1_k2(6*(i-1)+1:6*i, 6*(i-1)+1:6*i) = Q_inv_temp/(d_t(i)^2);
    end

    %% 7. compute the measurement errors
    p = rho_i_pj_i;
    y = y_k_j(:, k1:k2, :);
    T = zeros(4, 4, k2-k1+1);
    for i = 1:1:(k2-k1+1)
        T(:, :, i) = hat_T_vk_i(:, :, k1+i-1);
    end

    e_y = zeros((k2-k1+1)*4*20, 1);
    H_y = zeros((k2-k1+1)*4*20, (k2-k1+1)*6);
    for i = 1:1:(k2-k1+1)
        for j = 1:1:20
            e_y((i-1)*4*20 + (j-1)*4 + 1:(i-1)*4*20 + j*4, :) = measurement_error(reshape(y(:, i, j), 4, 1), D, T_c_v, p(:, j), reshape(T(:, :, i), 4, 4), ...
                 fu, fv, cu, cv, b);
            H_y((i-1)*4*20 + (j-1)*4 + 1:(i-1)*4*20 + j*4, (i-1)*6+1:6*i) = compute_G(p(:, j), reshape(T(:, :, i), 4, 4), T_c_v, D, fu, fv, b);
        end
    end

    %% 8. compute the covariance of the measurements
    R_inv = zeros((k2-k1+1)*4*20, (k2-k1+1)*4*20);
    for i = 1:1:(k2-k1+1)
        R_inv((i-1)*4*20 + 1:i*4*20, (i-1)*4*20 + 1:i*4*20) = R_k_inv;
    end

    %% 9. Filter out invalid measurements
    % mask = reshape(y_filter(:, k1:k2, :), [], 1);
    mask = zeros(4*(k2-k1+1)*20, 1);
    for i = 1:1:(k2-k1+1)
        for j = 1:1:20
            if y_filter(1, i+k1-1, j) == 0
                mask((i-1)*20*4 + 4*(j-1) + 1:(i-1)*20*4 + 4*j, 1) = 0;
            else
                mask((i-1)*20*4 + 4*(j-1) + 1:(i-1)*20*4 + 4*j, 1) = 1;
            end
        end
    end
    mask = find(mask);
    e_y = e_y(mask);
    H_y = H_y(mask, :);
    R_inv = R_inv(mask, mask);

    %% 10. stack all the vectors
    e_v = reshape(e_v, [], 1);
    e = [e_v0; e_v; e_y];
    H = [H_v_0; H_v; H_y];
    W_inv = blkdiag(P0_inv, Q_inv_k1_k2, R_inv);
    % load 2.mat;
    % 
    % % 将矩阵元素取整到最接近的整数
    % H_r = round(H);
    % H_k = round(H_k);
    % 
    % % 找出不同的坐标
    % different_indices = find(H_r ~= H_k);
    % 
    % % 输出不同的坐标及对应的值
    % for i = 1:length(different_indices)
    %     [row, col] = ind2sub(size(H_r), different_indices(i));
    %     fprintf('不同的坐标: (%d, %d)，值分别为: %d 和 %d\n', row, col, H_r(row, col), H_k(row, col));
    % end

    %% 11. solve the linear system
    LHS = H'*W_inv*H;
    RHS = H'*W_inv*e;
    LHS_inv = eye((k2-k1+1)*6)/LHS;
    L = chol(LHS, "lower");
    d_solve = L\RHS;
    delta_x_solve = L'\d_solve;
    % delta_x_solve = LHS_inv*RHS;
    eps = norm(delta_x_solve);
    %% 12. update each poses
    T = hat_T_vk_i(:, :, k1:k2);
    delta_x_solve = reshape(delta_x_solve, 6, k2-k1+1);

    hat_P_k = init_P;
    for i = 1:1:(k2-k1+1)
        hat_T_vk_i(:, :, i+k1-1) = expm(v2skew_se3(delta_x_solve(:, i))) * T(:, :, i);
        hat_P_k(:, :, i+k1-1) = LHS_inv(6*(i-1)+1:6*i, 6*(i-1)+1:6*i);
    end
    
    LHS_k = LHS;
end