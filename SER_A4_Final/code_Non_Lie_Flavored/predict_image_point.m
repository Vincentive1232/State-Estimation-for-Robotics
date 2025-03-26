function y_predict = predict_image_point(r_i_vk_i, theta_vk_i, ...
    rho_i_pj_i, C_c_v, rho_v_c_v, fu, fv, cu, cv, b)
    
    num_poses = size(r_i_vk_i, 2);
    num_landmarks = size(rho_i_pj_i, 2);
    y_predict = zeros(4, num_poses, 20);
    Landmark_Position_c = zeros(3, num_poses, 20);

    for i = 1:1:num_poses
        C_vk_i = expm(v2skew(-theta_vk_i(:,i)));
        for j = 1:1:num_landmarks
            Landmark_Position_c(:,i,j) = C_c_v*(C_vk_i*(rho_i_pj_i(:,j) - r_i_vk_i(:,i)) - rho_v_c_v);
            y_predict(:,i,j) = measurement_model(Landmark_Position_c(:,i,j), fu, fv, cu, cv, b);
        end
    end
end