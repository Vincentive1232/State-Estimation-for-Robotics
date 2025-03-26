function [e_v, trans_predict, rot_predict] = input_error(trans_op, rot_op, v_vk_vk_i, w_vk_vk_i, t, k1, k2)
    interval = (k2-k1+1);
    e_v = zeros(6, interval);

    [trans_predict, rot_predict] = motion_model(trans_op, rot_op, v_vk_vk_i, w_vk_vk_i, t);

    for i = 1:1:interval
        C_i_operating = expm(v2skew(rot_op(:, i)));
        C_i_predict = expm(v2skew(rot_predict(:, i)));
        e_v(4:6, i) = skew2v(logm(C_i_predict/C_i_operating));
    end

    e_v(1:3, :) = trans_predict - trans_op;
end