%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  This script is a BatchGaussianNewton pose estimation of the position of
%  a Robot in Lie algebra and Lie Group
%  Author: Jiaming Li
%  Created on: Feburary 1, 2024
%  Copyright (c) 2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 
% load dataset
clc
clear
load dataset3.mat

%% analyze the data and discover some hidden relations
k1 = 1000;
k2 = 1100;
interval = numel(t(:, k1:k2));
num_landmarks = size(rho_i_pj_i, 2);

% First we plot the measurement of the IMU, namely the translation velocity
% and rotation velocity
figure(1);
plot(w_vk_vk_i');
title('Rotation velocity')
xlabel('t');
ylabel('[rad/s]');
legend('X', 'Y', 'Z');
grid on;

figure(2);
plot(v_vk_vk_i');
title('Translation velocity')
xlabel('t');
ylabel(['[m/s]']);
legend('X', 'Y', 'Z');
grid on;

%% 
% Then we try to visualize the image points, which are given in the
% dataset(meansurement)
figure(3);
for k = k1:1:k2
    plot(squeeze(y_k_j(1,k,:)), squeeze(y_k_j(2,k,:)), 'ro');
    axis([0, 640, 0, 480]);
    grid on;
    hold on;
    plot(squeeze(y_k_j(3,k,:)), squeeze(y_k_j(4,k,:)), 'bo');

    % plot a line to link the left and right image features.
    line([squeeze(y_k_j(1,k,:)), squeeze(y_k_j(3,k,:))]', ...
        [squeeze(y_k_j(2,k,:)), squeeze(y_k_j(4,k,:))]', 'Color', 'g');
    hold off;
    title(['k =' num2str(k)]);
    drawnow;
end

disp('The covariance of the measurements(image points, [pixel^2]) is:');
y_var

y_std = sqrt(y_var);
disp('The std deviation of measurements(image points, [pixel]) is:');
y_std

%% here we plot the difference of between 2 timesteps, since the differences are not the same.
figure(4);
plot(diff(t), 'Color','blue');
xlabel('Timestep');
ylabel('Actual difference between 2 timesteps(t)');
title('Time differences');
grid on;

%% Ground truth poses
C_vk_i = zeros(3,3, interval);

for i = 1:1:interval
    C_vk_i(:, :, i) = expm(v2skew(-theta_vk_i(:, (i+k1-1))));
end
r_i_vk_i;

%% Plot the features shown on the groud. 
% (A little bit weird since the z coordinates of some features is minus zero) 
figure(5);
plot3(rho_i_pj_i(1, :), rho_i_pj_i(2, :), rho_i_pj_i(3, :), 'r.', 'MarkerSize', 10);
% axis vis3d;
% daspect([1, 1, 1]);  % make the ratio be equal
xlabel('X[m]');
ylabel('Y[m]');
zlabel('Z[m]');
zlim([-0.1, 1])
grid on;
title('location of the 3D landmarks');

%% Analyze the noises

% process noise --- Q_k
format short e
Q_k_temp = diag([v_var; w_var]);
disp('The covariance matrix of the imu measurement is: ');
Q_k_temp

% Since our approximate timestep is T=0.1, we should adjust out Q_k with
% that interval.
Q_k = Q_k_temp*(0.1)^2;
disp('The adjusted covariance matrix of the imu measurement is: ');
Q_k

% measurement noise --- R_k
format short
R_k = diag(y_var);
disp('The covariance matrix of the stereo camera measurement is: ');
R_k

%% check the number of visible landmarks at every timestep

% plot the number of visible landmarks at every timestep
visible_landmarks = sum(squeeze(y_k_j(1, :, :)) >= 0, 2);

figure(6);
stem(t, visible_landmarks, 'r.');
hold on;

index = (visible_landmarks >= 3);
stem(t(index), visible_landmarks(index), 'g.');
xlabel('t[s]')
ylabel('Number of visible landmarks time t');

% with timestep as the x axis
figure(7);
stem(t, visible_landmarks, 'r.');
hold on;

index = (visible_landmarks >= 3);
stem(find(index), visible_landmarks(index), 'g.');
xlabel('timestep')
ylabel('Number of visible landmarks at every timestep');

% recover Fig. 3.10 in the assignment document.
bin_centers = (0:num_landmarks);
r = squeeze(y_k_j(1,:,:)) >= 0;
figure(8);
histogram(sum(r'>0), bin_centers, 'FaceAlpha', 1, 'FaceColor', 'b');
xlim([bin_centers(1), bin_centers(end)]);
xlabel('Number of visible landmarks');
ylabel('Number of timesteps');

% here is the end of the data anaysis, we have gone through all the data
% and understood their structure
%% Here we start to implement and check the motion model and the measurment model

% 1. first we check the point projection, namely the measurment model using
%    the groundtruth poses.

% first we revaluate k1 and k2(just for convenience)
k1 = 1000;
k2 = 1050;

y_predict = predict_image_point(r_i_vk_i(:, k1:k2), theta_vk_i(:, k1:k2), ...
    rho_i_pj_i, C_c_v, rho_v_c_v, fu, fv, cu, cv, b);

figure(9);
visualize_image_points(y_k_j(:, k1:k2, :), y_predict, k1, k2);

%% Optimize poses using stereo constraints only
disp('-------------------------------------------------------------------------------');
disp('Optimize poses using stereo camera measurements only')
mode = 1;

% here we define the cost function(the quadratic distance between the
% prediction and the groundtruth).
f_res_pixels = @(x) compute_pixel_residual(x, y_k_j(:, k1:k2, :), ...
    rho_i_pj_i, C_c_v, rho_v_c_v, fu, fv, cu, cv, b, y_var);

% Set the initial value of our states
param_vec = [r_i_vk_i(:, k1:k2); theta_vk_i(:, k1:k2)];

% optimize
options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', ...
    'Display', 'iter', 'MaxIterations', 10);

tic
[param_vec_best, resnorm, residual, ~, ~, ~, jacobian] = lsqnonlin(f_res_pixels, param_vec, [], [], options);
toc

% Error and performance evaluation.
% compare the optimized results and the original ones
figure(10);
visualize_image_points(y_k_j(:, k1:k2, :), y_predict, k1, k2);

best_predict = predict_image_point(param_vec_best(1:3, :), param_vec_best(4:6, :), ...
    rho_i_pj_i, C_c_v, rho_v_c_v, fu, fv, cu, cv, b);
figure(11);
visualize_image_points(y_k_j(:, k1:k2, :), best_predict, k1, k2);

% compute the Root Mean Square Error
error_RMS = sqrt(resnorm / numel(residual));
disp(['RMS Error per entry of our estimation = ' num2str(error_RMS)]);
res_pixels = f_res_pixels(param_vec_best);
err_pix = norm(res_pixels, 1) / numel(res_pixels);
disp(['L1 error per coordinate of our estimation = ' num2str(err_pix) '[pixels]']);
err_pix = sqrt(res_pixels'*res_pixels / numel(res_pixels));
disp(['RMS error per coordinate of our estimation = ' num2str(err_pix) '[pixels]']);

%% Optimize poses using both IMU data and stereo camera measurements
disp('-------------------------------------------------------------------------------');
disp('Optimize poses using both IMU data and stereo camera measurements')

% here we define the cost function(the quadratic distance between the
% prediction and the groundtruth).
f_res_combined = @(x) residual_with_IMU_stereo(x, t(:, k1:k2), v_vk_vk_i(:, k1:k2), w_vk_vk_i(:, k1:k2), ...
    Q_k_temp, y_k_j(:, k1:k2, :), rho_i_pj_i, C_c_v, rho_v_c_v, fu, fv, cu, cv, b, y_var);

% Set the initial value of our states
param_vec = [r_i_vk_i(:, k1:k2); theta_vk_i(:, k1:k2)];

% optimize
options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', ...
    'Display', 'iter', 'MaxIterations', 10);

tic
[param_vec_best, resnorm, residual, ~, ~, ~, jacobian] = lsqnonlin(f_res_combined, param_vec, [], [], options);
toc

% Error and performance evaluation.
% compare the optimized results and the original ones
figure(12);
visualize_image_points(y_k_j(:, k1:k2, :), y_predict, k1, k2);

best_predict = predict_image_point(param_vec_best(1:3, :), param_vec_best(4:6, :), ...
    rho_i_pj_i, C_c_v, rho_v_c_v, fu, fv, cu, cv, b);
figure(13);
visualize_image_points(y_k_j(:, k1:k2, :), best_predict, k1, k2);

% compute the Root Mean Square Error
error_RMS = sqrt(resnorm / numel(residual));
disp(['RMS Error per entry of our estimation = ' num2str(error_RMS)]);
res_pixels = f_res_pixels(param_vec_best);
err_pix = norm(res_pixels, 1) / numel(res_pixels);
disp(['L1 error per coordinate of our estimation = ' num2str(err_pix) '[pixels]']);
err_pix = sqrt(res_pixels'*res_pixels / numel(res_pixels));
disp(['RMS error per coordinate of our estimation = ' num2str(err_pix) '[pixels]']);



%% Optimize poses using both IMU data and stereo camera measurements and initialize with dead reckoning
disp(' ')
disp('-------------------------------------------------------------------------------');
disp('Initialize using dead reckoning')

k1 = 1000;
k2 = 1050;

% Check the dead reckoning (motion model)
trans_init = zeros(3, k2-k1+1);
rot_init = zeros(3, k2-k1+1);

trans_init(:, 1) = r_i_vk_i(:, k1);
rot_init(:, 1) = theta_vk_i(:, k1);

[trans_init_op, rot_init_op] = motion_model(trans_init, rot_init, v_vk_vk_i(:, k1:k2), w_vk_vk_i(:, k1:k2), t(:, k1:k2), 1);

% plot the whole trajectory of the system
figure(14);
x = trans_init_op(1, :);
y = trans_init_op(2, :);
z = trans_init_op(3, :);

% Draw a line linking the states
plot3(x, y, z, '-',  'LineWidth', 1);
hold on;

x_feature = rho_i_pj_i(1,:)';
y_feature = rho_i_pj_i(2,:)';
z_feature = rho_i_pj_i(3,:)';

% Draw the landmarks
scatter3(x_feature, y_feature, z_feature, 10, 'r', 'filled');

xlabel('X');
ylabel('Y');
zlabel('Z');
title('Sensor Head Path using only IMU Data (similar to Fig 3.9)');
grid on;

% plot the dead reckoning and the groundtruth
figure(15);
plot(r_i_vk_i') % plot the groundtruth
hold on;
plot(k1:k2, trans_init_op', '*', 'MarkerSize', 2);
title('Translation: ground truth using dead reckoning')
xlabel('Timestep')
ylabel('Translation [m]');
xlim([k1-20, k2+20]);
grid on;

% compute the error between the dead reckoning and the groundtruth
trans_err = r_i_vk_i(:, k1:k2) - trans_init_op;
Id3 = eye(3);
rot_err = zeros(3, k2-k1+1);
C_vk_i = zeros(3,3, k2-k1+1);
C_init_op = zeros(3,3, k2-k1+1);
for i = 1:1:(k2-k1+1)
    C_vk_i(:, :, i) = expm(v2skew(-theta_vk_i(:, (i+k1-1))));
    C_init_op(:, :, i) = expm(v2skew(rot_init_op(:, i)));
    rot_err(:, i) = skew2v(Id3 -   C_vk_i(:, :, i)*C_init_op(:, :, i)');
end

% See how the translationa and rotational estimates degrade with dead
% reckoning
figure(16);
plot(sum(abs(trans_err), 1), 'k');
xlabel(['image index, starting from k1']);
ylabel(['magnitude of translational error [m]']);
grid on;

figure(17);
plot(sum(abs(rot_err), 1)*180/pi, 'k');
xlabel(['image index, starting from k1']);
ylabel(['magnitude of rotational error [deg]']);
grid on;

figure(18);
plot(trans_err');
xlabel(['image index, starting from k1']);
ylabel(['Translational error [m]']);

figure(19);
plot(rot_err'*180/pi);
xlabel(['image index, starting from k1']);
ylabel(['Rotational error [deg]']);


%% Optimize using the dead reckoning as the initial value

% Set the initial value of our states
param_vec = [trans_init_op; rot_init_op];

% optimize
options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', ...
    'Display', 'iter', 'MaxIterations', 10);

tic
[param_vec_best, resnorm, residual, ~, ~, ~, jacobian] = lsqnonlin(f_res_combined, param_vec, [], [], options);
toc

% Error and performance evaluation.
% compare the optimized results and the original ones
figure(20);
visualize_image_points(y_k_j(:, k1:k2, :), y_predict, k1, k2);

best_predict = predict_image_point(param_vec_best(1:3, :), param_vec_best(4:6, :), ...
    rho_i_pj_i, C_c_v, rho_v_c_v, fu, fv, cu, cv, b);
figure(21);
visualize_image_points(y_k_j(:, k1:k2, :), best_predict, k1, k2);

% compute the Root Mean Square Error
error_RMS = sqrt(resnorm / numel(residual));
disp(['RMS Error per entry of our estimation = ' num2str(error_RMS)]);
res_pixels = f_res_pixels(param_vec_best);
err_pix = norm(res_pixels, 1) / numel(res_pixels);
disp(['L1 error per coordinate of our estimation = ' num2str(err_pix) '[pixels]']);
err_pix = sqrt(res_pixels'*res_pixels / numel(res_pixels));
disp(['RMS error per coordinate of our estimation = ' num2str(err_pix) '[pixels]']);
