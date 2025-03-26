%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  This script is a BatchGaussianNewton pose estimation of the position of
%  a Robot in Lie algebra and Lie Group
%  Author: Jiaming Li
%  Created on: Feburary 1, 2024
%  Copyright (c) 2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Please pay attention to this instruction!
% [1] Before going into the iterating part of each method, you need to
%     run section 1~3 before hand. This is for initializing some variables and
%     visualizing some data.


% [2] Run each method step by step. For example:  
%     Batch step [1] --> Batch step [2] --> Batch step [3]
%     Sliding Window (kappa=50) step [1] -->  Sliding Window (kappa=50) step [2]
%     Sliding Window (kappa=10) step [1] -->  Sliding Window (kappa=10) step [2]


%     DON'T FORGET TO RERUN THE FIRST 3 SECTIONS BEFORE YOU CHANGE TO A DIFFERENT METHOD!!!!!!!!

%% 1. load dataset
clc
clear
load dataset3.mat

%% 2. check the number of visible landmarks at every timestep

% plot the number of visible landmarks at every timestep 
visible_landmarks = sum(squeeze(y_k_j(1, :, :)) >= 0, 2);
num_landmarks = size(rho_i_pj_i, 2);

figure(6);
plot(t, visible_landmarks, 'r.');
hold on;

index = (visible_landmarks >= 3);
plot(t(index), visible_landmarks(index), 'g.');
xlabel('t[s]')
ylabel('Number of visible landmarks time t');
grid on;

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


%% 3. Initialize some variables

% total timsteps
K = size(t, 2);

% number of landmarks
num_landmarks = size(rho_i_pj_i, 2);

% intrics of the stereo camera
fu, fv, cu, cv, b;

% the transformation between stereo camera and the IMU
T_c_v = vC2T(rho_v_c_v, C_c_v);

% Transforming the groundtruth(rotational and translation vectors) into a
% sequence of Poses
r_i_vk_i;
C_vk_i = zeros(3, 3, K);
T_vk_i = zeros(4, 4, K);
for i = 1:1:K
    C_vk_i(:, :, i) = Psi2C_so3(theta_vk_i(:, i));
    T_vk_i(:, :, i) = vC2T(r_i_vk_i(:, i), C_vk_i(:, :, i));
end

% Input stacking
varpi_vk_i_vk = [-v_vk_vk_i; -w_vk_vk_i];

% compute the time differences
ts = circshift(t, 1);
ts(1) = 0;
dt = t - ts;

% filter out invalid measurements and get the corresponding indexes
y_filter = zeros(size(y_k_j));
y_filter(y_k_j == -1) = 0;
y_filter(y_k_j ~= -1) = 1;

% covariances
w_var_inv = 1./w_var;
v_var_inv = 1./v_var;
y_var_inv = 1./y_var;
Q_k_inv = diag([v_var_inv; w_var_inv]);
Q_inv = zeros(6, 6, K);
for i = 1:1:K
    Q_inv(:, :, i) = Q_k_inv;
end
R_k_j_inv = diag(y_var_inv);
R_k_inv = zeros(num_landmarks*size(y_var, 1), num_landmarks*size(y_var, 1));
for i = 1:1:num_landmarks
    R_k_inv(((i-1)*size(y_var, 1)+1):i*size(y_var, 1), ((i-1)*size(y_var, 1)+1):i*size(y_var, 1)) = R_k_j_inv;
end

% helper matrix(D_Transpose is used for computing G_jk matrices)
D = [1, 0, 0;
     0, 1, 0;
     0, 0, 1;
     0, 0, 0];

% estimated values of variables
% hat_T_vk_i = zeros(4, 4, K);
hat_T_vk_i = T_vk_i;
hat_P = zeros(6, 6, K);
for i = 1:1:K
    hat_P(:, :, i) = eye(6)*0.0001;
end
hat_stds = ones(6, K)*sqrt(0.0001);

% Set initial operatinmg point
init_T_vk_i = hat_T_vk_i;
init_P = hat_P;

% Set optimizing interval
k1 = 1216;
k2 = 1715;

%% Batch step [1]: Initialization of the Batch Optimization

for i = k1+1:1:k2
    % initialize the mean
    hat_T_vk_i(:, :, i) = motion_model(hat_T_vk_i(:, :, (i-1)), varpi_vk_i_vk(:, (i-1)), dt(i));
    % initialize the covariance
    F = linearized_motion_model(hat_T_vk_i(:, :, (i-1)), varpi_vk_i_vk(:, (i-1)), dt(i));
    Q_inv = Q_k_inv / (dt(i)^2);
    hat_P(:, :, i) = F * hat_P(:, :, (i-1)) * F' + inv(Q_inv);
end

% This is only for initial error terms
init_T_vk_i = hat_T_vk_i;
init_P = hat_P;

%% Batch step [2]: Gauss-Newton update Batch
disp('---------------------------------------------------------------------------------------');
disp('Batch case');
tic

max_iterations = 10;

for i = 1:1:max_iterations
    [eps, hat_T_vk_i, hat_P_k, LHS_k] = GN_update(T_c_v, hat_T_vk_i, init_T_vk_i, init_P, varpi_vk_i_vk, Q_k_inv, rho_i_pj_i, y_k_j, y_filter,...
        R_k_inv, dt, fu, fv, cu, cv, b, D, k1, k2);
    
    disp(['iteration[' num2str(i) ']:' 'Norm error of our estimation = ' num2str(eps)]);
    
    if eps <= 1e-5
        break;
    end
end
toc

%% Batch step [3]: Batch Results analysis

hat_r = zeros(3, k2-k1+1);
hat_C = zeros(3, 3, k2-k1+1);
for i = 1:1:k2-k1+1
    [hat_r(:, i), hat_C(:, :, i)] = T2vC(hat_T_vk_i(:, :, i+k1-1));
end

r_error = hat_r - r_i_vk_i(:, k1:k2);
rot_error = zeros(3, k2-k1+1);
Id3 = eye(3);
for i = 1:1:k2-k1+1
    rot_error(:, i) = skew2v(Id3 - reshape(hat_C(:, :, i), 3, 3)*reshape(C_vk_i(:, :, i+k1-1), 3, 3)');
end

hat_stds = zeros(6, k2-k1+1);
for i = 1:1:k2-k1+1
    hat_stds(:, i) = sqrt(diag(reshape(hat_P_k(:, :, k1+i-1), 6, 6)));
end

figure(1);
subplot(7,1,1);
plot(t(k1:k2), visible_landmarks(k1:k2), 'g.');
hold on;
for i = 1:1:k2-k1+1
    if visible_landmarks(i+k1-1) <= 3
        plot(t(i+k1-1), visible_landmarks(i+k1-1), 'r.');
    end
end
% index = (visible_landmarks(k1:k2) <= 3);
% plot(t(k1:k2, index), visible_landmarks(k1:k2,(index)), 'r.');
xlabel('t[s]')
ylabel('Num. of visible');
grid on;

subplot(7,1,2);
plot(t(k1:k2), r_error(1, :), "Color", 'r');
hold on;
plot(t(k1:k2), 3*hat_stds(1, :), "Color", 'b');
hold on;
plot(t(k1:k2), -3*hat_stds(1, :), "Color", 'b');
xlabel('t [s]');
ylabel('error {r_x} [m]')
title('error {r_x}');
grid on;

subplot(7,1,3);
plot(t(k1:k2), r_error(2, :), "Color", 'g');
hold on;
plot(t(k1:k2), 3*hat_stds(2, :), "Color", 'r');
hold on;
plot(t(k1:k2), -3*hat_stds(2, :), "Color", 'r');
xlabel('t [s]');
ylabel('error {r_y} [m]')
title('error {r_y}');
grid on;

subplot(7,1,4);
plot(t(k1:k2), r_error(3, :), "Color", 'b');
hold on;
plot(t(k1:k2), 3*hat_stds(3, :), "Color", 'g');
hold on;
plot(t(k1:k2), -3*hat_stds(3, :), "Color", 'g');
xlabel('t [s]');
ylabel('error {r_z} [m]')
title('error {r_z}');
grid on;

subplot(7,1,5);
plot(t(k1:k2), rot_error(1, :), "Color", 'r');
hold on;
plot(t(k1:k2), 3*hat_stds(4, :), "Color", 'b');
hold on;
plot(t(k1:k2), -3*hat_stds(4, :), "Color", 'b');
xlabel('t [s]');
ylabel('error \theta_x [rad]')
title('error \theta_x');
grid on;

subplot(7,1,6);
plot(t(k1:k2), rot_error(2, :), "Color", 'g');
hold on;
plot(t(k1:k2), 3*hat_stds(5, :), "Color", 'r');
hold on;
plot(t(k1:k2), -3*hat_stds(5, :), "Color", 'r');
xlabel('t [s]');
ylabel('error \theta_y [rad]')
title('error \theta_y');
grid on;

subplot(7,1,7);
plot(t(k1:k2), rot_error(3, :), "Color", 'b');
hold on;
plot(t(k1:k2), 3*hat_stds(6, :), "Color", 'g');
hold on;
plot(t(k1:k2), -3*hat_stds(6, :), "Color", 'g');
xlabel('t [s]');
ylabel('error \theta_z [rad]')
title('error \theta_z');
grid on;

%% Sliding Window (kappa=50) step [1]: Gauss-Newton update Sliding Window 1.
disp('---------------------------------------------------------------------------------------');
disp('Sliding window case (k = 50)');

max_iterations = 10;
kappa = 50;
k_kappa_1 = k1;
k_kappa_2 = k1 + kappa;

tic
for j = k1:k2
    k_kappa_1 = j;
    k_kappa_2 = j + kappa;
    for i = k_kappa_1+1:1:k_kappa_2
        % initialize the mean
        hat_T_vk_i(:, :, i) = motion_model(hat_T_vk_i(:, :, (i-1)), varpi_vk_i_vk(:, (i-1)), dt(i));
        % initialize the covariance
        F = linearized_motion_model(hat_T_vk_i(:, :, (i-1)), varpi_vk_i_vk(:, (i-1)), dt(i));
        Q_inv = Q_k_inv / (dt(i)^2);
        hat_P(:, :, i) = F * hat_P(:, :, (i-1)) * F' + inv(Q_inv);
    end

    % This is only for initial error terms
    init_T_vk_i = hat_T_vk_i;
    init_P = hat_P;
    for i = 1:1:max_iterations
        [eps, hat_T_vk_i, hat_P, LHS_k] = GN_update(T_c_v, hat_T_vk_i, init_T_vk_i, init_P, varpi_vk_i_vk, Q_k_inv, rho_i_pj_i, y_k_j, y_filter,...
            R_k_inv, dt, fu, fv, cu, cv, b, D, k_kappa_1, k_kappa_2);
        
        disp(['iteration[' num2str(i) ']: ' 'Norm error of our estimation = ' num2str(eps)]);

        if eps <= 1e-5
            break;
        end
    end
end
toc

%% Sliding Window (kappa=50) step [2]: Sliding Window Results analysis

hat_r = zeros(3, k2-k1+1);
hat_C = zeros(3, 3, k2-k1+1);
for i = 1:1:k2-k1+1
    [hat_r(:, i), hat_C(:, :, i)] = T2vC(hat_T_vk_i(:, :, i+k1-1));
end

r_error = hat_r - r_i_vk_i(:, k1:k2);
rot_error = zeros(3, k2-k1+1);
Id3 = eye(3);
for i = 1:1:k2-k1+1
    rot_error(:, i) = skew2v(Id3 - reshape(hat_C(:, :, i), 3, 3)*reshape(C_vk_i(:, :, i+k1-1), 3, 3)');
end

hat_stds = zeros(6, k2-k1+1);
for i = 1:1:k2-k1+1
    hat_stds(:, i) = sqrt(diag(reshape(hat_P(:, :, k1+i-1), 6, 6)));
end

figure(1);
subplot(7,1,1);
plot(t(k1:k2), visible_landmarks(k1:k2), 'g.');
hold on;
for i = 1:1:k2-k1+1
    if visible_landmarks(i+k1-1) <= 3
        plot(t(i+k1-1), visible_landmarks(i+k1-1), 'r.');
    end
end
% index = (visible_landmarks(k1:k2) <= 3);
% plot(t(k1:k2, index), visible_landmarks(k1:k2,(index)), 'r.');
xlabel('t[s]')
ylabel('Num. of visible');
grid on;

subplot(7,1,2);
plot(t(k1:k2), r_error(1, :), "Color", 'r');
hold on;
plot(t(k1:k2), 3*hat_stds(1, :), "Color", 'b');
hold on;
plot(t(k1:k2), -3*hat_stds(1, :), "Color", 'b');
xlabel('t [s]');
ylabel('error {r_x} [m]')
title('error {r_x}');
grid on;

subplot(7,1,3);
plot(t(k1:k2), r_error(2, :), "Color", 'g');
hold on;
plot(t(k1:k2), 3*hat_stds(2, :), "Color", 'r');
hold on;
plot(t(k1:k2), -3*hat_stds(2, :), "Color", 'r');
xlabel('t [s]');
ylabel('error {r_y} [m]')
title('error {r_y}');
grid on;

subplot(7,1,4);
plot(t(k1:k2), r_error(3, :), "Color", 'b');
hold on;
plot(t(k1:k2), 3*hat_stds(3, :), "Color", 'g');
hold on;
plot(t(k1:k2), -3*hat_stds(3, :), "Color", 'g');
xlabel('t [s]');
ylabel('error {r_z} [m]')
title('error {r_z}');
grid on;

subplot(7,1,5);
plot(t(k1:k2), rot_error(1, :), "Color", 'r');
hold on;
plot(t(k1:k2), 3*hat_stds(4, :), "Color", 'b');
hold on;
plot(t(k1:k2), -3*hat_stds(4, :), "Color", 'b');
xlabel('t [s]');
ylabel('error \theta_x [rad]')
title('error \theta_x');
grid on;

subplot(7,1,6);
plot(t(k1:k2), rot_error(2, :), "Color", 'g');
hold on;
plot(t(k1:k2), 3*hat_stds(5, :), "Color", 'r');
hold on;
plot(t(k1:k2), -3*hat_stds(5, :), "Color", 'r');
xlabel('t [s]');
ylabel('error \theta_y [rad]')
title('error \theta_y');
grid on;

subplot(7,1,7);
plot(t(k1:k2), rot_error(3, :), "Color", 'b');
hold on;
plot(t(k1:k2), 3*hat_stds(6, :), "Color", 'g');
hold on;
plot(t(k1:k2), -3*hat_stds(6, :), "Color", 'g');
xlabel('t [s]');
ylabel('error \theta_z [rad]')
title('error \theta_z');
grid on;


%% Sliding Window (kappa=10) step [1]: Gauss-Newton update Sliding Window 1.
disp('---------------------------------------------------------------------------------------');
disp('Sliding window case (k = 10)');

max_iterations = 10;
kappa = 10;
k_kappa_1 = k1;
k_kappa_2 = k1 + kappa;

tic
for j = k1:k2
    k_kappa_1 = j;
    k_kappa_2 = j + kappa;
    for i = k_kappa_1+1:1:k_kappa_2
        % initialize the mean
        hat_T_vk_i(:, :, i) = motion_model(hat_T_vk_i(:, :, (i-1)), varpi_vk_i_vk(:, (i-1)), dt(i));
        % initialize the covariance
        F = linearized_motion_model(hat_T_vk_i(:, :, (i-1)), varpi_vk_i_vk(:, (i-1)), dt(i));
        Q_inv = Q_k_inv / (dt(i)^2);
        hat_P(:, :, i) = F * hat_P(:, :, (i-1)) * F' + inv(Q_inv);
    end

    % This is only for initial error terms
    init_T_vk_i = hat_T_vk_i;
    init_P = hat_P;
    for i = 1:1:max_iterations
        [eps, hat_T_vk_i, hat_P, LHS_k] = GN_update(T_c_v, hat_T_vk_i, init_T_vk_i, init_P, varpi_vk_i_vk, Q_k_inv, rho_i_pj_i, y_k_j, y_filter,...
            R_k_inv, dt, fu, fv, cu, cv, b, D, k_kappa_1, k_kappa_2);
        
        disp(['iteration[' num2str(i) ']: ' 'Norm error of our estimation = ' num2str(eps)]);

        if eps <= 1e-5
            break;
        end
    end
end
toc

%% Sliding Window (kappa=10) step [2]: Sliding Window Results analysis

hat_r = zeros(3, k2-k1+1);
hat_C = zeros(3, 3, k2-k1+1);
for i = 1:1:k2-k1+1
    [hat_r(:, i), hat_C(:, :, i)] = T2vC(hat_T_vk_i(:, :, i+k1-1));
end

r_error = hat_r - r_i_vk_i(:, k1:k2);
rot_error = zeros(3, k2-k1+1);
Id3 = eye(3);
for i = 1:1:k2-k1+1
    rot_error(:, i) = skew2v(Id3 - reshape(hat_C(:, :, i), 3, 3)*reshape(C_vk_i(:, :, i+k1-1), 3, 3)');
end

hat_stds = zeros(6, k2-k1+1);
for i = 1:1:k2-k1+1
    hat_stds(:, i) = sqrt(diag(reshape(hat_P(:, :, k1+i-1), 6, 6)));
end

figure(1);
subplot(7,1,1);
plot(t(k1:k2), visible_landmarks(k1:k2), 'g.');
hold on;
for i = 1:1:k2-k1+1
    if visible_landmarks(i+k1-1) <= 3
        plot(t(i+k1-1), visible_landmarks(i+k1-1), 'r.');
    end
end
% index = (visible_landmarks(k1:k2) <= 3);
% plot(t(k1:k2, index), visible_landmarks(k1:k2,(index)), 'r.');
xlabel('t[s]')
ylabel('Num. of visible');
grid on;

subplot(7,1,2);
plot(t(k1:k2), r_error(1, :), "Color", 'r');
hold on;
plot(t(k1:k2), 3*hat_stds(1, :), "Color", 'b');
hold on;
plot(t(k1:k2), -3*hat_stds(1, :), "Color", 'b');
xlabel('t [s]');
ylabel('error {r_x} [m]')
title('error {r_x}');
grid on;

subplot(7,1,3);
plot(t(k1:k2), r_error(2, :), "Color", 'g');
hold on;
plot(t(k1:k2), 3*hat_stds(2, :), "Color", 'r');
hold on;
plot(t(k1:k2), -3*hat_stds(2, :), "Color", 'r');
xlabel('t [s]');
ylabel('error {r_y} [m]')
title('error {r_y}');
grid on;

subplot(7,1,4);
plot(t(k1:k2), r_error(3, :), "Color", 'b');
hold on;
plot(t(k1:k2), 3*hat_stds(3, :), "Color", 'g');
hold on;
plot(t(k1:k2), -3*hat_stds(3, :), "Color", 'g');
xlabel('t [s]');
ylabel('error {r_z} [m]')
title('error {r_z}');
grid on;

subplot(7,1,5);
plot(t(k1:k2), rot_error(1, :), "Color", 'r');
hold on;
plot(t(k1:k2), 3*hat_stds(4, :), "Color", 'b');
hold on;
plot(t(k1:k2), -3*hat_stds(4, :), "Color", 'b');
xlabel('t [s]');
ylabel('error \theta_x [rad]')
title('error \theta_x');
grid on;

subplot(7,1,6);
plot(t(k1:k2), rot_error(2, :), "Color", 'g');
hold on;
plot(t(k1:k2), 3*hat_stds(5, :), "Color", 'r');
hold on;
plot(t(k1:k2), -3*hat_stds(5, :), "Color", 'r');
xlabel('t [s]');
ylabel('error \theta_y [rad]')
title('error \theta_y');
grid on;

subplot(7,1,7);
plot(t(k1:k2), rot_error(3, :), "Color", 'b');
hold on;
plot(t(k1:k2), 3*hat_stds(6, :), "Color", 'g');
hold on;
plot(t(k1:k2), -3*hat_stds(6, :), "Color", 'g');
xlabel('t [s]');
ylabel('error \theta_z [rad]')
title('error \theta_z');
grid on;