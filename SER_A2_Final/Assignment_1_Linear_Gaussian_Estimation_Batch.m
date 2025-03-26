%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  This script is a 1-dim linear Gaussian state estimation of the position of a Robot
% Author: Jiaming Li
% Created on: November 21, 2023
% Copyright (c) 2023
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% 
% First we should load the data from the filepath.
load dataset1.mat
n = input('please input the subset size (options: 1000,100,10,1): ');
ch = input('would you like to use cholesky decomposition(click "1") or directly comupte the inverse of the matrices(click "0")?');
K = 12709;

% take the number of the complete subsets
subset_num = fix(K/n);   
rest_num = mod(K,n);


% since the dimension of the input matrix would be different, we need to check the mod
% if the size of the subset is 1, then we need to decrease the size of the
% measurement and input
if rest_num == 0
    subset_num = subset_num - 1;
end

Sampling_Peroid = 0.1;
sigma_q = v_var;
sigma_r = r_var;
measurement = l-r;


%% 
% Here we need to accumulate the input in the whole subset
% And we should also take out the measurement value after the accumulated
% input is add to the system
% compress the input into subset size
sub_input = zeros(subset_num+1, 1);
sub_measurement = zeros(subset_num+1, 1);
for i = 1:1:subset_num
    for j = 1:1:n
        sub_input(i,1) = sub_input(i,1) + Sampling_Peroid*v((j+(i-1)*n), 1) - 0.000451;   % here we subtract the bias
    end
    sub_measurement(i,1) = measurement((i*n), 1);
end
for i = (n*subset_num+1):1:K
    sub_input(subset_num+1,1) = sub_input(subset_num+1,1) + Sampling_Peroid*v(i, 1);
end
sub_measurement(subset_num+1,1) = measurement(K, 1);


% Form the z vector and H matrix
z = [1; sub_input; 1; sub_measurement];
H = sparse([eye(subset_num+2); eye(subset_num+2)]);

% set the motion model in H matrix
for i=2:1:subset_num+2
    H(i,i-1) = -1;
end

% Form the W matrix
W = sparse([sigma_q*speye(subset_num+2), zeros(subset_num+2); zeros(subset_num+2), sigma_r*speye(subset_num+2)]);
W(1,1) = 0.001;
W(subset_num+3, subset_num+3) = 0.0001;

%% compute the estimation results
if ch == 0   % using direct inverse of the sparse matrices
    W_inv = inv(W);
    P = inv(H'*W_inv*H);

    % Find the optimal x for the batch solution
    x_predict = P*H'*W_inv*z;
elseif ch == 1  % using Cholesky decomposition of solving the equations
    W_inv = inv(W);
    L = chol(H'*W_inv*H);
    L = full(L);
    d = linsolve(L, full(H'*W_inv*z));
    x_predict = linsolve(full(L'), full(d));
end


%% 
% Compute the error of the predicted value and the true value
sub_t = zeros(size(x_predict,1), 1);
sub_x_true = zeros(size(x_predict,1), 1);
for i = 1:1:(size(x_predict,1) - 1)
    sub_t(i,1) = t((i-1)*n + 1, 1);
    sub_x_true(i,1) = x_true((i-1)*n + 1, 1);
end
sub_t(size(x_predict,1), 1) = t(K,1);
sub_x_true(size(x_predict,1), 1) = x_true(K,1);
Error = x_predict - sub_x_true;
% Compute the error of the predicted value and the true value


%% 
% Draw the error diagram of the estimation
figure(1);
plot(sub_t, Error,'.-', 'LineWidth', 1.5, 'MarkerSize', 8);
xlabel("t/s")
ylabel("error/m")
title(['The error between the prediction and the true value (', 'δ = ', num2str(n), ')']);
xlim([0,1280]);
grid on;
hold on;


%% 
% Draw the error diagram of the estimation
figure(2);
plot(sub_t, x_predict,'.-', 'LineWidth', 1.5, 'MarkerSize', 8);
hold on;
plot(sub_t, sub_x_true,'-', 'LineWidth', 1.5, 'MarkerSize', 8);
hold on;
xlabel("t/s")
ylabel("error/m")
title(['The curve of the prediction and the true value (', 'δ = ', num2str(n), ')']);
xlim([0,1280]);
grid on;
hold on;


%% 
figure(3);
sigma_squared_xk_delta = var(x_predict - sub_x_true); % variance σ²xkδ
three_sigma_xk_delta = 3 * sqrt(sigma_squared_xk_delta);
Enve = ones(size(sub_t));
for i = 1:1:subset_num+2
    Enve(i,1) = sqrt(P(i,i));
end
% plot(sub_t, three_sigma_xk_delta * ones(size(sub_t)), '.', 'LineWidth', 1.5); % 上边界
% hold on;
% plot(sub_t, -three_sigma_xk_delta * ones(size(sub_t)), '.', 'LineWidth', 1.5); % 下边界
plot(sub_t, 3*Enve, '.', 'LineWidth', 1.5); % 上边界
hold on;
plot(sub_t, -3*Enve, '.', 'LineWidth', 1.5); % 下边界
plot(sub_t, Error,'.-', 'LineWidth', 1.5, 'MarkerSize', 8);
title(['the uncertainty envelope of ±3σxkδ vs. tkδ (', 'δ = ', num2str(n), ')']);
xlabel('tkδ');
ylabel('±3σxkδ');
legend('+3σxkδ', '-3σxkδ');
grid on;


%% 
figure(4);
histogram(Error, 'Normalization', 'probability');
title(['the error histogram and the Gaussian fitting (', 'δ = ', num2str(n), ')']);

% Gaussian Fitting
pd = fitdist(Error, 'Normal');

% get the fitting prams
mu = pd.mu;  % mean
sigma = pd.sigma;  % stdDev

% generate the Gaussian Fitting Curve
x_values = linspace(-0.1, 0.1);
y_values = pdf(pd, x_values);

% use a zoom parameter to visualize the curve
zoom_param = 0;
if n == 1
    zoom_param = 500;
elseif n == 10
    zoom_param = 250;
elseif n == 100
    zoom_param = 100;
elseif n == 1000
    zoom_param = 30;
else
    zoom_param = 20;
end
hold on;
plot(x_values, y_values/zoom_param, 'r', 'LineWidth', 2);
legend('error histogram', 'Gaussian curve fitting');
hold off;
xlim([-0.1,0.1]);
xlabel('Error (x∗kδ - xkδ)');
ylabel('Probabilities');
grid on;


