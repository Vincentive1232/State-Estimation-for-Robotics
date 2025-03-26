%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  This script is a 1-dim linear Gaussian state estimation of the position of a Robot
% Author: Jiaming Li
% Created on: November 21, 2023
% Copyright (c) 2023
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 
% load dataset
load dataset2.mat
n = input('please choose whether you would like to use good initialization or not((yes = 1)<---->(No = 0)): ');
CRLB = input('would you like to use true robot state to evaluate the Jacobians(yes = 1)<---->(No = 0)): ');

%% 
% size and smapling period definition
K = 12609;
T = 0.1;
L = 17;
r_max = 5;  % by changing this parameter we can change the user defined range limit

% Matrices initial definition for noise variance
Qk = [v_var, 0; 0, om_var];
Rk = zeros(34, 34);
for i = 1:2:34
    Rk(i, i) = r_var;
    Rk(i+1, i+1) = b_var;
end
Gk_cat = zeros(34, 3);
Pk_hat_storage = zeros(3, 3*K);
% Rk = [r_var, 0; 0, b_var];

% initialization for state vector and the covariance Matrix
xk_check = zeros(3,K);      % x_k, y_k, theta_k
xk_hat = zeros(3,K+1);      % x_k, y_k, theta_k, this include x0_hat
if n == 1                   % if the initial value of x0 and P0 are perfect
    xk_hat(:,1) = [3.0198; 0.0709; -2.9102];  % take x0_true as a perfect initial value
    Pk_check = diag([1, 1, 0.1]);
    Pk_hat = diag([1, 1, 0.1]);
    Pk_hat_storage(:,1:3) = Pk_hat;
elseif n == 0                                 % if the initial value of x0 and P0 are poor
    xk_hat(:,1) = [1; 1; 0.1];
    Pk_check = diag([1, 1, 0.1]);
    Pk_hat = diag([1, 1, 0.1]);
    Pk_hat_storage(:,1:3) = Pk_hat;
end

% initialization for measurement vector
yk = zeros(34,12609);
for i=1:1:17
    yk(2*i - 1, :) = r(:,i)';
    yk(2*i, :) = b(:,i)';
end

% calculate the Jacobian of the motion model and the measurement model
% Jacobian of the measurement model
syms x y theta xl yl n_rk n_phik;
Measure_f1 = sqrt((xl - x - d*cos(theta))^2 + (yl - y - d*sin(theta))^2) + n_rk;
Measure_f2 = atan2((yl - y - d*sin(theta)), (xl - x - d*cos(theta))) - theta + n_phik;
Measure_F = [Measure_f1; Measure_f2];
X = [x; y; theta];
Measure_J = jacobian(Measure_F,X);
% here we need to input xk, yk, theta_k along with the true location of
% l_th landmark and d into the measurement model jacobian.
Measure_J_func = matlabFunction(Measure_J, 'Vars', {[x; y; theta]; [xl; yl]; [n_rk; n_phik]});


% Jacobian of the measurement noise
X = [n_rk; n_phik];
R_J = jacobian(Measure_F,X);
% here we need to input x(k-1), y(k-1), theta_(k-1) along with the input
% v_k, w_k, we don't need to include the noises since they are already
% included in the input
R_J_func = matlabFunction(R_J, 'Vars', {[x; y; theta]; [xl; yl]; [n_rk; n_phik]});


% Jacobian of the motion model
syms v_k w_k n_vk n_wk;
Motion_f1 = x + T*cos(theta)*(v_k + n_vk);
Motion_f2 = y + T*sin(theta)*(v_k + n_vk);
Motion_f3 = theta + T*(w_k + n_wk);
Motion_F = [Motion_f1; Motion_f2; Motion_f3];
X = [x; y; theta];
Motion_J = jacobian(Motion_F,X);
% here we need to input x(k-1), y(k-1), theta_(k-1) along with the input
% v_k, w_k, we don't need to include the noises since they are already
% included in the input
Motion_J_func = matlabFunction(Motion_J, 'Vars', {[x; y; theta]; [v_k; w_k]; [n_vk; n_wk]});


% Jacobian of the process noise
X = [n_vk; n_wk];
Q_J = jacobian(Motion_F,X);
% here we need to input x(k-1), y(k-1), theta_(k-1) along with the input
% v_k, w_k, we don't need to include the noises since they are already
% included in the input
Q_J_func = matlabFunction(Q_J, 'Vars', {[x; y; theta]; [v_k; w_k]; [n_vk; n_wk]});



% this is the input of the motion model since the noise has been included
% in the input value
stacked_v = [v'; om'];
% stacked_y = [r, b];

count = 0;
%% 
% main part
for i = 2:1:K+1
    if CRLB == 0
        Fk_1 = Motion_J_func(xk_hat(:,i-1), stacked_v(:,i-1), [0; 0]);
        Qk_pri = Q_J_func(xk_hat(:,i-1), stacked_v(:,i-1), [0; 0]) * Qk * (Q_J_func(xk_hat(:,i-1), stacked_v(:,i-1),[0; 0])');    % initially we use x0_hat to get Q1_prime
        Pk_check = Fk_1*Pk_hat*Fk_1'+ Qk_pri;
        xk_check(:,i-1) = ComputeXk_check(xk_hat(:,i-1), stacked_v(:,i-1), T);
        Gk_cat = zeros(34, 3);
        count = 0;
        for j = 1:1:L        % we should use every visible landmark to do a prediction and compute the average
            if (r(i - 1, j) ~= 0 && r(i - 1, j) <= r_max)
                count = count + 1;
                Gk = Measure_J_func(xk_check(:,i-1), l(j,:)', [0; 0]);
                Gk_cat(2*j - 1, :) = Gk(1, :);
                Gk_cat(2*j, :) = Gk(2, :);
            else 
                continue;
            end
        end
        if count >= 1
            Kk_l = Pk_check*Gk_cat'*(inv(Gk_cat*Pk_check*Gk_cat'+Rk));
            Pk_hat = (eye(3,3) - Kk_l*Gk_cat)*Pk_check;
            Pk_hat_storage(:, (i-1)*3+1:(i-1)*3+3) = Pk_hat;
            xk_hat(:,i) = xk_check(:,i-1) + Kk_l*(yk(:, i-1)- ComuputeYk_check(xk_check(:, i-1), l, d, r, i - 1));
        else
            Pk_hat = Pk_check;
            Pk_hat_storage(:, (i-1)*3+1:(i-1)*3+3) = Pk_hat;
            xk_hat(:,i) = xk_check(:,i-1);
        end
    elseif CRLB == 1
        Fk_1 = Motion_J_func([x_true(i-1, 1); y_true(i-1, 1); th_true(i-1, 1)], stacked_v(:,i-1), [0; 0]);
        Qk_pri = Q_J_func([x_true(i-1, 1); y_true(i-1, 1); th_true(i-1, 1)], stacked_v(:,i-1), [0; 0]) * Qk * (Q_J_func([x_true(i-1, 1); y_true(i-1, 1); th_true(i-1, 1)], stacked_v(:,i-1),[0; 0])');    % initially we use x0_hat to get Q1_prime
        Pk_check = Fk_1*Pk_hat*Fk_1'+ Qk_pri;
        xk_check(:,i-1) = ComputeXk_check([x_true(i-1, 1); y_true(i-1, 1); th_true(i-1, 1)], stacked_v(:,i-1), T);
        Gk_cat = zeros(34, 3);
        count = 0;
        for j = 1:1:L        % we should use every visible landmark to do a prediction and compute the average
            if (r(i - 1, j) ~= 0 && r(i - 1, j) <= r_max)
                count = count + 1;
                Gk = Measure_J_func(xk_check(:,i-1), l(j,:)', [0; 0]);
                Gk_cat(2*j - 1, :) = Gk(1, :);
                Gk_cat(2*j, :) = Gk(2, :);
            else 
                continue;
            end
        end
        if count >= 1
            Kk_l = Pk_check*Gk_cat'*(inv(Gk_cat*Pk_check*Gk_cat'+Rk));
            Pk_hat = (eye(3,3) - Kk_l*Gk_cat)*Pk_check;
            Pk_hat_storage(:, (i-1)*3+1:(i-1)*3+3) = Pk_hat;
            xk_hat(:,i) = xk_check(:,i-1) + Kk_l*(yk(:, i-1)- ComuputeYk_check(xk_check(:, i-1), l, d, r, i - 1));
        else
            Pk_hat = Pk_check;
            Pk_hat_storage(:, (i-1)*3+1:(i-1)*3+3) = Pk_hat;
            xk_hat(:,i) = xk_check(:,i-1);
        end
    end
end

%% Plot

% draw the animation of the EKF estimation
figure(1);
plot(l(:,1)', l(:,2)', 'o','MarkerSize', 5, 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'white'); % draw the estimated curve.
hold on;
% plot(xk_hat(1,:)', xk_hat(2,:)', '.','MarkerSize', 2, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'blue'); % draw the estimated curve.
% hold on;
% plot(x_true', y_true', '.', 'MarkerSize', 2, 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'black'); % % draw the true curve.
hold on;
h = plot(xk_hat(1,1), xk_hat(2,1), '.', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); 
h1 = plot(x_true(1,1), y_true(1,1), '.', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
h2 = ellipse_draw(Pk_hat_storage(1:2,1:2), [xk_hat(1,1); xk_hat(2,1); xk_hat(3,1)]);
title('compare the groundtruth with the estimation');
xlabel('X/m');
ylabel('Y/m');
grid on;
for i = 2:K
    delete(h2);
    set(h, 'XData', xk_hat(1,i), 'YData', xk_hat(2,i));
    set(h1, 'XData', x_true(i,1), 'YData', y_true(i,1));
    h2 = ellipse_draw(Pk_hat_storage(1:2, ((i-1)*3+1):((i-1)*3+2)), [xk_hat(1,i); xk_hat(2,i); xk_hat(3,i)]);
    pause(0.005); 
end
hold on;

% draw the animation of the EKF estimation
figure(2);
plot(x_true', y_true', '.','MarkerSize', 2, 'MarkerEdgeColor', 'red', 'MarkerFaceColor', 'red');
title('The true curve measured by the motion capture system');
xlabel('X/m');
ylabel('Y/m');
legend('groudtruth');
grid on;

% 绘制二维点
figure(3);
plot(xk_hat(1,:)', xk_hat(2,:)', '.','MarkerSize', 2, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'blue'); 
title('The estimated curve');
xlabel('X/m');
ylabel('Y/m');
legend('estimated position');
grid on;
hold on;

x_sigma = zeros(K, 1);
y_sigma = zeros(K, 1);
theta_sigma = zeros(K, 1);
for i = 1:1:K
    x_sigma(i, 1) = sqrt(Pk_hat_storage(1, 3*(i-1)+1));
    y_sigma(i, 1) = sqrt(Pk_hat_storage(2, 3*(i-1)+2));
    theta_sigma(i, 1) = sqrt(Pk_hat_storage(3, 3*(i-1)+3));
end

figure(4);
plot(t, xk_hat(1,2:K+1)' - x_true(:,1), 'b'); 
hold on;
plot(t, 3*x_sigma, 'r'); 
hold on;
plot(t, -3*x_sigma, 'r'); 
hold on;
title('The x-error between the estimate and the groundtruth');
xlabel('t/s');
ylabel('X/m');
legend('x error');
if r_max == 1
    xlim([0, K/10]);
    ylim([-4, 4]);
elseif r_max == 3
    xlim([0, K/10]);
    ylim([-0.2, 0.15]);
elseif r_max == 5
    xlim([0, K/10]);
    ylim([-0.2, 0.15]);
end
grid on;
hold on;

figure(5);
plot(t, xk_hat(2,2:K+1)' - y_true(:,1), 'b'); 
hold on;
plot(t, 3*y_sigma, 'r'); 
hold on;
plot(t, -3*y_sigma, 'r'); 
hold on;
title('The y-error between the estimate and the groundtruth');
xlabel('t/s');
ylabel('Y/m');
legend('y error');
if r_max == 1
    xlim([0, K/10]);
    ylim([-4, 4]);
elseif r_max == 3
    xlim([0, K/10]);
    ylim([-0.15, 0.25]);
elseif r_max == 5
    xlim([0, K/10]);
    ylim([-0.15, 0.25]);
end
grid on;
hold on;

figure(6);
theta_error = zeros(K, 1);
for i = 2:1:K+1
    if (xk_hat(3,i) - th_true((i-1),1)) >= 5
        theta_error((i-1), 1) = 2*pi - (xk_hat(3,i) - th_true((i - 1),1));
    elseif (xk_hat(3,i) - th_true((i-1),1)) <= -5
        theta_error((i-1), 1) = 2*pi + (xk_hat(3,i) - th_true((i - 1),1));
    else
        theta_error((i-1), 1) = (xk_hat(3,i) - th_true((i - 1),1));
    end
end
plot(t, theta_error*(180/pi), 'b'); 
hold on;
plot(t, 3*theta_sigma*(180/pi), 'r'); 
hold on;
plot(t, -3*theta_sigma*(180/pi), 'r'); 
hold on;
title('The theta-error between the estimate and the groundtruth');
xlabel('t/s');
ylabel('Theta/deg');
legend('theta error');
if r_max == 1
    xlim([0, K/10]);
    ylim([-80, 80]);
elseif r_max == 3
    xlim([0, K/10]);
    ylim([-10, 8]);
elseif r_max == 5
    xlim([0, K/10]);
    ylim([-10, 8]);
end
grid on;
hold on;

% draw the animation of the EKF estimation
figure(7);
plot(l(:,1)', l(:,2)', 'o','MarkerSize', 5, 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'white'); % draw the estimated curve.
hold on;
plot(xk_hat(1,:)', xk_hat(2,:)', '.','MarkerSize', 2, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'blue'); % draw the estimated curve.
hold on;
plot(x_true', y_true', '.', 'MarkerSize', 2, 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'black'); % % draw the true curve.
hold on;
title('Robot path(ground truth) and landmarks');
xlabel("x[m]");
ylabel("y[m]");
grid on;


%% 
% calculate h(x_k-1, uk, wk), g(x_k, nk_l)
function xk_check_temp = ComputeXk_check(last_x, v_k, T)
    B = [cos(last_x(3,1)), 0; sin(last_x(3,1)), 0; 0, 1];
    xk_check_temp = last_x + T*B*v_k;
    xk_check_temp(3, 1) = xk_check_temp(3, 1) - 2*pi*floor((xk_check_temp(3, 1)+pi)/(2*pi));
end


function yk_check_temp = ComuputeYk_check(x_k, l, d, r, time_step)
    yk_check_temp = zeros(34, 1);
    for i=1:1:17 
        if (r(time_step, i) ~= 0 && r(time_step, i) <= 6)
            a = l(i,1)-x_k(1,1)-d*cos(x_k(3,1));
            b = l(i,2)-x_k(2,1)-d*sin(x_k(3,1));
            yk_check_temp(2*i - 1, 1) = sqrt(a^2+b^2);
            temp = atan2(b,a)-x_k(3,1);
            if temp > pi
                temp = atan2(b,a)-x_k(3,1) - 2*pi;
            elseif temp < -pi
                temp = atan2(b,a)-x_k(3,1) + 2*pi;
            end
            yk_check_temp(2*i, 1) = temp;
        end
    end
end

%% 

function h = ellipse(ra,rb,ang,x0,y0,Nb)
% ra = radius along x-axis
% rb = radius along y-axis
% ang = angle in radians
% x0,y0 = center of ellipse
% C = color
% Nb = number of points to be used in drawing the ellipse

% Ellipse formula: (x-x0)^2/ra^2 + (y-y0)^2/rb^2 = 1

if nargin < 6
    Nb = 100;
end

theta = linspace(0, 2 * pi, Nb);
p = [ra * cos(theta); rb * sin(theta)];
R = [cos(ang), -sin(ang); sin(ang), cos(ang)];
p = R * p;
p(1, :) = p(1, :) + x0;
p(2, :) = p(2, :) + y0;

figure(1);
h = plot(p(1, :), p(2, :), 'b');
end

function h = ellipse_draw(covarianceMatrix, center)
[U,S,V] = svd(9*covarianceMatrix);    % eigenvalue/eigenvector decomposition
angle = cart2pol(U(1, :), U(2, :))*180/pi;
beta = angle(1, 1);
if beta<0; beta=beta+180; end
    
semimajor = 3*sqrt(S(1, 1)); % long axis
semiminor = 3*sqrt(S(2, 2)); % short axis
alpha = linspace(0, 360, 2000)';
level = 1;
% center
ellipse_X = center(1,1)+sqrt(level)*(semimajor*cosd(alpha)*cosd(beta)-...
    semiminor*sind(alpha)*sind(beta));
ellipse_Y = center(2,1)+sqrt(level)*(semimajor*cosd(alpha)*sind(beta)+...
    semiminor*sind(alpha)*cosd(beta));

h = plot(ellipse_X, ellipse_Y, 'Color', 'r');
end

