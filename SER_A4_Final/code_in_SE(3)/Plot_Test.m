%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  This script plots the data in order to get a overview of the problem
%  setup
%  Author: Jiaming Li
%  Created on: Feburary 1, 2024
%  Copyright (c) 2024
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 
% load dataset
load dataset3.mat

K = 1900;

k1 = 1215;   % we start our estimate from here.
k2 = 1714;   % we end our estimate here.

%% 
% 生成相机移动的轨迹的3D点
figure(1);
x = r_i_vk_i(1,:)';  
y = r_i_vk_i(2,:)';  
z = r_i_vk_i(3,:)'; 

x_feature = rho_i_pj_i(1,:)';
y_feature = rho_i_pj_i(2,:)';
z_feature = rho_i_pj_i(3,:)';

% 绘制landmarks
scatter3(x_feature, y_feature, z_feature, 'r', 'filled');

hold on;

plot3(x, y, z, 'b-');

hold off;

xlabel('X轴');
ylabel('Y轴');
zlabel('Z轴');
title('三维点和线段图');
grid on;


%% 
% 在同一个成像平面上绘制双目相机中的左相机和右相机对于同一个点的成像位置
% 是否在同一条水平线上；即其y坐标是否相同
% 注意，此处y_k_j的数据结构为行一个4*1900*20的矩阵
% 其相当于记录了20个Landmark在1900个时间节点上的左侧相机和右侧相机的图像坐标
figure(2);
for i=1:1:K
    for j=1:1:1
        if(y_k_j(1,i,j) >= 0 && y_k_j(2,i,j) >= 0)
            plot(y_k_j(1,i,j), y_k_j(2,i,j), '.', 'MarkerSize', 5, 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'black');
            hold on;
        end
        if(y_k_j(3,i,j) >= 0 && y_k_j(4,i,j) >= 0)
            plot(y_k_j(3,i,j), y_k_j(4,i,j), '.', 'MarkerSize', 5, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'blue');
            hold on;
        end
        pause(0.05);
    end
end

%% 
% 这里绘制了每个landmark在各个时刻是否可见的情况，其和上面左右相机展示的效果相同
figure(3);
for i=1:1:K
    for j=1:1:20
        if(y_k_j(1,i,j) >= 0 && y_k_j(2,i,j) >= 0)
            % y_k_j(1,i,j)
            % y_k_j(2,i,j)
            plot(i, j, '.', 'MarkerSize', 1, 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'black');
            hold on;
        end
    end
end

%% 
% 这里利用measurment model（stereo camera的measurement model）来对measurement进行预测，生成information

% lift the rotation from vector space to lie group
% C_cv = expm(v2skew(C_c_v));
C_vk_i = zeros(3,3);
Landmark_Position_c = zeros(3,1900,20);
Y_PREDICT = zeros(4,1900,20);
figure(4);
for i = 1:1:K
    C_vk_i = expm(v2skew(-theta_vk_i(:,i)));
    for j = 1:1:20
        Landmark_Position_c(:,i,j) = C_c_v*(C_vk_i*(rho_i_pj_i(:,j) - r_i_vk_i(:,i)) - rho_v_c_v);
        Y_PREDICT(:,i,j) = observation_model(Landmark_Position_c(:,i,j), fu, fv, cu, cv, b);
    end
end

for i = k1:1:k2
    for j = 1:1:1
        if(y_k_j(1,i,j) >= 0 && y_k_j(2,i,j) >= 0 && y_k_j(3,i,j) >= 0 && y_k_j(4,i,j) >= 0)
            plot(i, y_k_j(1,i,j) - Y_PREDICT(1,i,j), '.', 'MarkerSize', 5, 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'black');
            % plot(Y_PREDICT(1,i,j), Y_PREDICT(2,i,j), '*', 'MarkerSize', 1, 'MarkerEdgeColor', 'blue', 'MarkerFaceColor', 'blue');
            hold on;
        else
            plot(i, 0, '.', 'MarkerSize', 5, 'MarkerEdgeColor', 'black', 'MarkerFaceColor', 'black');
        end
    end
end
