close all;
clear all;clc;
dbstop error

%%
addpath('functions/functions_dataload');
addpath('functions/functions_transform');
addpath('functions/functions_algorithm');

%% global
global D2R R2D
D2R = pi/180; R2D = 180/pi;

%% User-define parameters
n_ring = 16;
azimuth_res = 0.2;
dis_lim = 1.5 ;
ref_seq = 1 ;

% Path to datasets
dir_to_dataset = 'D:\lidar_plane_data\';
dataset_name   = 'cut_flip45';
data_type      = '1cams2lidars';

data = loadData(dir_to_dataset, dataset_name, data_type); % load all data in a dataset
data = sortRings(data); % generate ".pcls_rings", ".IndexRings" in "data".

fprintf('# of data: %d\n', data.n_data);
%% variable
n_az_step = 360 / azimuth_res +1;

%% lidar image 생성!
imgs_intensity = cell(2,data.n_data);
imgs_rho       = cell(2,data.n_data);
imgs_index     = cell(2,data.n_data);

for m = 1:2
    for n = 1:data.n_data
        [imgs_intensity{m,n}, imgs_rho{m,n}, imgs_index{m,n}] = generateLidarImages(data.pcls{m,n}, azimuth_res, dis_lim);
    end
end

%%
X_0 = cell(data.n_data,1); 
X_1 = cell(data.n_data,1);
X_0_ch = cell(data.n_data,1); 
X_1_ch = cell(data.n_data,1);
roi_imgs_range = cell(2, data.n_data);
plane_0 = cell(data.n_data,1);
plane_1 = cell(data.n_data,1);

l0_reject_data = [];
l1_reject_data = [];

mask_invalid_data = false(2,data.n_data);

i_data = 1:data.n_data;
n_i_data = length(i_data);

for m = 1:2
    for n = i_data %1:data.n_data
        roi = [];
        %% foreground and outliers suppression
        [roi, roi_3D_pts]=forground_outsup(data,imgs_rho,imgs_index,m,n,ref_seq);
        
        [out_flag, mask_invalid_data] = check_n_pts('foreground_outsup',roi, mask_invalid_data, m,n);
        if out_flag
            continue;
        end
        
        %% Target detection & completion
        %ransac pramaters
        ransac.param.iter   = 50;
        ransac.param.thr    = 0.02; %3cm
        ransac.param.inlier = 0.6*size(roi_3D_pts,2);
        
        %% RANSAC
        [plane_a,~,~, roi] = ransac_plane(roi_3D_pts, roi, ransac.param.iter, ransac.param.thr, ransac.param.inlier ,m ,n);
        
        %% Restoration
        % roi_imgs_range --> for visualization
        [roi,roi_3D_pts, roi_imgs_range{m,n}] = restore_channel(roi, imgs_rho,imgs_index,data,m, n);
        
        [out_flag, mask_invalid_data] = check_n_pts('restore_channel',roi, mask_invalid_data, m,n);
        if out_flag
            continue;
        end

        %% RANSAC
        [plane_b,roi_3D_pts,~,roi] = ransac_plane(roi_3D_pts, roi, ransac.param.iter,ransac.param.thr,0.9*size(roi_3D_pts,2),m,n);
        
        [out_flag , mask_invalid_data] = check_ransac_plane(plane_a, plane_b, mask_invalid_data, m,n);
        if out_flag
            continue;
        end
        
        %% Plane_reweight
        [plane, roi_3D_pts, mask_reweight, roi] = plane_reweight(roi_3D_pts,roi,m,n,1);
        if m==1
            plane_0{n} = plane;
        elseif m==2
            plane_1{n} = plane;
        end
        
        [out_flag, mask_invalid_data,roi,roi_3D_pts] = check_ratio_pts('plane_reweight',mask_reweight, mask_invalid_data,roi, imgs_index, data, m,n);
        if out_flag
            continue;
        end
        
        %%
        [ch, ~] = find(roi>0);
        
        if m==1
            X_0{n} = roi_3D_pts;
            X_0_ch{n} = ch;
        elseif m==2
            X_1{n} = roi_3D_pts;
             X_1_ch{n} = ch;
        end
        
        n_inliers = size(roi_3D_pts,2);
    end %n
end %m
fprintf('==============================================================================================\n');
figure(100); set(gcf,'Color','k'); set(gca,'Color','k'); set(gca,'xcolor','w'); set(gca,'ycolor','w'); set(gca,'zcolor','w');
title('Before restoration (Total)','Color','w','FontSize', 15);
figure(200); set(gcf,'Color','k'); set(gca,'Color','k'); set(gca,'xcolor','w'); set(gca,'ycolor','w'); set(gca,'zcolor','w');
title('After restoration (Total)','Color','y','FontSize', 15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
mask_invalid_data = sum(mask_invalid_data,1);
idx_valid_data = find(mask_invalid_data<1);
n_v_data = size(idx_valid_data,2); %valid data의 개수

X_0_     = cell(size(idx_valid_data,2),1);
X_1_     = cell(size(idx_valid_data,2),1);
X_0_ch_  = cell(size(idx_valid_data,2),1);
X_1_ch_  = cell(size(idx_valid_data,2),1);
plane_0_ = cell(size(idx_valid_data,2),1);
plane_1_ = cell(size(idx_valid_data,2),1);
k=1;
for i = idx_valid_data
    X_0_{k}    = X_0{i,1};
    X_1_{k}    = X_1{i,1};
    X_0_ch_{k} = X_0_ch{i,1};
    X_1_ch_{k} = X_1_ch{i,1};
    plane_0_{k} = plane_0{i};
    plane_1_{k} = plane_1{i};
    k=k+1;
end

% break, if (# of points < 4)
if length(X_0_) < 4
    fprintf('Error: not enough valid measuremetns');
    return;
end

%% =======================================================================
%% =======================================================================
%% =======================================================================
%% ===================== Relative Pose Estimation ========================
%% =======================================================================
%% =======================================================================
%% =======================================================================
%% Normal vectors & center initialization
n_0 = cell(n_v_data,1); center_0 = cell(n_v_data,1);
n_1 = cell(n_v_data,1); center_1 = cell(n_v_data,1);

for i=1:n_v_data
    n_0{i} = plane_0_{i}'; %row -> col
    n_1{i} = plane_1_{i}'; %row -> col
    center_0{i} = mean(X_0_{i},2);
    center_1{i} = mean(X_1_{i},2);
end

%% Find initial rotation between LiDARs by Kabsch algorithm (plane normal version)
R_01 = solveKabsch(n_0 , n_1);
R_10 = R_01^-1;
xi = se3Log([R_10 [0 0 0]';0 0 0 1]);

%% Estimate an initial 6-DoF Pose between LiDARs
n_1_warp = cell(n_v_data,1);
J = zeros(4*n_v_data,6);
r = zeros(4*n_v_data,1);

iter=1;
cost_prev = 1e10;
cost_curr = 0;
cost_save=[];
huber_thr = 0.006; %0.006;

while 1
    % rotational matrix
    xi_save(:,iter) = xi;
    
    T_10 = se3Exp(xi);
    
    % rotated normal vector
    for i = 1:n_v_data
        n_1_warp{i,1} = T_10'*n_1{i};
    end
    
    % Calculate Jacobian (don't forget minus(-) )
    for i = 1:n_v_data
        J(4*(i-1)+1,:) = [0, 0, 0, n_1{i}(3)*T_10(2,1)-n_1{i}(2)*T_10(3,1), ...
            n_1{i}(1)*T_10(3,1)-n_1{i}(3)*T_10(1,1), n_1{i}(2)*T_10(1,1)-n_1{i}(1)*T_10(2,1)];
        J(4*(i-1)+2,:) = [0, 0, 0, n_1{i}(3)*T_10(2,2)-n_1{i}(2)*T_10(3,2), ...
            n_1{i}(1)*T_10(3,2)-n_1{i}(3)*T_10(1,2), n_1{i}(2)*T_10(1,2)-n_1{i}(1)*T_10(2,2)];
        J(4*(i-1)+3,:) = [0, 0, 0, n_1{i}(3)*T_10(2,3)-n_1{i}(2)*T_10(3,3), ...
            n_1{i}(1)*T_10(3,3)-n_1{i}(3)*T_10(1,3), n_1{i}(2)*T_10(1,3)-n_1{i}(1)*T_10(2,3)];
        J(4*(i-1)+4,:) = [n_1{i}(1), n_1{i}(2), n_1{i}(3), n_1{i}(3)*T_10(2,4)-n_1{i}(2)*T_10(3,4), ...
            n_1{i}(1)*T_10(3,4)-n_1{i}(3)*T_10(1,4), n_1{i}(2)*T_10(1,4)-n_1{i}(1)*T_10(2,4)];
    end
    
    % Calculate residua l
    for i = 1:n_v_data
        r(4*(i-1)+1) = n_1_warp{i}(1) - n_0{i}(1);
        r(4*(i-1)+2) = n_1_warp{i}(2) - n_0{i}(2);
        r(4*(i-1)+3) = n_1_warp{i}(3) - n_0{i}(3);
        r(4*(i-1)+4) = n_1_warp{i}(4) - n_0{i}(4);
    end

    if(iter>1)
        W = ones(size(r));
        over_ind = abs(r) > huber_thr;
        W(over_ind) = huber_thr ./ abs(r(over_ind));
        JW = bsxfun(@times,J,W);
        H  = JW.'*J;
        delta_xi = -(H+(1e-5)*eye(size(H)))^-1*JW'*r;
    else
        W = ones(size(r));
        H  = J.'*J;
        delta_xi = -(H+(1e-5)*eye(size(H)))^-1*J'*r;
    end
    expm_xi = se3Exp(delta_xi)*se3Exp(xi);
    xi      = se3Log(expm_xi);

    residual_curr = norm(W.*r);
    cost_save = [cost_save, residual_curr];
    cost_curr = residual_curr;
    
    if(norm(delta_xi) <= 1e-7)
        figure();
        plot(xi_save(1,:)); hold on;
        plot(xi_save(2,:));
        plot(xi_save(3,:));
        plot(xi_save(4,:));
        plot(xi_save(5,:));
        plot(xi_save(6,:));
        xlabel('iteration');
        xlim([1,size(xi_save,2)]);
        title('se(3) optimization');
        legend('xi1','xi2','xi3','xi4','xi5','xi6');
        break;
    end
    
    cost_prev = cost_curr;
    
    iter=iter+1;
end
T_01 = T_10^-1;
xi = se3Log(T_01);

%% Bi dir
ab_1 = repmat([0], n_ring,1);
ab_0 = repmat([0], n_ring,1);

X_0_p_warp = cell(n_v_data,1);
X_0_p = cell(n_v_data,1);
X_1_p_warp = cell(n_v_data,1);
X_1_p = cell(n_v_data,1);

n_pts0 = 0;
for i=1:length(X_0_)
    n_pts0 = n_pts0 + length(X_0_{i}); % # of valid 3D points
end
n_pts1 = 0;
for i=1:length(X_1_)
    n_pts1 = n_pts1 + length(X_1_{i}); % # of valid 3D points
end


J = zeros(n_pts0 + n_pts1 , 6+n_ring+n_ring);
r = zeros(n_pts0 + n_pts1 , 1);
r_total = [];

iter=1;
cost_save=[];
huber_thr = 0.005;

s_save=[];
while 1
    s_save(:,iter) = [xi; ab_0; ab_1];
    
    T_10 = T_01^-1;
    
    % rotated normal vector
    for i = 1:n_v_data
        [theta, psi] = theta_psi_generator(X_0_{i});
        for j=1:length(X_0_{i})
            X_0_p{i,1}(:,j) = X_0_{i}(:,j) + ...
                ab_0(X_0_ch_{i, 1}(j,1), 1) * [cos(theta(j))*cos(psi(j)); cos(theta(j))*sin(psi(j)); sin(theta(j))];
        end
        X_0_p_warp{i,1} = T_10*[X_0_p{i}; ones(1, length(X_0_p{i}) )];
    end
    
    for ii = 1:n_v_data
        [theta, psi] = theta_psi_generator(X_1_{ii});
        for j=1:length(X_1_{ii})
            X_1_p{ii,1}(:,j) = X_1_{ii}(:,j) + ...
                ab_1(X_1_ch_{ii, 1}(j,1), 1) * [cos(theta(j))*cos(psi(j)); cos(theta(j))*sin(psi(j)); sin(theta(j))];
        end
        X_1_p_warp{ii,1} = T_01*[X_1_p{ii}; ones(1, length(X_1_p{ii}) )];
    end
    
    % Calculate Jacobian (don't forget minus(-) )
    start_row = 0;
    for i = 1:n_v_data
        [theta, psi] = theta_psi_generator(X_1_{i});
        for j=1:length(X_1_{i})
            M1 = [1 0 0 0 X_1_p_warp{i}(3,j) -X_1_p_warp{i}(2,j); ...
                0 1 0 -X_1_p_warp{i}(3,j) 0 X_1_p_warp{i}(1,j); ...
                0 0 1 X_1_p_warp{i}(2,j) -X_1_p_warp{i}(1,j) 0; ...
                0 0 0 0 0 0];
            M2 = zeros(4, n_ring + n_ring);
            M2(1:3, n_ring + X_1_ch_{i, 1}(j,1)) =...
                T_01(1:3,1:3) * [cos(theta(j))*cos(psi(j)), cos(theta(j))*sin(psi(j)), sin(theta(j))]';
            M = [M1, M2];
            J(start_row + j,:) = plane_0_{i} * M;
        end
        start_row = start_row + length(X_1_{i});
    end
    
    R_10 = T_10(1:3,1:3);
    
    for i = 1:n_v_data
        [theta, psi] = theta_psi_generator(X_0_{i});
        for j=1:length(X_0_{i})
            M1 = [ -R_10, [-R_10(7)*X_0_p{i}(2,j)+R_10(4)*X_0_p{i}(3,j) , R_10(7)*X_0_p{i}(1,j)-R_10(1)*X_0_p{i}(3,j) , -R_10(4)*X_0_p{i}(1,j)+R_10(1)*X_0_p{i}(2,j);...
                -R_10(8)*X_0_p{i}(2,j)+R_10(5)*X_0_p{i}(3,j) , R_10(8)*X_0_p{i}(1,j)-R_10(2)*X_0_p{i}(3,j) , -R_10(5)*X_0_p{i}(1,j)+R_10(2)*X_0_p{i}(2,j);...
                -R_10(9)*X_0_p{i}(2,j)+R_10(6)*X_0_p{i}(3,j) , R_10(9)*X_0_p{i}(1,j)-R_10(3)*X_0_p{i}(3,j) , -R_10(6)*X_0_p{i}(1,j)+R_10(3)*X_0_p{i}(2,j)];...
                0, 0, 0, 0 , 0, 0];
            M2 = zeros(4, n_ring+ n_ring);
            M2(1:3, X_0_ch_{i, 1}(j,1)) =...
                T_10(1:3,1:3) * [cos(theta(j))*cos(psi(j)), cos(theta(j))*sin(psi(j)), sin(theta(j))]';
            M = [M1, M2];
            J(start_row + j,:) = plane_1_{i} * M;
        end
        start_row = start_row + length(X_0_{i});
    end
    
    % Calculate residual
    start_row = 0;
    for i = 1:n_v_data
        for j=1:length(X_1_{i})
            r(start_row + j) = plane_0_{i} *  X_1_p_warp{i,1}(:,j);
        end
        start_row = start_row + length(X_1_{i});
    end
    
    for i = 1:n_v_data
        for j=1:length(X_0_{i})
            r(start_row + j) = plane_1_{i} *  X_0_p_warp{i,1}(:,j);
        end
        start_row = start_row + length(X_0_{i});
    end
    
    r_total(:,iter)=r;
    r_norm_total(:,iter)= norm(r);
    
    if(iter>1)
        
        W = ones(size(r));
        over_ind = abs(r) > huber_thr;
        W(over_ind) = huber_thr ./ abs(r(over_ind));
        JW = bsxfun(@times,J,W);
        H  = JW.'*J;
        delta_s = -(H+(1e-5)*eye(size(H)))^-1*JW'*r;
    else
        W = ones(size(r));
        H  = J.'*J;
        delta_s = -(H+(1e-5)*eye(size(H)))^-1*J'*r;
    end
    expm_xi = se3Exp(delta_s(1:6,1))*se3Exp(xi);
    xi = se3Log(expm_xi);
    ab_0 = ab_0 + delta_s(7:6+n_ring,1);
    ab_1 = ab_1 + delta_s(6+n_ring+1: end,1);
    
    residual_curr = norm(W.*r);
    cost_save = [cost_save, residual_curr];
    
    T_01 = se3Exp(xi);
    
    plane_1_p=cell(n_v_data,1);
    plane_0_p=cell(n_v_data,1);
    for i = 1: n_v_data
        [plane,~,~,~] = plane_reweight(X_1_p{i},[],1,i,0);
        plane_1_p{i} = plane;
    end
    for i = 1: n_v_data
        [plane,~,~,~] = plane_reweight(X_0_p{i},[],1,i,0);
        plane_0_p{i} = plane;
    end
    
    plane_0_ = plane_0_p;
    plane_1_ = plane_1_p;
    
    if(norm(delta_s) <= 1e-3)
        figure();
        plot(s_save(1,:)); hold on;
        plot(s_save(2,:));
        plot(s_save(3,:));
        plot(s_save(4,:));
        plot(s_save(5,:));
        plot(s_save(6,:));
        xlabel('iteration');
        xlim([1,size(s_save,2)]);
        title('se(3) optimization');
        legend('xi1','xi2','xi3','xi4','xi5','xi6');
        break;
    end
    
    iter=iter+1;
end
    
%% Resulting relative pose between two LiDARs.
T_01 = se3Exp(xi);
R_01 = T_01(1:3,1:3);
t_01 = T_01(1:3,4);

%% visualization 1- extracted 3D planes
scr_visualization1;

%% visualization 2- aligned point clouds with range offset compensations
scr_visualization2;

%% visualization 3- aligned point clouds without range offset compensations
scr_visualization3;