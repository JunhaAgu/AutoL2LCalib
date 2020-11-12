close all; clear all; clc;
dbstop error

%%
addpath('functions/functions_dataload');
addpath('functions/functions_transform');
addpath('functions/functions_algorithm');

%% global
global D2R R2D
D2R = pi/180; R2D = 180/pi;

%% User-define parameters
%lidar0 hardware spec
l0_n_ring = 16;
l0_azimuth_res = 0.2;
l0_elevation_res = 2;
l0_elevation_min = -15;
%lidar1 hardware spec
l1_n_ring = 16;
l1_azimuth_res = 0.2;
l1_elevation_res = 2;
l1_elevation_min = -15;

lim.ang = [-3/4*pi(), 3/4*pi()]; %-pi ~ +pi
ref_seq = 1 ;

%% Path to datasets
dir_to_dataset = 'D:\lidar_plane_data\datasets\config_1\'; %'config_1', 'config_2', 'config_3', 'config_4', 'config_perpendicular'
dataset_name   = 'chess'; %'chess', 'cut', 'long'
data_type      = '1cams2lidars';

data = loadData(dir_to_dataset, dataset_name, data_type); % load all data in a dataset
data = sortRings(data); % generate ".pcls_rings", ".IndexRings" in "data".

fprintf('# of data: %d\n', data.n_data);
fprintf('==============================================\n')

data.l0.spec.n_ring = l0_n_ring;
data.l0.spec.azimuth_res = l0_azimuth_res;
data.l0.spec.elevation_res = l0_elevation_res;
data.l0.spec.elevation_min = l0_elevation_min;
data.l1.spec.n_ring = l1_n_ring;
data.l1.spec.azimuth_res = l1_azimuth_res;
data.l1.spec.elevation_res = l1_elevation_res;
data.l1.spec.elevation_min = l1_elevation_min;

%% generate lidar image
imgs_intensity = cell(2,data.n_data);
imgs_rho       = cell(2,data.n_data);
imgs_index     = cell(2,data.n_data);

for m = 1:2
    for n = 1:data.n_data
        [imgs_intensity{m,n}, imgs_rho{m,n}, imgs_index{m,n}] = generateLidarImages(data, lim, m, n);
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

mask_invalid_data = false(2,data.n_data);

%interest_data
i_data = 1:data.n_data;
n_i_data = length(i_data);

for m = 1:2
    for n = i_data %1:data.n_data
        roi = [];
        
        %% foreground and outliers suppression
        [roi, roi_3D_pts] = suppOutliersAndExtForeground(data, imgs_rho, imgs_index, ref_seq, m, n);
        
        %check validity
        [out_flag, mask_invalid_data] = checkROI('foreground_outsup', roi, mask_invalid_data, m, n);
        if out_flag
            continue;
        end
        
        %% Target detection & completion
        %ransac pramaters
        ransac.param.iter   = 50;
        ransac.param.thr    = 0.02; %3cm
        ransac.param.inlier = 0.7*size(roi_3D_pts,2);
        
        %% RANSAC
        [plane_a,~,~, roi] = ransacPlane(roi_3D_pts, roi, ransac.param);
        
        %% Restoration
        % roi_imgs_range --> for visualization
        [roi, roi_3D_pts, roi_imgs_range{m,n}] = restoreChannel(roi, imgs_rho, imgs_index, data, m, n);
        
        [out_flag, mask_invalid_data] = checkROI('restore_channel',roi, mask_invalid_data, m,n);
        if out_flag
            continue;
        end

        %% RANSAC        
        [plane_b,roi_3D_pts,~,roi] = ransacPlane(roi_3D_pts, roi, ransac.param);
        
        [out_flag , mask_invalid_data] = checkRansacPlane(plane_a, plane_b, mask_invalid_data, m,n);
        if out_flag
            continue;
        end

        %% Plane_reweight
        [plane, ~, mask_reweight, roi] = fitPlaneReweight(roi_3D_pts,roi,1);
        if m==1
            plane_0{n} = plane;
        elseif m==2
            plane_1{n} = plane;
        end
        
        [out_flag, mask_invalid_data,roi,roi_3D_pts] = ...
            checkRatioPts('plane_reweight',mask_reweight, mask_invalid_data,roi, imgs_index, data, m,n);
        if out_flag
            continue;
        end
        
        scr_visualization_extract;
        
        %%
        [ch, ~] = find(roi>0);
        
        if m==1
            X_0{n} = roi_3D_pts;
            X_0_ch{n} = ch;
        elseif m==2
            X_1{n} = roi_3D_pts;
            X_1_ch{n} = ch;
        end
        
    end %n
end %m

fprintf('================ Invalid data ================\n')
fprintf('[1]-th lidar: ')
fprintf('%d ',find(mask_invalid_data(1,:)==1))
fprintf('\n')
fprintf('[2]-th lidar: ')
fprintf('%d ',find(mask_invalid_data(2,:)==1))
fprintf('\n')
fprintf('==============================================\n')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
mask_invalid_data = sum(mask_invalid_data,1);
idx_valid_data = find(mask_invalid_data<1);
n_valid_data = size(idx_valid_data,2); % # of valid data
data.n_valid_data = n_valid_data;

data.l0.X_0_valid     = X_0(idx_valid_data,1);
data.l1.X_1_valid     = X_1(idx_valid_data,1);
data.l0.X_0_ch_valid  = X_0_ch(idx_valid_data,1);
data.l1.X_1_ch_valid  = X_1_ch(idx_valid_data,1);
data.l0.plane_0_valid = plane_0(idx_valid_data,1);
data.l1.plane_1_valid = plane_1(idx_valid_data,1);

% break, if (# of points < 4)
if length(data.l0.X_0_valid) < 4
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
data.l0.n_0 = cell(n_valid_data,1); data.l0.center_0 = cell(n_valid_data,1);
data.l1.n_1 = cell(n_valid_data,1); data.l1.center_1 = cell(n_valid_data,1);

for i=1:n_valid_data
    data.l0.n_0{i} = data.l0.plane_0_valid{i}'; %row -> col
    data.l1.n_1{i} = data.l1.plane_1_valid{i}'; %row -> col
    data.l0.center_0{i} = mean(data.l0.X_0_valid{i},2);
    data.l1.center_1{i} = mean(data.l1.X_1_valid{i},2);
end

%% Find initial rotation between LiDARs by Kabsch algorithm (plane normal version)
R_01 = solveKabsch(data);

%% Estimate an initial 6-DoF Pose between LiDARs
[xi_01, T_01] = initialOptimization(R_01, data);

%% Bi directional optimization
[xi_01, T_01, data] = biDirecOptimization(xi_01, T_01, data);

%% Resulting relative pose between two LiDARs.
R_01 = T_01(1:3,1:3);
t_01 = T_01(1:3,4);

%% visualization 1- extracted 3D planes
scr_visualization1;

%% visualization 2- aligned point clouds with range offset compensations
scr_visualization2;

%% visualization 3- aligned point clouds without range offset compensations
% scr_visualization3;