del_rho0 = data.l0.del_rho0;
del_rho1 = data.l1.del_rho1;

X_1_rot_all  = cell(data.n_data,1);

% range limits
x_lims = [-20,20];
y_lims = [-20,20];
z_lims = [-20,20];
i_lims = [0,255]; % reflection intensity range.
dist_lims = 1.5;

X_lidar = cell(size(data.pcls,1),data.n_data);
X_ring  = cell(size(data.pcls,1),data.n_data);

for n = 1:data.n_data
    idxs = findCondition(data.pcls{1,n}, x_lims, y_lims, z_lims, dist_lims, i_lims);
    X    = data.pcls{1,n}.Location(:,idxs);
    X_lidar{1,n} = X;
    X_ring{1,n} = data.pcls{1,n}.Ring(1,idxs);
    idxs = findCondition(data.pcls{2,n}, x_lims, y_lims, z_lims, dist_lims, i_lims);
    X    = data.pcls{2,n}.Location(:,idxs);
    X_lidar{2,n} = X;
    X_ring{2,n} = data.pcls{2,n}.Ring(1,idxs);
end


for i = 1:data.n_data
    X_1_rot_all{i} = R_01*X_lidar{2, i} + repmat(t_01,1,length(X_lidar{2, i}) ); %X_1_rot{i} = T_01*X_1{i}
end

X_0_all_p = cell(data.n_data,1);
for i = 1:data.n_data
    [theta, psi] = generateThetaPsi(X_lidar{1, i});
    for j=1:length(X_lidar{1, i})
        X_0_all_p{i,1}(:,j) = X_lidar{1, i}(:,j) + ...
            del_rho0(X_ring{1,i}(1,j)+1, 1) * [cos(theta(j))*cos(psi(j)); cos(theta(j))*sin(psi(j)); sin(theta(j))];
    end
end

X_1_all_p = cell(data.n_data,1);
X_1_all_p_warp = cell(data.n_data,1);
for i = 1:data.n_data
    [theta, psi] = generateThetaPsi(X_lidar{2, i});
    for j=1:length(X_lidar{2, i})
        X_1_all_p{i,1}(:,j) = X_lidar{2, i}(:,j) + ...
        del_rho1(X_ring{2,i}(1,j)+1, 1) * [cos(theta(j))*cos(psi(j)); cos(theta(j))*sin(psi(j)); sin(theta(j))];
    end
    X_1_all_p_warp{i,1} = T_01*[X_1_all_p{i}; ones(1, length(X_1_all_p{i}) )];
end

%% you can see all the results
for i = 1 %1:data.n_data
    
    figure();

    n_pts = length(X_0_all_p{i, 1});
    hues = 0.4*ones(n_pts,1);
    colors = hsv2rgb([hues,ones(n_pts,1),ones(n_pts,1)]);
    markersizes = 1*ones(n_pts,1);
    scatter3(X_0_all_p{i, 1}(1,:),X_0_all_p{i, 1}(2,:),X_0_all_p{i, 1}(3,:), markersizes,colors);
    hold on;

    n_pts = length(X_1_all_p_warp{i,1});
    hues = 0.6*ones(n_pts,1);
    colors = hsv2rgb([hues,ones(n_pts,1),ones(n_pts,1)]);
    markersizes = 1*ones(n_pts,1);
    scatter3(X_1_all_p_warp{i,1}(1,:),X_1_all_p_warp{i,1}(2,:),X_1_all_p_warp{i,1}(3,:), markersizes,colors);
    
    plot3(0,0,0,'g*','MarkerSize',10,'LineWidth',1);
    plot3(0,0,0,'gs','MarkerSize',10,'LineWidth',1);
    plot3(t_01(1,1),t_01(2,1),t_01(3,1),'b*','MarkerSize',10,'LineWidth',1);
    plot3(t_01(1,1),t_01(2,1),t_01(3,1),'bs','MarkerSize',10,'LineWidth',1);
    axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title([ num2str(i) 'th warping: Green(LiDAR0), Blue(LiDAR1) w/ offset model'] ,'Color', 'y','FontSize', 15);
    grid on;
    set(gcf,'Color','k'); set(gca,'Color','k'); set(gca,'xcolor','w'); set(gca,'ycolor','w'); set(gca,'zcolor','w')
    view(0,90);
end
