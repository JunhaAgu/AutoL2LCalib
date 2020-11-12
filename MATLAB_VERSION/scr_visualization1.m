n_valid_data = data.n_valid_data;
X_0_ = data.l0.X_0_;
X_1_ = data.l1.X_1_;
X_0_p = data.l0.X_0_p;
X_1_p = data.l1.X_1_p;
X_1_p_warp = data.l1.X_1_p_warp;
center_0 = data.l0.center_0;
n_1_p =  data.l1.n_1_p;
n_0_p =  data.l0.n_0_p;

%%
% With considering delta_rho, points measured by LiDAR0 and points measured
% by LiDAR1 are represented in LiDAR0 coordinate frame.
figure(); hold on;
for i = 1:n_valid_data
    n_pts = length(X_0_p{i});
    hues = 0.4*ones(1,n_pts);
    colors = hsv2rgb([hues;ones(1,n_pts);ones(1,n_pts)]');
    scatter3(X_0_p{i}(1,:),X_0_p{i}(2,:),X_0_p{i}(3,:),2*ones(1,n_pts), colors); hold on;

    n_pts = length(X_1_p_warp{i});
    hues = 0.6*ones(1,n_pts);
    colors = hsv2rgb([hues;ones(1,n_pts);ones(1,n_pts)]');
    scatter3(X_1_p_warp{i}(1,:),X_1_p_warp{i}(2,:),X_1_p_warp{i}(3,:),2*ones(1,n_pts), colors); hold on;
    
    plot3(center_0{i}(1)+[0,n_0_p{i}(1)/2],center_0{i}(2)+[0,n_0_p{i}(2)/2],center_0{i}(3)+[0,n_0_p{i}(3)/2],'g','LineWidth',2);
    plot3(center_0{i}(1)+[0,n_1_p{i}(1)/2],center_0{i}(2)+[0,n_1_p{i}(2)/2],center_0{i}(3)+[0,n_1_p{i}(3)/2],'b','LineWidth',2);
end
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Green(LiDAR0), Blue(LiDAR1)' ,'Color', 'y');
grid on;
set(gcf,'Color','k'); set(gca,'Color','k'); set(gca,'xcolor','w'); set(gca,'ycolor','w'); set(gca,'zcolor','w')

%%
% Results of w/ and w/o range offset model 
figure(); hold on;
for i = 1:n_valid_data
    n_pts = length(X_0_{i});
    hues = 0.4*ones(1,n_pts);
    colors = hsv2rgb([hues;ones(1,n_pts);ones(1,n_pts)]');
    scatter3(X_0_{i}(1,:),X_0_{i}(2,:),X_0_{i}(3,:),2*ones(1,n_pts), colors); hold on;
    
    n_pts = length(X_0_p{i});
    hues = 0.6*ones(1,n_pts);
    colors = hsv2rgb([hues;ones(1,n_pts);ones(1,n_pts)]');
    scatter3(X_0_p{i}(1,:),X_0_p{i}(2,:),X_0_p{i}(3,:),2*ones(1,n_pts), colors); hold on;
end
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Lidar0: Green(No \Delta\rho), Blue(considering \Delta\rho)' ,'Color', 'y');
grid on;
set(gcf,'Color','k'); set(gca,'Color','k'); set(gca,'xcolor','w'); set(gca,'ycolor','w'); set(gca,'zcolor','w')

%%
% Results of w/ and w/o range offset model
figure(); hold on;
for i = 1:n_valid_data
    n_pts = length(X_1_{i});
    hues = 0.4*ones(1,n_pts);
    colors = hsv2rgb([hues;ones(1,n_pts);ones(1,n_pts)]');
    scatter3(X_1_{i}(1,:),X_1_{i}(2,:),X_1_{i}(3,:),2*ones(1,n_pts), colors); hold on;
    
    n_pts = length(X_1_p{i});
    hues = 0.6*ones(1,n_pts);
    colors = hsv2rgb([hues;ones(1,n_pts);ones(1,n_pts)]');
    scatter3(X_1_p{i}(1,:),X_1_p{i}(2,:),X_1_p{i}(3,:),2*ones(1,n_pts), colors); hold on;
end
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Lidar0: Green(No \Delta\rho), Blue(considering \Delta\rho)' ,'Color', 'y');
grid on;
set(gcf,'Color','k'); set(gca,'Color','k'); set(gca,'xcolor','w'); set(gca,'ycolor','w'); set(gca,'zcolor','w')