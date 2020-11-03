
figure(); hold on;

for i = 1:n_v_data
    n_pts = length(X_0_p{i});
    hues = 0.0*ones(1,n_pts);
    colors = hsv2rgb([hues;ones(1,n_pts);ones(1,n_pts)]');
    scatter3(X_0_p{i}(1,:),X_0_p{i}(2,:),X_0_p{i}(3,:),2*ones(1,n_pts), colors); hold on;

    n_pts = length(X_1_p_warp{i});
    hues = 0.6*ones(1,n_pts);
    colors = hsv2rgb([hues;ones(1,n_pts);ones(1,n_pts)]');
    scatter3(X_1_p_warp{i}(1,:),X_1_p_warp{i}(2,:),X_1_p_warp{i}(3,:),2*ones(1,n_pts), colors); hold on;
end

for i = 1:n_v_data
    plot3(center_0{i}(1)+[0,n_0{i}(1)/n_0{i}(4)],center_0{i}(2)+[0,n_0{i}(2)/n_0{i}(4)],center_0{i}(3)+[0,n_0{i}(3)/n_0{i}(4)],'m','LineWidth',2);
end
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Red(LiDAR0), Blue(LiDAR1)' ,'Color', 'y');
grid on;
set(gcf,'Color','k'); set(gca,'Color','k'); set(gca,'xcolor','w'); set(gca,'ycolor','w'); set(gca,'zcolor','w')


figure(); hold on;
for i = 1:n_v_data
    n_pts = length(X_0_{i});
    hues = 0.0*ones(1,n_pts);
    colors = hsv2rgb([hues;ones(1,n_pts);ones(1,n_pts)]');
    scatter3(X_0_{i}(1,:),X_0_{i}(2,:),X_0_{i}(3,:),2*ones(1,n_pts), colors); hold on;
    
    n_pts = length(X_0_p{i});
    hues = 0.6*ones(1,n_pts);
    colors = hsv2rgb([hues;ones(1,n_pts);ones(1,n_pts)]');
    scatter3(X_0_p{i}(1,:),X_0_p{i}(2,:),X_0_p{i}(3,:),2*ones(1,n_pts), colors); hold on;
end
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Lidar0: Red(Before), Blue(After)' ,'Color', 'y');
grid on;
set(gcf,'Color','k'); set(gca,'Color','k'); set(gca,'xcolor','w'); set(gca,'ycolor','w'); set(gca,'zcolor','w')

figure(); hold on;
for i = 1:n_v_data
    n_pts = length(X_1_{i});
    hues = 0.0*ones(1,n_pts);
    colors = hsv2rgb([hues;ones(1,n_pts);ones(1,n_pts)]');
    scatter3(X_1_{i}(1,:),X_1_{i}(2,:),X_1_{i}(3,:),2*ones(1,n_pts), colors); hold on;
    
    n_pts = length(X_1_p{i});
    hues = 0.6*ones(1,n_pts);
    colors = hsv2rgb([hues;ones(1,n_pts);ones(1,n_pts)]');
    scatter3(X_1_p{i}(1,:),X_1_p{i}(2,:),X_1_p{i}(3,:),2*ones(1,n_pts), colors); hold on;
end
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Lidar1: Red(Before), Blue(After)' ,'Color', 'y');
grid on;
set(gcf,'Color','k'); set(gca,'Color','k'); set(gca,'xcolor','w'); set(gca,'ycolor','w'); set(gca,'zcolor','w')
% view(-40,30);
% xlim([0 4]); ylim([-2 2]); zlim([-0.5 1]);
