for i = 1:data.n_data
    
    figure();
    
    n_pts = length(X_lidar{1, i});
    hues = 0.4*ones(n_pts,1);
    colors = hsv2rgb([hues,ones(n_pts,1),ones(n_pts,1)]);
    markersizes = 3*ones(n_pts,1);
    scatter3(X_lidar{1, i}(1,:),X_lidar{1, i}(2,:),X_lidar{1, i}(3,:), markersizes,colors);
    hold on;

    n_pts = length(X_1_rot_all{i});
    hues = 0.6*ones(n_pts,1);
    colors = hsv2rgb([hues,ones(n_pts,1),ones(n_pts,1)]);
    markersizes = 3*ones(n_pts,1);
    scatter3(X_1_rot_all{i}(1,:),X_1_rot_all{i}(2,:),X_1_rot_all{i}(3,:), markersizes,colors);
    
    plot3(0,0,0,'m*','MarkerSize',10,'LineWidth',1);
    plot3(0,0,0,'ms','MarkerSize',10,'LineWidth',1);
    plot3(t_01(1,1),t_01(2,1),t_01(3,1),'g*','MarkerSize',10,'LineWidth',1);
    plot3(t_01(1,1),t_01(2,1),t_01(3,1),'gs','MarkerSize',10,'LineWidth',1);
    axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title([ num2str(i) 'th warping: Green(lidar0), Blue(lidar1)'] ,'Color', 'y','FontSize', 15);
    grid on;
    set(gcf,'Color','k'); set(gca,'Color','k'); set(gca,'xcolor','w'); set(gca,'ycolor','w'); set(gca,'zcolor','w')
    view(0,90);

end