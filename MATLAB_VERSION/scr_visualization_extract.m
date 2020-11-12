% visualization
figure(100); hold on;
n_roi_pts = size(roi_3D_pts,2);
if m==1
    hues = 0.4*ones(1,n_roi_pts);
elseif m==2
    hues = 0.6*ones(1,n_roi_pts);
end
colors = hsv2rgb([hues;ones(1,n_roi_pts);ones(1,n_roi_pts)]');
scatter3(roi_3D_pts(1,:),roi_3D_pts(2,:),roi_3D_pts(3,:),1*ones(1,n_roi_pts), colors); hold on;
set(gcf,'Color','k'); set(gca,'Color','k'); set(gca,'xcolor','w'); set(gca,'ycolor','w'); set(gca,'zcolor','w');
title('Planar Board Extraction (Total)','Color','y','FontSize', 15);
view(-40, 30); axis equal; grid on;
title(num2str(n));