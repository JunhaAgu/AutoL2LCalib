function ind = findCondition(pcl, x_lims, y_lims, z_lims, dist_lims, i_lims)


mask_intensity = and(pcl.Intensity >= i_lims(1),pcl.Intensity <= i_lims(2));
mask_x = and(pcl.Location(1,:) >= x_lims(1),pcl.Location(1,:) <= x_lims(2));
mask_y = and(pcl.Location(2,:) >= y_lims(1),pcl.Location(2,:) <= y_lims(2));
mask_z = and(pcl.Location(3,:) >= z_lims(1),pcl.Location(3,:) <= z_lims(2));
mask_dist = (pcl.Location(1,:).^2 + pcl.Location(2,:).^2 + pcl.Location(3,:).^2) > dist_lims*ones(1,size(pcl.Location,2));

mask = and(and(mask_x,mask_y), and(mask_z,mask_intensity));
mask = and(mask, mask_dist);
ind = find(mask);

end