function [out_flag, mask_invalid_data] = check_ransac_plane(plane_a, plane_b,mask_invalid_data, m,n)
a = plane_a(1:3)./norm(plane_a(1:3));
b = plane_b(1:3)./norm(plane_b(1:3));

if abs(dot(a,b))>0.9
    out_flag = 0;
else
    out_flag = 1;
    mask_invalid_data(m,n) = 1;
    fprintf('==========> Lidar%d data %d a lot of outliers \n',m-1,n);
end
end