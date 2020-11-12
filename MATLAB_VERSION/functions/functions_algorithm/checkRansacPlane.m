function [out_flag, mask_invalid_data] = checkRansacPlane(plane_a, plane_b,mask_invalid_data, m,n)
a = plane_a(1:3)./norm(plane_a(1:3));
b = plane_b(1:3)./norm(plane_b(1:3));

if abs(dot(a,b))>0.9
    out_flag = 0;
else
    out_flag = 1;
    mask_invalid_data(m,n) = 1;
    fprintf('==========> Lidar%d data%d: wrongly plane detection \n',m-1,n);
end
end