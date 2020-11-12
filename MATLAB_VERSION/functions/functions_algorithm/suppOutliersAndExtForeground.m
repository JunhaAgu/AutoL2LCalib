function [roi, roi_3D_pts] = suppOutliersAndExtForeground(data, imgs_rho,imgs_index,ref_seq,m,n)

        diff_rho=imgs_rho{m,n}-imgs_rho{m,ref_seq};
        
        mask_diff_rho1 = and((diff_rho<-0.2),imgs_rho{m,n}>1.5);
        mask_diff_rho2 = and(imgs_rho{m,n}>1.5,imgs_rho{m,ref_seq}==0);
        mask_diff_rho = or(mask_diff_rho1, mask_diff_rho2);
        
        sd = strel('square',3); %length=3
        mask_image_e = imerode(mask_diff_rho, sd);
        mask_image_d = imdilate(mask_image_e, sd);
             
        roi = mask_image_d;
        
        idx_pts = imgs_index{m,n}(roi>0);
        roi_3D_pts = data.pcls{m, n}.Location(:, idx_pts);
        
end