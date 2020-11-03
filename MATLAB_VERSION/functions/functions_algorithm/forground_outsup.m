function [roi, roi_3D_pts] = forground_outsup(data, imgs_rho,imgs_index,m,n,ref_seq)

        diff_rho=imgs_rho{m,n}-imgs_rho{m,ref_seq};
        
        mask_diff_rho = and((diff_rho<-0.2),imgs_rho{m,n}>1.5); %Â÷ÀÌ>1[m] 
        valid_diff_rho = diff_rho.*mask_diff_rho;
                
        sd = strel('square',3);
        mask_image_e = imerode(mask_diff_rho, sd);
        mask_image_d = imdilate(mask_image_e, sd);
             
        roi = mask_image_d;
        
        idx_pts = imgs_index{m,n}(roi>0);
        roi_3D_pts = data.pcls{m, n}.Location(:, idx_pts);
end