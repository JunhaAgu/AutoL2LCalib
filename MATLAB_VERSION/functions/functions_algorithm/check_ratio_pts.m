function [out_flag, mask_invalid_data,roi,roi_3D_pts] = check_ratio_pts(step,mask_reweight, mask_invalid_data, roi, imgs_index, data, m,n)
if(strcmp(step,'plane_reweight'))
    if nnz(mask_reweight)/size(mask_reweight,2)  > 0.80 || ~isempty(mask_reweight)
        out_flag = 0;
            idx_out = sum(roi,2)<30;
            roi(idx_out>0,:) = 0;
            roi_3D_pts = data.pcls{m, n}.Location(:, imgs_index{m,n}(find(roi>0)));
            
            if nnz(roi) == 0
                out_flag = 1;
                mask_invalid_data(m,n) = 1;
                fprintf('==========> Lidar%d data %d after reweight, no valid channel\n',m-1,n);
            end

    else
        out_flag = 1;
        mask_invalid_data(m,n) = 1;
        fprintf('==========> Lidar%d data %d after reweight inlier ratio: < 0.6\n',m-1,n);
    end

    
    
end

end