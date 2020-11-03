function [out_flag, mask_invalid_data] = check_n_pts(step,roi, mask_invalid_data, m,n)
if(strcmp(step,'foreground_outsup'))
    if nnz(roi)>50
        out_flag = 0;
    else
        out_flag = 1;
        mask_invalid_data(m,n) = 1;
        fprintf('==========> Lidar%d data %d lack of ROI after <foreground_outsup>: 0\n',m-1,n);
    end
elseif(strcmp(step,'restore_channel'))
    if nnz(roi)>50 % at least
        out_flag = 0;
    else
        out_flag = 1;
        mask_invalid_data(m,n) = 1;
        fprintf('==========> Lidar%d data %d after restoration: <50 \n',m-1,n);
    end
end

end