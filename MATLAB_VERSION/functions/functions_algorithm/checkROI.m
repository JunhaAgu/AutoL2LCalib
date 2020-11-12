function [out_flag, mask_invalid_data] = checkROI(detection_step,roi, mask_invalid_data, m,n)
if(strcmp(detection_step,'foreground_outsup'))
    if nnz(roi)>50
        out_flag = 0;
    else
        out_flag = 1;
        mask_invalid_data(m,n) = 1;
        fprintf('==========> Lidar%d data%d: Insufficient number of ROI after <foreground_outsup>\n',m-1,n);
    end
elseif(strcmp(detection_step,'restore_channel'))
    if nnz(roi)>50 % at least
        out_flag = 0;
    else
        out_flag = 1;
        mask_invalid_data(m,n) = 1;
        fprintf('==========> Lidar%d data%d: Insufficient number of ROI after <restoration>\n',m-1,n);
    end
end

end