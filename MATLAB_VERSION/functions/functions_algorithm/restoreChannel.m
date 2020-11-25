function [roi_out, roi_3D_pts, roi_imgs_range] = restoreChannel(roi, imgs_range, imgs_index, data, m, n)
%%%%% input %%%%%
% roi            : logical      -> n_ring x n_step
% imgs_range     : range value
% imgs_index     : correnspondence to imgs_range
% data           : all data structure
% m              : which lidar
% n              : which sequence

%%%%% output %%%%%
% roi_out        : logical      -> n_ring x n_step
% roi_3D_pts     : 3D points    -> 3 X M (M<=N)
% roi_imgs_range : range value  -> n_ring x n_step
if m==1
    spec_acc = data.l0.spec.accuracy;
elseif m==2
    spec_acc = data.l1.spec.accuracy;
end

img_range = imgs_range{m,n};

[ch, ~]     = find(roi>0);
ch_max_init = max(ch);
idx_ch      = zeros(ch_max_init,1);
for i_ch = 1:size(ch,1)
   idx_ch(ch(i_ch)) = 1;
end
valid_ch = find(idx_ch>0)'; %row-wise

roi_out        = zeros(size(roi));
roi_imgs_range = zeros(size(img_range));

for i_ch = valid_ch
   one_ch = roi(i_ch,:)>0;
   valid_az_step = find(one_ch==1);
   
   for az_step = valid_az_step
      % search in a channel
      if roi_out(i_ch,az_step)==0
         cur_az_step = az_step;
         cur_range_val = img_range(i_ch,cur_az_step);
         
         if cur_az_step~=1
            roi_out(i_ch,cur_az_step)=1;
            roi_imgs_range(i_ch,cur_az_step) = cur_range_val;
            
            pre_range_val = cur_range_val;
            pre_az_step      = cur_az_step;
            while 1 % search left direction first
               if pre_az_step==1
                  break
               end
               left_range_val = img_range(i_ch,pre_az_step-1);
               if abs(left_range_val-pre_range_val) > 3*(spec_acc+0.2*pi/180*pre_range_val) %change_val
                  break
               end
               roi_out(i_ch,pre_az_step-1)=1;
               roi_imgs_range(i_ch,pre_az_step-1)=left_range_val;
               pre_range_val = left_range_val;
               pre_az_step      = pre_az_step-1;
            end
            
            %search right direction
            pre_range_val = cur_range_val;
            pre_az_step      = cur_az_step;
            while 1
               if pre_az_step==size(roi,2)
                  break
               end
               right_range_val = img_range(i_ch,pre_az_step+1);
               if abs(right_range_val-pre_range_val) > 3*(spec_acc+0.2*pi/180*pre_range_val) %change_val
                  break
               end
               roi_out(i_ch,pre_az_step+1)=1;
               roi_imgs_range(i_ch,pre_az_step+1)=right_range_val;
               pre_range_val = right_range_val;
               pre_az_step      = pre_az_step+1;
            end
         end
      end
   end %i_step
end %i_ch

%% validate whether # of points is sufficient in a channel.
for i=valid_ch
   if sum(roi_out(i,:))<10 %50
      roi_out(i,:)=0;
      roi_imgs_range(i,:)=0;
   end
end

if nnz(sum(roi_out,2)>0)<3
   roi_out        = zeros(size(roi));
   roi_imgs_range = zeros(size(img_range));
end

% foreground region only.
mid = sum(roi_imgs_range(:))/nnz(roi_out);
idx_forg = roi_imgs_range(:) < mid+2;
roi_out = and(roi_out, reshape(idx_forg,size(roi_out)));
roi_imgs_range(idx_forg<1) = 0;


% 3D pts output
idx_pts = imgs_index{m,n}(roi_out>0);
roi_3D_pts = data.pcls{m, n}.Location(:, idx_pts);

end