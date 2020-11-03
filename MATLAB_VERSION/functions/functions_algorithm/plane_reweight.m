% plane fitting using Iterative Reweight Least Square (IRLS)
function [plane,inliers,mask_inlier,roi] = plane_reweight(points,roi,left_right,plane_order, prior_flag)

%%%%% input %%%%%
% points      : 3D points, size     -> 3 x N
% left_right  : 0 - left, 1 - right
% plane_order : 1 or 2 or 3 or ...

%%%%% output %%%%%
% plane       : [A B C D]           -> 1 x 4
% inliers     : inliers             -> 3 x N ; D>0, norm([A B C])!=0
% mask_inlier : 0 or 1              -> N X 1

% n_sample=3; %3 points for one plane
n_pts=length(points); % #_points


if size(points,1)>3
    points=points'; %3 X N
end

d_init=mean(points,2);

%% iterative reweight least square
w = [0 0 0]';
theta=[w',0]'; %[w1,w2,w3,d]'

cost_prev = 1e10;
cost_curr = 0;
cost_save = [];
huber_thr = 0.005; %1[cm]

iter_opt=1;

pts=points;

while 1
    % rotational matrix
    theta_save(:,iter_opt) = theta;
    w = theta(1:3,1);
    R_res = so3Exp(w);
    
    %Jacobian
    for i=1:n_pts
        J(i,:) = [-R_res(3,1)*pts(2,i)+R_res(2,1)*pts(3,i), +R_res(3,1)*pts(1,i)-R_res(1,1)*pts(3,i), -R_res(2,1)*pts(1,i)+R_res(1,1)*pts(2,i), 1];
    end
    
    % Calculate residual
    for i=1:n_pts
        r(i,1) = [1 0 0]*R_res'*pts(:,i)+theta(4,1); % row
    end

    r_norm_total(:,iter_opt)= norm(r);
    
    % update w
    
    if(iter_opt>3)
        W = ones(size(r));
        over_ind = abs(r) > huber_thr;
        W(over_ind) = huber_thr ./ abs(r(over_ind));
        JW = bsxfun(@times,J,W);
        H  = JW.'*J;
        delta_theta = -(H+(1e-5)*eye(size(H)))^-1*JW'*r;
        %delta_theta = -pinv(H)*JW'*r;
    else
        W = ones(size(r));
        H  = J.'*J;
        delta_theta = -(H+(1e-5)*eye(size(H)))^-1*J'*r;
        %delta_theta = -pinv(H)*J'*r;
    end
    
    theta  = theta + delta_theta;
    
    residual_curr = norm(W.*r);
    
    r_total(:,iter_opt)=W.*r;
    
    cost_save = [cost_save, residual_curr];
    cost_curr = residual_curr;
    
    abcd(iter_opt,:) = [ [1 0 0]*R_res' , theta(4,1) ]; %[a b c d] for check
    
    if prior_flag==1
        if(norm(delta_theta) < 1e-5) || iter_opt==1000
            break;
        end
    else
        if(norm(delta_theta) < 1e-5)
            break;
        end
    end
    
    cost_prev = cost_curr;
    
    iter_opt=iter_opt+1;
    
end

if prior_flag==1
    if iter_opt==1000
        plane=[];
        inliers=[];
        mask_inlier=[];
        return;
    end
else
end

iter_fin=iter_opt-1;

 %% output plane
if abcd(iter_fin,4)<0
    plane = -abcd(iter_fin,:);
else
    plane = abcd(iter_fin,:);
end

%%
% inlierÆÇ´Ü
thr=0.02; %3[cm] 
residual=zeros(n_pts,1);
idx_inlier=zeros(n_pts,1);

for i=1:n_pts
    residual(i,1) = abs( plane(1,1)*points(1,i) + plane(1,2)*points(2,i) + plane(1,3)*points(3,i) + plane(1,4) ) /...
        sqrt(plane(1,1)^2+plane(1,2)^2+plane(1,3)^2)  ;
    if residual(i,1) < thr
        idx_inlier(i,1)=1;
    end
end

inliers     = pts(:,idx_inlier>0);
mask_inlier = idx_inlier;

if prior_flag
    idx_roi = find(roi>0);
    roi(idx_roi(mask_inlier<1) ) = 0;
else
    roi=[];
end

%%
idxx=find(abs(plane(1,1:3))==min(abs(plane(1,1:3))));

if idxx==2
    z_max=max(inliers(3,:)); z_min=min(inliers(3,:));
    y_max=max(inliers(2,:)); y_min=min(inliers(2,:));
    [refined_z,~] = meshgrid(linspace(z_min-0.03,z_max+0.03,3));
    [~,refined_y] = meshgrid(linspace(y_min-0.03,y_max+0.03,3));
    refined_x=-(plane(1,3)*refined_z+plane(1,2)*refined_y+plane(1,4) )./plane(1,1) ;
elseif idxx==3
    x_max=max(inliers(1,:)); x_min=min(inliers(1,:));
    z_max=max(inliers(3,:)); z_min=min(inliers(3,:));
    [refined_x,~] = meshgrid(linspace(x_min-0.03,x_max+0.03,3));
    [~,refined_z] = meshgrid(linspace(z_min-0.03,z_max+0.03,3));
    refined_y=-(plane(1,1)*refined_x+plane(1,3)*refined_z+plane(1,4) )./plane(1,2) ;
elseif idxx==1
    x_max=max(inliers(1,:)); x_min=min(inliers(1,:));
    y_max=max(inliers(2,:)); y_min=min(inliers(2,:));
    [refined_x,~] = meshgrid(linspace(x_min-0.03,x_max+0.03,3));
    [~,refined_y] = meshgrid(linspace(y_min-0.03,y_max+0.03,3));
    refined_z=-(plane(1,1)*refined_x+plane(1,2)*refined_y+plane(1,4) )./plane(1,3) ;
end

end