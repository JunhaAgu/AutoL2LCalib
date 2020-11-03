%ransac
function [plane,inliers,mask_inlier,roi] = ransac_plane(points, roi, iter, thr,mini_inlier, left_right, plane_order) %, x_min , x_max , y_min, y_max, z_min, z_max, n_ROI_las)

%%%%% input %%%%%
% points      : 3D points, size     -> 3 x N
% iter        : maximum iteration   -> scalar
% thr         : distance threshold  -> scalar
% mini_inlier : XX% of points       -> scalar
% left_right  : 0 - left, 1 - right
% plane_order : 1 or 2 or 3 ...

%%%%% output %%%%%
% plane       : plane coefficient    -> 1 X 4
% inliers     : 3D points            -> 3 X M
% mask_inlier : mask of inliers      -> 1 X N

n_sample=3; %3 points for one plane
n_pts=length(points); % #_points
id_good_fit = zeros(iter, 1);

if size(points,1)>3
    points=points'; %3 X N
end

ini_inlier      = zeros(iter,n_pts);       % iter x #_points
mask            = zeros(iter,n_pts);       % iter x #_points
residual        = zeros(iter,n_pts);        % iter x #_points
residual_signed = zeros(1,n_pts);    % 1    x #_points

inlier=cell(iter,1);
ABCD=zeros(4,iter);

while(1)
    for m=1:iter
        inlier_cnt(m,1)=1;
        
        % draw three points randomly
        k = floor(n_pts*rand(n_sample,1))+1; % +1
        n1=k(1,1);
        n2=k(2,1);
        n3=k(3,1);
        
        % calc planes
        X=[points(1,n1) points(1,n2) points(1,n3)]' ;
        Y=[points(2,n1) points(2,n2) points(2,n3)]' ;
        Z=[points(3,n1) points(3,n2) points(3,n3)]' ;
        ABCD(1,m) = det( [ones(3,1), Y, Z] );
        ABCD(2,m) = det( [X, ones(3,1), Z] ) ;
        ABCD(3,m) = det( [X, Y, ones(3,1)] ) ;
        ABCD(4,m) = -det( [X, Y, Z] ) ;
        
        % inlier?
        for i=1:n_pts
            residual(m,i) = abs( ABCD(1,m)*points(1,i) + ABCD(2,m)*points(2,i) + ABCD(3,m)*points(3,i) +ABCD(4,m) ) /...
                sqrt(ABCD(1,m)^2+ABCD(2,m)^2+ABCD(3,m)^2)  ;
            if residual(m,i) < thr
                ini_inlier(m,i)=1;
                mask(m,i) = 1;
            end
        end
        
        for j=1:n_pts
            if ini_inlier(m,j)==1
                % inlier{m,i nlier_cnt(m,1)}=[points(1,j) , points(2,j) , points(3,j)];
                inlier{m,1}(:, inlier_cnt(m,1))=[points(1,j) , points(2,j) , points(3,j)]';
                %             plot3(points(1,j) , points(2,j), points(3,j) ,'yp');
                inlier_cnt(m,1) = inlier_cnt(m,1) + 1;
            end
            
        end
        
        if ( inlier_cnt(m,1) - 1 ) > mini_inlier
            id_good_fit(m,1)=1;
        end
    end
    
    if isempty(find(id_good_fit==1,1))==0
        break;
    else
        thr=thr+0.0005;
    end
end

max_cnt=find(inlier_cnt==max(inlier_cnt));
mini_pre=1000;
% test
if length(max_cnt)>1
    n_candi=length(max_cnt);
    for i_can=1:n_candi
        mini=min( mean(residual(max_cnt(i_can,1),:),2), mini_pre );
        if mini<mini_pre
            id_mini=i_can;
        end
        mini_pre=mini;
    end
    max_cnt=max_cnt(id_mini,1);
else
    max_cnt=max_cnt(1,1); % delete duplicates
end

best_n_inlier=inlier_cnt(max_cnt,1)-1;
% fprintf('          The %d-th has inlier the most (Its number is %d)          \n',max_cnt , best_n_inlier);

%least square
%A
for i=1:best_n_inlier
    A(i,:)=[inlier{max_cnt,1}(1,i) , inlier{max_cnt,1}(2,i) , inlier{max_cnt,1}(3,i) , 1];
    %     b(i,1)=[-inlier{max_cnt,i}(1,1)];
end

[~,~,V]=svd(A);

t=V(:,4);

idxx=find(abs(t(1:3,1))==min(abs(t(1:3,1))));

if idxx==2
    z_max=max(inlier{max_cnt,1}(3,:)); z_min=min(inlier{max_cnt,1}(3,:));
    y_max=max(inlier{max_cnt,1}(2,:)); y_min=min(inlier{max_cnt,1}(2,:));
    [refined_z,~] = meshgrid(linspace(z_min-0.03,z_max+0.03,3));
    [~,refined_y] = meshgrid(linspace(y_min-0.03,y_max+0.03,3));
    refined_x=-(t(3,1)*refined_z+t(2,1)*refined_y+t(4,1) )./t(1,1) ;
elseif idxx==3
    x_max=max(inlier{max_cnt,1}(1,:)); x_min=min(inlier{max_cnt,1}(1,:));
    z_max=max(inlier{max_cnt,1}(3,:)); z_min=min(inlier{max_cnt,1}(3,:));
    [refined_x,~] = meshgrid(linspace(x_min-0.03,x_max+0.03,3));
    [~,refined_z] = meshgrid(linspace(z_min-0.03,z_max+0.03,3));
    refined_y=-(t(1,1)*refined_x+t(3,1)*refined_z+t(4,1) )./t(2,1) ;
elseif idxx==1
    x_max=max(inlier{max_cnt,1}(1,:)); x_min=min(inlier{max_cnt,1}(1,:));
    y_max=max(inlier{max_cnt,1}(2,:)); y_min=min(inlier{max_cnt,1}(2,:));
    [refined_x,~] = meshgrid(linspace(x_min-0.03,x_max+0.03,3));
    [~,refined_y] = meshgrid(linspace(y_min-0.03,y_max+0.03,3));
    refined_z=-(t(1,1)*refined_x+t(2,1)*refined_y+t(4,1) )./t(3,1) ;
end

%% Outputs
inliers=( inlier{max_cnt,:} ); %3 X N
mask_inlier = mask(max_cnt,:);
plane=[t'];

idx_inlier = find(roi>0);
roi(idx_inlier(mask_inlier<1) ) = 0;
end