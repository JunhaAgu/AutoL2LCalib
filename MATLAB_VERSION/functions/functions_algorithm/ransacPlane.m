%ransac
function [plane, inliers, mask_inlier, roi] = ransacPlane(points, roi, param) %, x_min , x_max , y_min, y_max, z_min, z_max, n_ROI_las)
% written by JunhaKim
%%%%% input %%%%%
% points      : 3D points, size     -> 3 x N
% roi         : logical             -> n_ring X n_step
% param -> iter, thr, inlier 
% iter        : maximum iteration   -> scalar
% thr         : distance threshold  -> scalar
% mini_inlier : XX% of points       -> scalar

%%%%% output %%%%%
% plane       : plane coefficient   -> 1 X 4
% inliers     : 3D points           -> 3 X M (M<=N)
% mask_inlier : mask of inliers     -> 1 X N
% roi         : logical             -> n_ring X n_step

iter = param.iter;
thr = param.thr;
mini_inlier = param.inlier;

n_sample=3; %3 points for one plane
n_pts=length(points); % #_points
id_good_fit = zeros(iter, 1);

if size(points,1)>3
    points=points'; %3 X N
end

ini_inlier      = zeros(iter,n_pts);       % iter x #_points
mask            = zeros(iter,n_pts);       % iter x #_points
residual        = zeros(iter,n_pts);        % iter x #_points

inlier=cell(iter,1);
ABCD=zeros(4,iter);

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
            inlier{m,1}(:, inlier_cnt(m,1))=[points(1,j) , points(2,j) , points(3,j)]';
            inlier_cnt(m,1) = inlier_cnt(m,1) + 1;
        end
    end
    
    if ( inlier_cnt(m,1) - 1 ) > mini_inlier
        id_good_fit(m,1)=1;
    end
end

max_cnt=find(inlier_cnt==max(inlier_cnt));
mini_pre=1000;

% delete duplicates
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
    max_cnt=max_cnt(1,1);
end

best_n_inlier=inlier_cnt(max_cnt,1)-1;
% fprintf('          The %d-th has inlier the most (Its number is %d)          \n',max_cnt , best_n_inlier);

%least square
A = zeros(best_n_inlier,4);
for i=1:best_n_inlier
    A(i,:)=[inlier{max_cnt,1}(1,i) , inlier{max_cnt,1}(2,i) , inlier{max_cnt,1}(3,i) , 1];
end

[~,~,V]=svd(A);

t=V(:,4);

%% Outputs
inliers=( inlier{max_cnt,:} ); %3 X N
mask_inlier = mask(max_cnt,:);
plane = t';

idx_inlier = find(roi>0);
roi(idx_inlier(mask_inlier<1) ) = 0;
end