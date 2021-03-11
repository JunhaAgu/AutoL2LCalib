function [img_intense, img_rho, img_index] = generateLidarImages(data, lim, m, n)
D2R = pi/180; R2D = 180/pi;
twopi = 2*pi;

pcls = data.pcls{m,n};
if m==1
    azimuth_res = data.l0.spec.azimuth_res;
    elevation_res = data.l0.spec.elevation_res;
    elevation_min = data.l0.spec.elevation_min;
    n_ring = data.l0.spec.n_ring;
elseif m==2
    azimuth_res = data.l1.spec.azimuth_res;
    elevation_res = data.l1.spec.elevation_res;
    elevation_min = data.l1.spec.elevation_min;
    n_ring = data.l1.spec.n_ring;
end

ang_lim = lim.ang;

n_pts = length(pcls.Location); % # of 3-D points

intensity = zeros(1,n_pts);
rho   = zeros(1,n_pts);
theta   = zeros(1,n_pts);
phi = zeros(1,n_pts);

offset_theta = pi;

for i = 1:n_pts
    
    X = pcls.Location(:,i);
    intensity(i) = pcls.Intensity(i);
    rho(i)   = norm(X);
    phi(i) = asin(X(3)/rho(i));
    invrhocos = 1/(rho(i)*cos(phi(i)));
    
    cospsi = X(1)*invrhocos;
    sinpsi = X(2)*invrhocos;
    if(cospsi >= 0)
        if(sinpsi >=0) % 1 quadrant
            theta(i) = acos(cospsi)+offset_theta;
        else % 4 quadrant
            theta(i) = twopi-acos(cospsi)+offset_theta;
        end
    else
        if(sinpsi >=0) % 2 quadrant
            theta(i) = pi-acos(-cospsi)+offset_theta;
        else % 3 quadrant
            theta(i) = pi+acos(-cospsi)+offset_theta;
        end
    end
    if(theta(i) >= twopi)
        theta(i) = theta(i)-twopi;
    end
end
theta = real(theta);

% generate range images
az_step = 1/azimuth_res; % 0.2 degrees step.
img_intense = zeros(n_ring,360*az_step+1);
img_rho     = zeros(n_ring,360*az_step+1);
img_index   = zeros(n_ring,360*az_step+1);

img_rho_dup   = cell(n_ring,360*az_step+1);
img_index_dup = cell(n_ring,360*az_step+1);
mask_dup = zeros(n_ring,360*az_step+1);

for i = 1:n_pts
    if theta(i) > ang_lim(1,1)+offset_theta && theta(i) < ang_lim(1,2)+offset_theta
        i_row = round( ( (phi(i)*R2D) - elevation_min)/elevation_res + 1 );
        i_col = round(theta(i)*az_step*R2D)+1;
        
        if img_rho(i_row,i_col)~=0
            mask_dup(i_row,i_col)= mask_dup(i_row,i_col)+1;
            img_index_dup{i_row,i_col} = [img_index(i_row,i_col), i];
            img_rho_dup{i_row,i_col} = [img_rho(i_row,i_col), rho(i)];
            
            if img_rho(i_row,i_col) > rho(i)
                img_rho(i_row,i_col) = rho(i);
                img_intense(i_row,i_col) = intensity(i);
                img_index(i_row,i_col) = i;
            end
        else
            mask_dup(i_row,i_col) =1;
            
            img_rho(i_row,i_col) = rho(i);
            img_intense(i_row,i_col) = intensity(i);
            img_index(i_row,i_col) = i;
        end
    end
end

for i=1:n_ring
    idx_d=find(mask_dup(i,:)>1);
    for j=idx_d
        if or(j==1,j==360*az_step+1)
        else
            max_th = find(img_rho_dup{i,j}==max(img_rho_dup{i,j}),1);
            if img_rho(i,j-1)==0
                img_rho(i,j-1)      = img_rho_dup{i,j}(max_th);
                img_index(i,j-1)    = img_index_dup{i,j}(max_th);
            elseif img_rho(i,j+1)==0
                img_rho(i,j+1)      = img_rho_dup{i,j}(max_th);
                img_index(i,j+1)    = img_index_dup{i,j}(max_th);
            end
        end
    end
end
end