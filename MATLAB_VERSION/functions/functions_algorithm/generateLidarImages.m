function [img_intense, img_rho, img_index] = generateLidarImages(pcls, azimuth_res, dis_lim)
D2R = pi/180; R2D = 180/pi;
twopi = 2*pi;

n_pts = length(pcls.Location); % # of 3-D points

intensity = zeros(1,n_pts);
rho   = zeros(1,n_pts);
psi   = zeros(1,n_pts);
theta = zeros(1,n_pts);

offset_psi = pi;

for i = 1:n_pts
   
   X = pcls.Location(:,i);
   if norm(X) > dis_lim
   intensity(i) = pcls.Intensity(i);
   rho(i)   = norm(X);
   theta(i) = asin(X(3)/rho(i));
   invrhocos = 1/(rho(i)*cos(theta(i)));
   
   cospsi = X(1)*invrhocos;
   sinpsi = X(2)*invrhocos;
   if(cospsi >= 0)
      if(sinpsi >=0) % 1 quadrant
         psi(i) = acos(cospsi)+offset_psi;
      else % 4 quadrant
         psi(i) = twopi-acos(cospsi)+offset_psi;
      end
   else
      if(sinpsi >=0) % 2 quadrant
         psi(i) = pi-acos(-cospsi)+offset_psi;
      else % 3 quadrant
         psi(i) = pi+acos(-cospsi)+offset_psi;
      end
   end
   if(psi(i) >= twopi)
      psi(i) = psi(i)-twopi;
   end
   end
end
psi = real(psi);

% generate range images
step = 1/azimuth_res; % 0.2 degrees step.
img_intense = zeros(16,360*step+1);
img_rho     = zeros(16,360*step+1);
img_index   = zeros(16,360*step+1);

img_rho_d   = cell(16,360*step+1);
img_index_d = cell(16,360*step+1);
mask_double = zeros(16,360*step+1);

for i = 1:n_pts
        ind_row(i) = round((17+(theta(i)*R2D))/2);
        ind_col(i) = round(psi(i)*step*R2D)+1;
        
        if img_rho(ind_row(i),ind_col(i))~=0  
            mask_double(ind_row(i),ind_col(i))= mask_double(ind_row(i),ind_col(i))+1;
            img_index_d{ind_row(i),ind_col(i)} = [img_index(ind_row(i),ind_col(i)), i];
            img_rho_d{ind_row(i),ind_col(i)} = [img_rho(ind_row(i),ind_col(i)), rho(i)];
        else
            mask_double(ind_row(i),ind_col(i)) =1;
        end
        
        if img_rho(ind_row(i),ind_col(i))~=0
            if img_rho(ind_row(i),ind_col(i)) > rho(i) 
                img_rho(ind_row(i),ind_col(i)) = rho(i);
                img_intense(ind_row(i),ind_col(i)) = intensity(i);
                img_index(ind_row(i),ind_col(i)) = i;
            else
            end
        else
            img_rho(ind_row(i),ind_col(i)) = rho(i);
            img_intense(ind_row(i),ind_col(i)) = intensity(i);
            img_index(ind_row(i),ind_col(i)) = i;
        end
end

k=0;
for i=1:16
    idx_d=find(mask_double(i,:)>1);
    for j=idx_d
        if or(j==1,j==360*step+1)
        else
            max_th = find(img_rho_d{i,j}==max(img_rho_d{i,j}),1);
            if img_rho(i,j-1)==0
                img_rho(i,j-1)      = img_rho_d{i,j}(max_th);
                img_index(i,j-1)    = img_index_d{i,j}(max_th);
                k=k+1;
            elseif img_rho(i,j+1)==0
                img_rho(i,j+1)      = img_rho_d{i,j}(max_th);
                img_index(i,j+1)    = img_index_d{i,j}(max_th);
                k=k+1;
            else
            end
        end
    end
end
end