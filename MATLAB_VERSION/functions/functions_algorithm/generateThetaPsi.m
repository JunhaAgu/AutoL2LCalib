function [phi, theta] = generateThetaPsi(pts)
D2R = pi/180; R2D = 180/pi;
twopi = 2*pi;

n_pts = length(pts); % # of 3-D points

theta   = zeros(1,n_pts);
phi = zeros(1,n_pts);
rho   = zeros(1,n_pts);

for i = 1:n_pts
   X = pts(:,i);
   rho(i)   = norm(X);
   phi(i) = asin(X(3)/rho(i));
   invrhocos = 1/(rho(i)*cos(phi(i)));
   
   costheta = X(1)*invrhocos;
   sintheta = X(2)*invrhocos;
   if(costheta >= 0)
      if(sintheta >=0) % 1 quadrant
         theta(i) = acos(costheta);
      else % 4 quadrant
         theta(i) = twopi-acos(costheta);
      end
   else
      if(sintheta >=0) % 2 quadrant
         theta(i) = pi-acos(-costheta);
      else % 3 quadrant
         theta(i) = pi+acos(-costheta);
      end
   end
   if(theta(i) >= twopi)
      theta(i) = theta(i)-twopi;
   end
end
theta = real(theta);

end