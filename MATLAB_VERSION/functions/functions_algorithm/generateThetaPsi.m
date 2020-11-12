function [theta, psi] = generateThetaPsi(pts)
D2R = pi/180; R2D = 180/pi;
twopi = 2*pi;

n_pts = length(pts); % # of 3-D points

psi   = zeros(1,n_pts);
theta = zeros(1,n_pts);
rho   = zeros(1,n_pts);

for i = 1:n_pts
   X = pts(:,i);
   rho(i)   = norm(X);
   theta(i) = asin(X(3)/rho(i));
   invrhocos = 1/(rho(i)*cos(theta(i)));
   
   cospsi = X(1)*invrhocos;
   sinpsi = X(2)*invrhocos;
   if(cospsi >= 0)
      if(sinpsi >=0) % 1 quadrant
         psi(i) = acos(cospsi);
      else % 4 quadrant
         psi(i) = twopi-acos(cospsi);
      end
   else
      if(sinpsi >=0) % 2 quadrant
         psi(i) = pi-acos(-cospsi);
      else % 3 quadrant
         psi(i) = pi+acos(-cospsi);
      end
   end
   if(psi(i) >= twopi)
      psi(i) = psi(i)-twopi;
   end
end
psi = real(psi);

end