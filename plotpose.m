clear all;
load pose.m
load map.m
dN = size(dead);
dN = dN(1);
mN = size(mappose);
mN = mN(1);
mx = mappose(1:2:mN-1,1);
my = mappose(2:2:mN,1);
N = size(pose);
N = N(1);
x = pose(1:2:N-1,1);
y = pose(2:2:N,1);
deadx = dead(1:2:dN-1,1);
deady = dead(2:2:dN,1);
realx = realpose(1:2:N-1,1);
realy = realpose(2:2:N,1);
plot(x,y,"linewidth",2);
hold on;
plot(realx,realy,"color","red","linewidth",2);
for i = [1 : dN/2]
  plot(deadx(i),deady(i),"color","green","linewidth",3);
end
cN = size(mapcov);
cN = cN(1);



for i = [1:2:cN]
  C = mapcov(i:i+1,:);
  [eigenvec, eigenval ] = eig(C);
  [largest_eigenvec_ind_c, r] = find(eigenval == max(max(eigenval)));
  largest_eigenvec = eigenvec(:, largest_eigenvec_ind_c);

% Get the largest eigenvalue
  largest_eigenval = max(max(eigenval));

% Get the smallest eigenvector and eigenvalue
  if(largest_eigenvec_ind_c == 1)
      smallest_eigenval = max(eigenval(:,2));
      smallest_eigenvec = eigenvec(:,2);
  else
      smallest_eigenval = max(eigenval(:,1));
      smallest_eigenvec = eigenvec(1,:);
  end

  % Calculate the angle between the x-axis and the largest eigenvector
  angle = atan2(largest_eigenvec(2), largest_eigenvec(1));

  % This angle is between -pi and pi.
  % Let's shift it such that the angle is between 0 and 2pi
  if(angle < 0)
      angle = angle + 2*pi;
  end
  
  avg = mappose(i:i+1,:);
  chisquare_val = 5.991;
  theta_grid = linspace(0,2*pi);
  phi = angle;
  X0=avg(1);
  Y0=avg(2);
  a=chisquare_val*sqrt(largest_eigenval);
  b=chisquare_val*sqrt(smallest_eigenval);

  % the ellipse in x and y coordinates 
  ellipse_x_r  = a*cos( theta_grid );
  ellipse_y_r  = b*sin( theta_grid );

  %Define a rotation matrix
  R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];

  %let's rotate the ellipse to some angle phi
  r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;

  % Draw the error ellipse
  plot(r_ellipse(:,1) + X0,r_ellipse(:,2) + Y0)
  hold on;
  
end

N = size(cov);
N = N(1);
for i = [1:20:N]
  C = cov(i:i+1,:);
  [eigenvec, eigenval ] = eig(C);
  [largest_eigenvec_ind_c, r] = find(eigenval == max(max(eigenval)));
  largest_eigenvec = eigenvec(:, largest_eigenvec_ind_c);

% Get the largest eigenvalue
  largest_eigenval = max(max(eigenval));

% Get the smallest eigenvector and eigenvalue
  if(largest_eigenvec_ind_c == 1)
      smallest_eigenval = max(eigenval(:,2));
      smallest_eigenvec = eigenvec(:,2);
  else
      smallest_eigenval = max(eigenval(:,1));
      smallest_eigenvec = eigenvec(1,:);
  end

  % Calculate the angle between the x-axis and the largest eigenvector
  angle = atan2(largest_eigenvec(2), largest_eigenvec(1));

  % This angle is between -pi and pi.
  % Let's shift it such that the angle is between 0 and 2pi
  if(angle < 0)
      angle = angle + 2*pi;
  end
  
  avg = pose(i:i+1,:);
  chisquare_val = 2.4477;
  theta_grid = linspace(0,2*pi);
  phi = angle;
  X0=avg(1);
  Y0=avg(2);
  a=chisquare_val*sqrt(largest_eigenval);
  b=chisquare_val*sqrt(smallest_eigenval);

  % the ellipse in x and y coordinates 
  ellipse_x_r  = a*cos( theta_grid );
  ellipse_y_r  = b*sin( theta_grid );

  %Define a rotation matrix
  R = [ cos(phi) sin(phi); -sin(phi) cos(phi) ];

  %let's rotate the ellipse to some angle phi
  r_ellipse = [ellipse_x_r;ellipse_y_r]' * R;

  % Draw the error ellipse
  plot(r_ellipse(:,1) + X0,r_ellipse(:,2) + Y0,'-')
  hold on;
  
end

axis("equal")
  