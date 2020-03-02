close all
% Gaussian plume model 
%    using MATLAB analytical solutions                   
%
%   $Ekkehard Holzbecher  $Date: 2006/08/21$

% Input
Dy = 0.2; Dz = 1;           % diffusivities
v = 0.5;                    % velocity
lambda = 0;                 % decay rate
Q = 1;                      % emission rate(s)
xstack = 10; ystack = 50;    % stack location(s) % NOTE: xstack must be <= xmin
xmin = 10; xmax = 1000;     % x-axis interval
ymin = 0; ymax = 100;       % y-axis interval (used only for d>1)
xyinc = 100;                % x and y axis granularity
zmin = 0; zmax = 100;       % z-axis
H = 50;                     % effective stack height(s) 

% Process
[x,y] = meshgrid (linspace(xmin,xmax,xyinc),linspace(ymin,ymax,xyinc));
c = zeros(xyinc, xyinc); e = ones(xyinc, xyinc);
figure;
set(gca, 'NextPlot', 'replace', 'Visible', 'off');
for z = zmin:10:zmax % height of observation (=0 for ground surface)
    for i = 1:size(Q,2)
        xx = x - xstack(i); yy = y - ystack(i); 
        c = c + Q(i)*e./(4*pi*xx*sqrt(Dy*Dz)).*exp(-v*yy.*yy./(4*Dy*xx)).*... 
        (exp(-v*(z-H(i))*(z-H(i))*e./(4*Dz*xx))+exp(-v*(z+H(i))*(z+H(i))*e./(4*Dz*xx)))...
        .*exp(-lambda*xx/v);
    end
    % Plot
    % for i = 1:100 % 10:10:100
    %     plot(c(:,i)); xlabel('X'); ylabel('Y'); hold on;
    % end
    contourf(x,y,c); xlabel('X'); ylabel('Y'); colorbar;
    %contour(x,y,c); xlabel('X'); ylabel('Y');
    pause(1.0);
end


