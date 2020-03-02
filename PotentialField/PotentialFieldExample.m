function PotentialFieldExample( xmax, ymax, obstacles, start, goal )
% Creates a 2D world with obstacles. A replusive field is generated around
% obstacles and an attractive field is created around the goal. 
% A robot navigates from its start location to its goal by following the
% gradient of the field.
% Obstacles is an xmax by ymax boolean matrix. Obstacles are marked with by
% 'true' entries and clearspace by 'false'.
% The start and goal positions of the robot are given by coordinate pairs
% start = [x,y] and goal = [x,y].

% Create a coordinate system
[x, y] = meshgrid (1:xmax, 1:ymax);


%% Compute distance transform
d = bwdist(obstacles);

% Rescale and transform distances
d2 = (d/100) + 1;
d0 = 2;
nu = 800;

repulsive = nu*((1./d2 - 1/d0).^2);

repulsive (d2 > d0) = 0;

%% Compute attractive force
xi = 1/700;
attractive = xi * ( (x - goal(1)).^2 + (y - goal(2)).^2 );


%% Combine terms
total_potential = attractive + repulsive;

%% Plan route
route = GradientDescentPlanner(total_potential, start, goal, 1000);


% -------------------------------------------------------------

% Create plots to visualise the total energy surface, repulsive, and
% attractive fields and an animation of the path taken by the robot.

figure;
m = mesh (attractive);
m.FaceLighting = 'phong';
axis equal;

% Create a light for 3D plotting effects
view(0,75)
shading interp
lightangle(-45,30)

title ('Attractive Potential');


%% Display 2D configuration space
figure;
imshow(~obstacles);

hold on;
plot (goal(1), goal(2), 'r.', 'MarkerSize', 25);
hold off;

axis ([0 xmax 0 ymax]);
axis xy;
axis on;

xlabel ('x');
ylabel ('y');

title ('Configuration Space');

figure;
m = mesh (total_potential);
m.FaceLighting = 'phong';
axis equal;

% Create a light for 3D plotting effects
view(0,75)
shading interp
lightangle(-45,30)

title ('Total Potential');

%% Display repulsive potential

figure;
m = mesh (repulsive);
m.FaceLighting = 'phong';
axis equal;

% Create a light for 3D plotting effects
view(0,75)
shading interp
lightangle(-45,30)

title ('Repulsive Potential');

%% Plot the energy surface

figure;
m = mesh (total_potential);
axis equal;

%% Plot ball sliding down hill

[sx, sy, sz] = sphere(20);

scale = 20;
sx = scale*sx;
sy = scale*sy;
sz = scale*(sz+1);

hold on;
p = mesh(sx, sy, sz);
p.EdgeColor = 'none';
p.FaceLighting = 'phong';

% Create a light for 3D plotting effects
view(120,50)
shading interp
lightangle(-45,30)
p.FaceLighting = 'gouraud';
p.AmbientStrength = 0.3;
p.DiffuseStrength = 0.8;
p.SpecularStrength = 0.9;
p.SpecularExponent = 25;
p.BackFaceLighting = 'unlit';

hold off;

for i = 1:size(route,1)
    P = round(route(i,:));
    new_x = P(1);
    new_y = P(2);
    new_z = total_potential(P(2), P(1));
    
    p.XData = sx + new_x;
    p.YData = sy + new_y;
    p.ZData = sz + new_z;
    
    drawnow;
    
end

%% quiver plot
[gx, gy] = gradient (-total_potential);
skip = 20;

figure;

xidx = 1:skip:xmax;
yidx = 1:skip:ymax;

qplot = quiver (x(yidx,xidx), y(yidx,xidx), gx(yidx,xidx), gy(yidx,xidx), 0.4);
set(qplot,'AutoScale','on', 'AutoScaleFactor',1.0)

axis ([1 xmax 1 ymax]);

hold on;

ps = plot(start(1), start(2), 'r.', 'MarkerSize', 30);
pg = plot(goal(1), goal(2), 'g.', 'MarkerSize', 30);
p3 = plot (route(:,1), route(:,2), 'r', 'LineWidth', 2);
end