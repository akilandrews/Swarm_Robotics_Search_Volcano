function obstacles = createObstacleMap(xmax, ymax)

obstacles = false(ymax, xmax);

% Create a grid of (x,y) coordinates with (1,1), at the top left and 
% (ymax,xmax) at the bottom right.
[x, y] = meshgrid (1:xmax, 1:ymax);

%% Generate some obstacles
obstacles (400:500, 300:350) = true;
obstacles (200:250, 400:450) = true;

t = ((x - 200).^2 + (y - 150).^2) < 60^2;
obstacles(t) = true;

end

