% Generate an empty configuration map
% Preconditions:
% Postconditions:
%   x, y, z = map
%   max_dim = length of each dimension of map
function [x, y, z, max_dim] = generateBaseMap()
    max_coord = 100;
    [x, y] = meshgrid(0:1e-1:max_coord);
    max_dim = length(x);
    z = 0*x + 0*y;
end