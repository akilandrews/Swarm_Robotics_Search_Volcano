% (GUI) Simulation of swarm-robotics volcanic eruption forecast research
% Preconditions:
%   filename(optional) = to store recorded swarm positions
% Postconditions:
function simulateVolcanicZoneResearch(filename)
    global bo_map
    close all
    % Intialize ROS
%  %TODO   setenv('ROS_MASTER_URI','http://192.168.1.55:11311');
%     setenv('ROS_IP','192.168.1.51');
%     rosinit;
    master = robotics.ros.Core;
    pause(5);
    % Configuration space map
    figure(1);
    [x, y, vent_map] = createVolcanoZoneMap();
    surf(x, y, vent_map, 'EdgeColor', 'none', 'FaceColor', 'interp');
    view(0, 0);
    axis off;
    colormap(jet);
    caxis([0 3]);
    % Save map to file
    if nargin > 0 && ~isempty(filename)
        filename = sprintf('%s.ventmap.txt', filename);
        csvwrite(filename, vent_map);
    end
    % Boundary map
    bo_map = createBoundaryMap();
    % Gradient map
    [GX, GY] = gradient(vent_map, 1e-1);
    grad_map = [GX; GY];
    % Render robots
    if nargin > 0 && ~isempty(filename)
        renderSwarm(vent_map, 0.01, grad_map, filename);
    else
        renderSwarm(vent_map, 0.01, grad_map);
    end
    % Save maps to file
    if nargin > 0 && ~isempty(filename)
        filename = sprintf('%s.ventmap.txt', filename);
        csvwrite(filename, vent_map);
        filename = sprintf('%s.gradmap.txt', filename);
        csvwrite(filename, grad_map);
        filename = sprintf('%s.bomap.txt', filename);
        csvwrite(filename, bo_map);
    end
    % Shutdown ROS
    clear master;
end

% Build a configuration space map with 10 randomly distributed and sized vents
% Preconditions:
% Postconditions:
%   x, y, z = gas vent map
function [x, y, z] = createVolcanoZoneMap()
    % Create gaussian distribution
    N = 5;
    x = linspace(-N, N);
    y = x;
    [X,Y] = meshgrid(x, y);
    Z = (7 / sqrt(2*pi) .* exp(-(X.^2/2)-(Y.^2/2))); % Note: 10 replaced 1000
    % Create base plane z = 0
    [x, y, z, max_dim] = generateBaseMap();
    % Translate each vent by random rx ry max=1000
    rx = randi([1 (max_dim - 101)], 1, 10);
    ry = randi([1 (max_dim - 101)], 1, 10);
    % Scale each vent by random rr
    rr = boundRandomTranslationValues(rx, ry, max_dim);
    % Add 10 randomly distrubted 'gas vents'
    for i = 1:10
        incr = rr(i);
        sx = 1 + rx(i); sy = 1 + ry(i);
        ex = 99 * incr + sx;% ((100+rx(i)) - sx) * incr + sx;
        ey = 99 * incr + sy;% ((100+ry(i)) - sy) * incr + sy;
        ty = 1; ctrk = 1;
        for k = sy:ey
            tx = 1; ctrj = 1;
            for j = sx:ex
                z(j, k) = z(j, k) + Z(tx, ty);
                if mod(ctrj, incr) == 0
                    tx = tx + 1;
                end
                ctrj = ctrj + 1;
            end
            if mod(ctrk, incr) == 0
                ty = ty + 1;
            end
            ctrk = ctrk + 1;
        end
    end
end

% Prevent random size of vents from exceeding max dimensions of grid
% Preconditions:
%   rxvals, ryvals = random values to add to cartesian position of vents
%   max_dim = maximum array dimension for a vent map
% Postconditions:
%   rv = random radius values
function rv = boundRandomTranslationValues(rxvals, ryvals, max_dim)
    rv = zeros(1, 10);
    ctr = 1;
    while rv(10) == 0
        rv1 = randi([1 5]);
        rxvalid = 99 * rv1 + (1 + rxvals(ctr));
        ryvalid = 99 * rv1 + (1 + ryvals(ctr));
        if rxvalid < max_dim && ryvalid < max_dim
            rv(ctr) = rv1;
            ctr = ctr + 1;
        end
    end
end

% Build a boundary map with obstacle=1
% Preconditions:
% Postconditions:
%   z = boundary map
function z = createBoundaryMap()
    [~, ~, z, max_dim] = generateBaseMap();
    z(:, 1:20) = 1;
    z(:, ((max_dim-21):max_dim)) = 1;
    z(1:20, :) = 1;
    z(((max_dim-21):max_dim), :) = 1;
end
