% Render and animate swarm robots
%   Uses a scout swarm algorithm where scouts use random navigation to 
%   search configuration space and identify locations that exceed a 
%   threshold. Then use a single agent gradient walk, pause to collect 
%   samples, and return to base.
% Preconditions:
%   vent_map = gas concentration configuration map
%   threshold = value that if exceeded modifies swarm behaviour
%   grad_map = gradient of gas concentration configuration map
%   filename(optional) = to store recorded swarm positions
function renderSwarm(vent_map, threshold, grad_map, filename)
    global field_map cover_map mission_time
    num_robots = 2;
    mission_time = 1e9;
    % Build maps
    [x, y, field_map, ~] = generateBaseMap();
    [~, ~, cover_map, ~] = generateBaseMap();
    title({[num2str(num_robots), ' Swarm robots measuring CO2 concentrations']; ' in 100m x 100m volcano region'});
    % Build robots
    scout_bots(1, num_robots) = Robot;
    for i=1:num_robots
       scout_bots(1, i).initialize([mission_time (i*10)], [2 2 0], vent_map, threshold, grad_map, i); 
    end
    % Animation - render, pause, delete each step
    t = 1e9; % Max simulation time steps
    at_home_count = 0;
    for i = 1:t
        % Render time
        current_time = text(5.0, 95.0, 6.0, [num2str(i), ' (ts)'], 'Color', 'black');
        % Render robots
        for j = 1:length(scout_bots)
            scout_bots(j).simulate(i);
            if scout_bots(j).isAtHome()
                at_home_count = at_home_count + 1;
            end
        end
        % If all robots have returned home end simulation
        if at_home_count == length(scout_bots)
            break;
        else
            at_home_count = 0;
        end
        pause(0.1);
        if i < t
            delete(current_time);
            for k = 1:length(scout_bots)
                delete(scout_bots(k).robot);
                delete(scout_bots(k).tether);
                delete(scout_bots(k).sensor);
            end
        end
    end
    % Clear ROS nodes
    for j = 1:length(scout_bots)
        clear scout_bots(j).node scout_bots(j).pub;
    end
    % Render field results
    figure(2);
    createFieldMap(x, y);
    figure(3);
    createCoverageMap(x, y);
    % Save maps to file
    if nargin > 3 && ~isempty(filename)
        filename = sprintf('%s.fieldmap.txt', filename);
        csvwrite(filename, field_map);
        filename = sprintf('%s.covermap.txt', filename);
        csvwrite(filename, cover_map);
    end
end

% Build a field map
% Preconditions:
%   x, y = vent map
% Postconditions:
%   fmgh = field map graphics handle
function fmgh = createFieldMap(x, y)
    global field_map
    fmgh = surf(x, y, field_map, 'EdgeColor', 'none', 'FaceColor', 'interp');
    view(0, 90);% view(0, 75);
    xticks([0 10 20 30 40 50 60 70 80 90 100]);
    yticks([0 10 20 30 40 50 60 70 80 90 100]);
    xlabel('X'); ylabel('Y');
    title('Field map of measured CO2 concentrations (0-3000 ppm)');
    colormap(jet);
    colorbar;
    caxis([0 3]);
end

% Build a coverage map
% Preconditions:
%   x, y = cover map
% Postconditions:
%   cmgh = coverage map graphics handle
function cmgh = createCoverageMap(x, y)
    global cover_map
    % Covert from surface to scatter
    max_dim = length(cover_map);
    max_dim = max_dim^2;
    x1 = reshape(x, [1, max_dim]);
    y1 = reshape(y, [1, max_dim]);
    z1 = reshape(cover_map, [1, max_dim]);
    x2 = zeros(1, max_dim);
    y2 = zeros(1, max_dim);
    z2 = zeros(1, max_dim);
    count = 1;
    % Add positions that were recorded
    for i = 1:max_dim
        if z1(i) == 1
            x2(count) = x1(i);
            y2(count) = y1(i);
            z2(count) = z1(i);
            count = count + 1;
        end
    end
    % Resize vectors deleting unused indices
    x2(:, count:1:max_dim) = [];
    y2(:, count:1:max_dim) = [];
    z2(:, count:1:max_dim) = [];
    % Plot
    cmgh = scatter3(x2, y2, z2, 'black', '.');
    view(0, 90);
    xticks([0 10 20 30 40 50 60 70 80 90 100]);
    yticks([0 10 20 30 40 50 60 70 80 90 100]);
    xlabel('X'); ylabel('Y');
    title(['Coverage = ', num2str((count / max_dim) * 100), '%']);
%     %i Save map to file
%     if nargin > 0 && ~isempty(filename)
%         filename = sprintf('%s.bomap.txt', filename);
%         csvwrite(filename, vent_map);
%     end
end
