function route = GradientDescentPlanner (f, start_coords, end_coords, max_its)
% This function calculates which direction to move given a force field

% Calculate the gradient at each point given the forces at each point.
[gx, gy] = gradient (-f);

% Add the initial condition to the route
route = start_coords;

% Start at the initial position
current_coords = start_coords;

% Define how close we need to be to the goal state before deciding we have
% arrived
epsilon = 2.0;

% Iteratively descend through the force field
for i = 1:max_its
              
    % Get the gradient calculated above for our current position
    % Make the step we take be in the direction of the gradient
    % Round the values so we make sure our position is at the nearest
    % grid point.
    step = [ gx(round(current_coords(2)), round(current_coords(1))),...
             gy(round(current_coords(2)), round(current_coords(1))) ];
    
    % Take a step of length one (unit vector) in the direction of the
    % gradient
    step = step/norm(step);
    
    % Update our current location
    current_coords = current_coords + step;
        
    % Add this step to the route through the field
    route = [route; current_coords];
    
    % Determine if our current location is within epsilon of the goal
    % location. If so stop the iterative descent.
    if norm(current_coords-end_coords) < epsilon
        disp('Yay, reached goal!');
        break;
    end
end

end
