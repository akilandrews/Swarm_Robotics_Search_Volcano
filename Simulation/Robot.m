% Render a swarm robot
%   Uses a random navigation search of configuration space and identifies
%   and broadcast to support robots locations that exceed a threshold.
% Preconditions:
%   mission_time = time to complete mission and deploy time
%   home_pos = starting location
%   vent_map = gas concentration configuration map
%   threshold = value that if exceeded modifies swarm behaviour
%   grad_map = gradient of gas concentration configuration map
% Postconditions:
%   robot = graphics handle
classdef Robot < handle
    properties
        % ROS publisher
        id = 0;
        node = 0;
        pub = 0;
        % Time step and graphics handles
        time = 0;
        robot = 0;
        tether = 0;
        sensor = 0;
%TODO properties (Access = private)
        % Default parameters
        mission_time = 3000;
        deploy_time = 0;
        home_pos = [1 1 0];
        center_pos = [50 50 0];
        vent_map = [];
        threshold = 0.5;
        grad_map_x = [];
        grad_map_y = [];
        max_vent = 0;
        max_sample = 2.5;
        % Kinematics - position >1 <99, velocity, sample coordinates 
        % [initial final], trajectory update every rc secs, bound V obstacle
        cp = [1 1 0];
        velocity = [0.2 0.2 0]; 
        vent_pos = [0 0 0];
        rc = 50;
        rscatter = 251;
        rscatter_time = 250;
        is_bounds_or_obstacle = false;
        status = RobotStatus.deploy;
        linear_x = [];
        linear_index = 1;
        linear_len = 0;
        linear_slope = 0;
        linear_y_intercept = 0;
        spiral_x = [];
        spiral_y = [];
        spiral_z = [];
        spiral_index = 1;
        spiral_len = 0;
        gwalk_time = 0;
        gwalk_time_limit = 250;
        % Robot - cube 1m x 1m x 1m
        robot_color = [1 1 1];
        rx = [-0.5 -0.5 -0.5 -0.5 -0.5 0.5;...
            0.5 0.5 0.5 0.5 -0.5 0.5;...
            0.5 0.5 0.5 0.5 -0.5 0.5;...
            -0.5 -0.5 -0.5 -0.5 -0.5 0.5];
        ry = [-0.5 -0.5 -0.5 0.5 -0.5 -0.5;...
            -0.5 -0.5 -0.5 0.5 0.5 0.5;...
            0.5 0.5 -0.5 0.5 0.5 0.5;...
            0.5 0.5 -0.5 0.5 -0.5 -0.5];
        rz = [(40-1) 40 (40-1) (40-1) (40-1) (40-1);...
            (40-1) 40 (40-1) (40-1) (40-1) (40-1);...
            (40-1) 40 40 40 40 40;...
            (40-1) 40 40 40 40 40];
        % Tether and sensor - line 3m
        tx = [0.0 0.0];
        ty = [0.0 0.0];
        tz = [36.0 39.0];
        % Sensor - sphere 1m diameter
        sx = []; sy = []; sz = [];
    end
    methods
        % Constructor
        function obj = Robot(mission_time, home_pos, vent_map, threshold, grad_map, id)
            if nargin == 6
                obj.id = id;
                obj.mission_time = mission_time(1);
                obj.deploy_time = mission_time(2);
                obj.home_pos = home_pos; obj.cp = home_pos;
                obj.vent_map = vent_map;
                obj.threshold = threshold;
                max_dim = length(grad_map(1,:));
                obj.grad_map_x = grad_map(1:max_dim,:);
                obj.grad_map_y = grad_map((max_dim+1):(2*max_dim),:);
                obj.rscatter = 150*rand();
                [obj.sx, obj.sy, obj.sz] = sphere(6);
                obj.node = robotics.ros.Node(sprintf('/R%d', id));
                obj.pub = robotics.ros.Publisher(obj.node, sprintf('/R%d/co2', id), 'std_msgs/Float32MultiArray');
            end
        end
        % Initialize
        function initialize(obj, mission_time, home_pos, vent_map, threshold, grad_map, id)
                obj.id = id;
                obj.mission_time = mission_time(1);
                obj.deploy_time = mission_time(2);
                obj.home_pos = home_pos; obj.cp = home_pos;
                obj.vent_map = vent_map;
                obj.threshold = threshold;
                max_dim = length(grad_map(1,:));
                obj.grad_map_x = grad_map(1:max_dim,:);
                obj.grad_map_y = grad_map((max_dim+1):(2*max_dim),:);
                obj.rscatter = 150*rand();
                [obj.sx, obj.sy, obj.sz] = sphere(6);
                obj.node = robotics.ros.Node(sprintf('/R%d', id));
                obj.pub = robotics.ros.Publisher(obj.node, sprintf('/R%d/co2', id), 'std_msgs/Float32MultiArray');
        end
        % Simulate robot
        % Preconditions:
        %   time = simulation time step
        % Postconditions:
        function simulate(obj, time)
            obj.time = time;
            % Return sample to home base or begin search
            if obj.time == obj.mission_time
                obj.status = RobotStatus.goHome;
            elseif obj.status == RobotStatus.deploy
                obj.robot_color = [1 1 1];
                obj.navigateDeploy();
            elseif obj.status == RobotStatus.atHome
                obj.robot_color = [1 0 0];
            elseif obj.status == RobotStatus.trapped
                obj.robot_color = [1 0 0];
            elseif obj.status == RobotStatus.goHome
                obj.robot_color = [1 0 0];
                obj.navigateHome();
            elseif obj.status == RobotStatus.exitGradientWalk
                obj.robot_color = [0 1 0];
                obj.navigateExitGradientWalk();
            elseif obj.status == RobotStatus.gradientWalk
                obj.robot_color = [0 1 0];
                obj.navigateGradientWalk();
            elseif obj.status == RobotStatus.search
                obj.robot_color = [1 1 1];
                obj.navigateSearch();
            end
            % Render robot
            if ~obj.is_bounds_or_obstacle
                obj.robot = patch(obj.rx+obj.cp(1), obj.ry+obj.cp(2),...
                    obj.rz+obj.cp(3), obj.robot_color);
                hold on;
                obj.tether = plot3(obj.tx+obj.cp(1), obj.ty+obj.cp(2),...
                    obj.tz+obj.cp(3), 'Color', 'k', 'LineWidth', 0.5);
                [f, v, ~] = surf2patch(0.5*obj.sx+obj.cp(1),...
                    0.5*obj.sy+obj.cp(2),...
                    0.5*obj.sz+obj.cp(3)+obj.tz(1, 1));
                obj.sensor = patch('Faces', f, 'Vertices', v,...
                    'FaceColor', obj.robot_color);
            end
        end
        % At home base check
        % Preconditions:
        % Postconditions:
        %   is_home = true if robot has returned to home base
        function is_home = isAtHome(obj)
            is_home = false;
            if obj.status == RobotStatus.atHome
                is_home = true;
            end
        end
    end
    methods (Access = private)
        % Navigation algorithm - deploy to research area
        % Preconditions:
        % Postconditions:
        function navigateDeploy(obj)
            % Do not deploy until deploy time
            if obj.time < obj.deploy_time
                return;
            end
            deploy_t = obj.time - obj.deploy_time;
            % Calculate new position
            obj.get3DVectorPosition();
            % Do not start search until reach center 50,50 of 100x100 boundary
            if deploy_t < obj.rscatter_time
                return;
            elseif deploy_t == obj.rscatter_time
                % Choose random velocity vector
                ra = 2*pi*(rand());
                p = sqrt(obj.velocity(1)^2 + obj.velocity(2)^2 + obj.velocity(3)^2);
                obj.velocity = [p*1*cos(ra) p*1*sin(ra) 0];
            elseif deploy_t > (obj.rscatter + obj.rscatter_time)
                obj.status = RobotStatus.search;
            end
        end
        % Navigation algorithm - random search
        % Preconditions:
        % Postconditions:
        function navigateSearch(obj)
            % Calculate new position
            obj.get3DVectorPosition();
            if obj.isOutBoundary()
                obj.status = RobotStatus.goHome;
            % Boundary check, note size of rover 2x2x2m
            elseif obj.isBoundary()
                obj.get3DVectorPosition();
            % Choose random velocity vector every rc secs
            elseif mod(obj.time, obj.rc) == 0
                ra = 2*pi*(rand());
                p = sqrt(obj.velocity(1)^2 + obj.velocity(2)^2 + obj.velocity(3)^2);
                obj.velocity = [p*1*cos(ra) p*1*sin(ra) 0];
            else
                obj.executeGasSensor();
            end
            % Raise tether
            obj.raiseTether();
        end
        % Navigation algorithm - linear path to home base
        % Preconditions:
        % Postconditions:
        function navigateHome(obj)
            % Check if close then stop, otherwise follow linear path
            distance = sqrt((obj.home_pos(1) - obj.cp(1))^2 + (obj.home_pos(2) - obj.cp(2))^2 + (obj.home_pos(3) - obj.cp(3))^2);
            if distance < 3
                obj.status = RobotStatus.atHome;
            else
                % Calculate linear x coordinates from position to home
                if length(obj.linear_x) < 2
                    if (obj.cp(1) - obj.home_pos(1)) < 0
                        obj.linear_x = obj.cp(1):0.2:obj.home_pos(1);
                    else
                        obj.linear_x = obj.cp(1):-0.2:obj.home_pos(1);
                    end
                    obj.linear_len = length(obj.linear_x);
                    obj.linear_slope = (obj.home_pos(2) - obj.cp(2))/(obj.home_pos(1) - obj.cp(1));
                    obj.linear_y_intercept = obj.home_pos(2) - obj.linear_slope * obj.home_pos(1);
                end
                % Calculate new position
                obj.get2DLinearPosition();
                % Raise tether
                obj.raiseTether();
            end 
        end
        % Navigation algorithm - walk up gradient of the gas vent
        % Preconditions:
        % Postconditions:
        function navigateGradientWalk(obj)
            % Boundary check, note size of rover 2x2x2m
            if obj.isBoundary()
                obj.status = RobotStatus.trapped;
                return;
            end
            % Check time limit for gradient walk
            if obj.gwalk_time > obj.gwalk_time_limit
                obj.status = RobotStatus.exitGradientWalk;
                return;
            else
                obj.gwalk_time = obj.gwalk_time + 1;
            end 
            % Determine velocity based on gradient
            max_dim = length(obj.grad_map_x(1,:));
            curx = int32(10 * round(obj.cp(1), 1));
            cury = int32(10 * round(obj.cp(2), 1));
            if curx < 1
                curx = 1;
            end
            if curx > max_dim
                curx = max_dim;
            end
            if cury < 1
                cury = 1;
            end
            if cury > max_dim
                cury = max_dim;
            end
            gradx = obj.grad_map_x(cury, curx);
            grady = obj.grad_map_y(cury, curx);
            if gradx > 0.01
                obj.velocity(1) = 0.1;
            elseif gradx < -0.01
                obj.velocity(1) = -0.1;
            end
            if grady > 0.01
                obj.velocity(2) = 0.1;
            elseif grady < -0.01
                obj.velocity(2) = -0.1;
            end
            % Calculate new position
            obj.get3DVectorPosition();
            % Test air for gas compounds
            obj.executeGasSensor();
            % Lower tether
            obj.lowerTether();
        end
        % Navigation algorithm - exit gradient walk
        % Preconditions:
        % Postconditions:
        function navigateExitGradientWalk(obj)
            % Spiral x coordinates from position to home
            if obj.isOutBoundary()
                obj.status = RobotStatus.goHome;
            else
                if length(obj.spiral_x) < 2
                    distance = 1.2*sqrt((obj.vent_pos(1) - obj.cp(1))^2 + (obj.vent_pos(2) - obj.cp(2))^2 + (obj.vent_pos(3) - obj.cp(3))^2);
                    if (distance < 4.0)
                        distance = 4.0;
                    end
                    t = 0:pi/180:2*pi;
                    obj.spiral_len = length(t);
                    obj.spiral_x = (distance*0.05*(exp(0.5*t).*cos(10*t))) + obj.cp(1);
                    obj.spiral_y = (distance*0.05*(exp(0.5*t).*sin(10*t))) + obj.cp(2);
                    obj.spiral_z = (zeros(obj.spiral_len)) + obj.cp(3);
                end
                % Calculate new position
                obj.getLogSpiralPosition();
                % Test air for gas compounds
                obj.executeGasSensor();
                % Lower tether
                obj.lowerTether();
            end
        end
        % Detect if over a vent that exceeds threshold, broadcast target
        % Preconditions:
        % Postconditions:
        function executeGasSensor(obj)
            if obj.status == RobotStatus.search
                gs = obj.getGasSample();
                if gs > obj.threshold
                    notify(obj, 'validVent');
                    obj.vent_pos(1) = obj.cp(1);
                    obj.vent_pos(2) = obj.cp(2);
                    obj.status = RobotStatus.gradientWalk;
                end
            elseif obj.status == RobotStatus.gradientWalk
                gs = obj.getGasSample();
                if gs > obj.max_sample
                    obj.status = RobotStatus.exitGradientWalk;
                end
            elseif obj.status == RobotStatus.exitGradientWalk
                obj.getGasSample();
            end
        end
        % Boundary check
        % Preconditions:
        % Postconditions:
        %   is_bound = true if value border=1, false otherwise
        function is_bound = isBoundary(obj)
            global bo_map
            obj.is_bounds_or_obstacle = false;
            is_bound = false;
            max_dim = length(bo_map(1,:));
            curx = int32(10 * round(obj.cp(1), 1));
            cury = int32(10 * round(obj.cp(2), 1));
            if curx < 1
                curx = 1;
            end
            if curx > max_dim
                curx = max_dim;
            end
            if cury < 1
                cury = 1;
            end
            if cury > max_dim
                cury = max_dim;
            end
            if bo_map(cury, curx) == 1
                is_bound = true;
                obj.is_bounds_or_obstacle = true;
                % Handle 4 corners
                if (obj.cp(1) < 2 && obj.cp(2) < 2) || ...
                        (obj.cp(1) < 2 && obj.cp(2) > 98) || ...
                        (obj.cp(1) > 98 && obj.cp(2) > 98) || ...
                        (obj.cp(1) > 98 && obj.cp(2) < 2) 
                    obj.velocity = [-obj.velocity(1) -obj.velocity(2) obj.velocity(3)];   
                % Handle left bounds and right bounds
                elseif obj.cp(1) > 98 || obj.cp(1) < 2
                    obj.velocity = [-obj.velocity(1) obj.velocity(2) obj.velocity(3)];
                % Handle top bounds and bottom bounds
                elseif obj.cp(2) > 98 || obj.cp(2) < 2
                    obj.velocity = [obj.velocity(1) -obj.velocity(2) obj.velocity(3)];
                else
                %? Handle obstacles
                    obj.velocity = [-obj.velocity(1) -obj.velocity(2) obj.velocity(3)];
                end
            end
        end
        % Boundary check
        % Preconditions:
        % Postconditions:
        %   is_bound = true if outside boundary
        function is_bound = isOutBoundary(obj)
            is_bound = false;
            if (obj.cp(1) > 100 || obj.cp(1) < 0) || ...
                    (obj.cp(2) > 100 || obj.cp(2) < 0)
                is_bound = true;
            end
        end
        % Determine if current position exceeds gas threshold
        % Preconditions:
        % Postconditions:
        %   sample = sensor value from vent map
        function sample = getGasSample(obj)
            global field_map cover_map
            max_dim = length(obj.vent_map(1,:));
            curx = int32(10 * round(obj.cp(1), 1));
            cury = int32(10 * round(obj.cp(2), 1));
            if curx < 1
                curx = 1;
            end
            if curx > max_dim
                curx = max_dim;
            end
            if cury < 1
                cury = 1;
            end
            if cury > max_dim
                cury = max_dim;
            end
            sample = obj.vent_map(cury, curx);
            obj.max_vent = max(obj.max_vent, sample);
            % Record sampling
            if (obj.status == RobotStatus.gradientWalk || ...
                obj.status == RobotStatus.exitGradientWalk)
                field_map(cury, curx) = sample;
            end
            % Record coverage - area visited by robot
            cover_map(cury, curx) = 1;
            % Publish to ROS network
            msg = rosmessage(obj.pub);
            msg.Data = [obj.time sample];
            %Debug showdetails(msg);
            send(obj.pub, msg);
        end
        % Determine position
        % Preconditions:
        % Postconditions:
        function get2DLinearPosition(obj)
            cpx = obj.linear_x(obj.linear_index);
            if obj.linear_index == obj.linear_len
                cpy = obj.cp(2) - abs(obj.velocity(2));
            else
                cpy = obj.linear_slope * cpx + obj.linear_y_intercept;
            end
            cpz = 0;
            obj.cp = [cpx cpy cpz];
            obj.linear_index = obj.linear_index + 1;
            if obj.linear_index > obj.linear_len
                obj.linear_index = obj.linear_len;
            end                
        end
        % Determine position
        % Preconditions:
        % Postconditions:
        function get3DVectorPosition(obj)
            cpx = obj.velocity(1) + obj.cp(1);
            cpy = obj.velocity(2) + obj.cp(2);
            cpz = obj.velocity(3) + obj.cp(3);
            obj.cp = [cpx cpy cpz];              
        end
        % Determine position
        % Preconditions:
        % Postconditions:
        function getLogSpiralPosition(obj)
            cpx = obj.spiral_x(obj.spiral_index);
            cpy = obj.spiral_y(obj.spiral_index);
            cpz = obj.spiral_z(obj.spiral_index);
            obj.cp = [cpx cpy cpz];
            obj.spiral_index = obj.spiral_index + 1;
            if obj.spiral_index > obj.spiral_len
                obj.resetRobot();
            end                
        end
        % Lower tether
        % Preconditions:
        % Postconditions:
        function lowerTether(obj)
            if obj.tz(1, 1) > obj.max_sample
                obj.tz(1, 1) = obj.tz(1, 1) - 1;
            end
        end
        % Raise tether
        % Preconditions:
        % Postconditions:
        function raiseTether(obj)
            if (obj.tz(1, 1) < 36.0)
                obj.tz(1, 1) = obj.tz(1, 1) + 1;
            end
        end
        % Reset robot state to initial state
        % Preconditions:
        % Postconditions:
        function resetRobot(obj)
            obj.velocity = [0.2 0.2 0]; 
            obj.vent_pos = [0 0 0];
            obj.is_bounds_or_obstacle = false;
            obj.status = RobotStatus.search;
            obj.linear_x = [];
            obj.linear_index = 1;
            obj.linear_len = 0;
            obj.linear_slope = 0;
            obj.linear_y_intercept = 0;
            obj.spiral_x = [];
            obj.spiral_y = [];
            obj.spiral_z = [];
            obj.spiral_index = 1;
            obj.spiral_len = 0;
            obj.gwalk_time = 0;
        end
    end
    events (NotifyAccess = private)
        validVent
    end
end
