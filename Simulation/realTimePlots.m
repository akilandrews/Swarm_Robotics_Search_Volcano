% (GUI) Realtime plots of swarm-robotics volcanic eruption forecast research data
% Preconditions:
%   publisher = ros publisher node subscribing to
%   mission_time = number of timesteps robot is sampling
% Postconditions:
function realTimePlots(publisher, mission_time)
    if nargin < 1
        fprintf('Error: not enough arguments');
        return;
    end
    if ~robotics.ros.internal.Global.isNodeActive
        fprintf('Error: cannot connect to global node\n');
        return;
    end
    figure;
    hold on;
    xlim([0 mission_time]);
    ylim([0 6]);
    g_time = zeros(1, mission_time);
    g_co2 = zeros(1, mission_time);
    co2_subscriber = rossubscriber(sprintf('/%s/co2', publisher));
    phandle = plot(NaN, NaN);
    j = 1;
    while j < mission_time
        msg = receive(co2_subscriber);
        %Debug showdetails(msg);
        g_time(j) = msg.Data(1);
        g_co2(j) = msg.Data(2);
        set(phandle, 'XData', g_time(1:j), 'YData', g_co2(1:j));
        j = j + 1;
    end
end
