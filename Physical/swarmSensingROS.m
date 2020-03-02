close all

global g_time g_co2
g_time = []; g_co2 = [];

setenv('ROS_MASTER_URI','http://192.168.1.55:11311');
setenv('ROS_IP','192.168.1.51');
rosinit;
pause(5);
co2_subscriber = rossubscriber('/R21/co2');

figure;
hold on;
xlim([0 1e8]);
ylim([300 16e3]);
phandle = plot(NaN, NaN);
while true
    data = receive(co2_subscriber); % 5 sec sample rate
    g_time = [g_time data.Data(1)];
    g_co2 = [g_co2 data.Data(2)];
    if ishandle(phandle)
        set(phandle, 'XData', g_time, 'YData', g_co2);
    else
        break;
    end 
end

rosshutdown;