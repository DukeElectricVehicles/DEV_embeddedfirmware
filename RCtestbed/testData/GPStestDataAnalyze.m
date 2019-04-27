%%  Gerry Chen
%   GPStestDataAnalyze.m

clear;

load manualControl_19-04-26_01_grossHall

figure(1);clf;
plot(curPosLLH_lon, curPosLLH_lat, 'k-', 'DisplayName','manual control'); hold on;

load selfDrive_19-04-26_01_grossHall
indsToPlot = (curPosLLH_lon~=0);
plot(curPosLLH_lon(indsToPlot), curPosLLH_lat(indsToPlot), 'r-', 'DisplayName','path following');
plot(waypoint_lon, waypoint_lat, 'k^', 'DisplayName','path waypoints');

grid on;
daspect([1,cosd(36),1]);