%%  Gerry Chen
%   simulate

clear all
global newWaypoints_lat newWaypoints_lon
global LATPERM LONPERM delHeadingMax MINTURNRAD

totalTime = 200;
speed = 1.5;
dt = .01;
numTimeSteps = totalTime / dt;
LATPERM = 1 / 6371000 / 0.01745329252;
LONPERM = LATPERM / cosd(36);
MINTURNRAD = 8;
delHeadingMax = speed*dt / MINTURNRAD;

load path4
curPosLLH_lat = zeros(100,1);
curPosLLH_lon = zeros(100,1);
curPosLLH_lat(1) = 36.0021;
curPosLLH_lon(1) = -78.945468;
heading = pi/2;
    
for i = 2:numTimeSteps
    curPosLLH_lat(i) = curPosLLH_lat(i-1) + sin(heading)*speed*dt * LATPERM;
    curPosLLH_lon(i) = curPosLLH_lon(i-1) + cos(heading)*speed*dt * LONPERM;
    
	[heading, isDone] = getHeadingBezier(curPosLLH_lat(i),curPosLLH_lon(i),heading);

    if isDone
        break;
    end
end

figure(1);clf;
plot(newWaypoints_lon, newWaypoints_lat, 'g^','MarkerSize',4,'DisplayName','path waypoints'); hold on;
plot(curPosLLH_lon, curPosLLH_lat,'k-','DisplayName','Vehicle Path');
grid on;

legend show
grid on;
xlim([-78.9466732,-78.9452114]);
ylim([36.0013262,36.0024697]);
daspect([1,cosd(36),1]);
title('MPC Algorithm - Simulation');
xlabel('Latitude (deg)'); ylabel('Longitude (deg)');