%%  Gerry Chen
%   newPathGen.m

load manualDrive_19-04-27_02_grossHall

[odoUni, indsUni] = unique(odo);

latSpline = fit(odo(indsUni), curPosLLH_lat(indsUni),'smoothingspline','SmoothingParam',.01);
lonSpline = fit(odo(indsUni), curPosLLH_lon(indsUni),'smoothingspline','SmoothingParam',.01);

%% visualize real quick
dVals = linspace(odo(1),odo(end)-5, 1e4);
figure(1);clf;
img = imread('satelliteImage_1.png');
imagesc(...
    'YData',[36.0024697,36.0013262],'XData',[-78.9466732,-78.9452114],...
    'CData',img); hold on;
plot(curPosLLH_lon, curPosLLH_lat, 'k.','MarkerSize',1); hold on;
plot(lonSpline(dVals), latSpline(dVals),'r-');

grid on;
% daspect([1,cosd(36),1]);
axis square

%% calculate waypoints
[xp,xpp] = differentiate(lonSpline, dVals);
[yp,ypp] = differentiate(latSpline, dVals);
curvature = abs(xp.*ypp - yp.*xpp)./(xp.^2+yp.^2).^(3/2);
% dTds = @(odo) differentiate(T,odo);

% figure(2);clf;
% plot(dVals, curvature);
curvature = smooth(curvature,300,'sgolay');
scaledCurvature = curvature./max(curvature);

% scatter(lonSpline(dVals), latSpline(dVals), 1+fix(100*scaledCurvature));
max(curvature)

intermediateIntegral = cumtrapz(scaledCurvature);
intermediateLocations = linspace(0,intermediateIntegral(end), 200) - 5;
[v,inds] = min(abs(intermediateIntegral - intermediateLocations));
distPerInd_m = dVals(2)-dVals(1);
while (any(abs(diff(inds))>(1/distPerInd_m)))
    [v_,toChange] = max(abs(diff(inds)));
    inds = [inds(1:toChange), fix(mean(inds(toChange:toChange+1))), inds(toChange+1:end)];
end
% inds = inds(2:end); % manually remove the first one because I don't like it
newWaypoints_odo = dVals(inds);
newWaypoints_lat = latSpline(newWaypoints_odo);
newWaypoints_lon = lonSpline(newWaypoints_odo);

plot(newWaypoints_lon, newWaypoints_lat,'r^');

%% export as .h file
contentsbegin = "static const double waypoints_RTK[][2] = {";
contentsmiddle = sprintf('\t{%.8f, %.8f},\n', [newWaypoints_lat, newWaypoints_lon]');
contentsend = "};";
contents = contentsbegin+newline+...
    extractBefore(contentsmiddle,length(contentsmiddle)-1)+...
    contentsend;

fid = fopen('test.h','w');
fwrite(fid, contents);
fclose(fid);

% fid = fopen('../BMS_wireless/grossHallLoop3.h','w');
% fwrite(fid, contents);
% fclose(fid);