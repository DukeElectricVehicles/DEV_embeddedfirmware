%%  Gerry Chen
%   GPStestDataConvert.m

clear;

filename = 'selfDrive_19-04-27_13_grosshall_4.tsv';
manualLatOffset = 36.00;
manualLonOffset = -78.94;

data = dlmread(filename,'\t');

odo = data(:,1);

if (size(data,2) == 31) % speed control
    data(:,3:8) = [];
end

ctrl_steer = data(:,3);
ctrl_throttle = data(:,4);
ctrl_brake = data(:,5);

curPosLLH_lat = data(:,7) / 1e6 + manualLatOffset;
curPosLLH_lon = data(:,8) / 1e6 + manualLonOffset;
waypoint_lat = data(:,9) / 1e6 + manualLatOffset;
waypoint_lon = data(:,10) / 1e6 + manualLonOffset;

delLat_m = data(:,12);
delLon_m = data(:,13);
setLat_m = data(:,14);
setLon_m = data(:,15);

velN_mmpers = data(:,16);
velE_mmpers = data(:,17);

curHeading = data(:,19);
desHeading = data(:,20);
isPathComplete = data(:,21);
progress = data(:,22);
isAutonomous = data(:,23); % as of run 02 and later

clear data;
save(filename(1:end-4));