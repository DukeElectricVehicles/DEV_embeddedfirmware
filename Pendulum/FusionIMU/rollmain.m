clear;close all;clc;

%data = importdata('blue50_take3.txt');
%data = importdata('blackbluewide50.txt');
%data = importdata('blackbluewide50_2.txt');
%data = importdata('blackblue50_2.txt');
%data = importdata('blackblue50_alum.txt');
%data = importdata('blackblue50_alum_wide.txt');
%data = importdata('blackblack_6_21_rear.txt');
data = importdata('blackblack_6_21_right.txt');
%data = importdata('blackblack_6_21_left.txt');
%data = importdata('blackblack_6_21_MD2.txt');

elapsed = data(:, 1) ./ 1000;
yaw = data(:, 2);
pitch = data(:, 3);
roll = data(:, 4);


%Extract theta from IMU---------------------------------
angle = data(:, 2:4) ./ (180 / pi);
quat = eul2quat(angle);
r = [0 0 1];
vec = quatrotate(quat, r);
%pos0 = vec(25, :)
%pos1 = vec(120, :)
pos0 = [-0.062267945372983  -0.018463536001600   0.997888671554771];
pos1 = [-0.393300136124167   0.914490245036143  -0.094987339463987];
pos1 = pos1 - pos0 * dot(pos1, pos0);
pos1 = pos1 / norm(pos1);
theta = atan2(vec * pos0', vec * pos1');
theta = (theta - theta(end)) * 180 ./ pi;
theta = wrapTo180(theta);
%theta = roll;

plot(theta); hold on;
envel = envelope(theta, 100, 'peak');
plot(envel);

wheelRad = 0.495 / 2;
mass = 29.7;
cg = 0.0431; %cg below axle
g = 9.8;

%Find windows---------------------------------------------
windows = [];
state = 0;
start = 0;
for i = 101:length(envel) - 201
    if state == 0
        if all(envel(i-100:i-50) < envel(i)) && envel(i) > 50 && all(envel(i+1:i+100) < envel(i))
            start = i + 000;
            state = 1;
        end
    else
        if std(envel(i-100:i+100)) < 0.5
            state = 0;
            windows = [windows; start, i];
            line([start, start], [-100, 100], 'Color', 'green', 'LineWidth', 1);
            line([i, i], [-100, 100], 'Color', 'red', 'LineWidth', 1);
        end
    end
end

theta = theta ./ 180 .* pi;

%Extract meaning from windows----------------------------------------
log = [];
for i = 1:size(windows, 1)
    start = windows(i, 1);
    stop = windows(i, 2);
    thetaCut = theta(start:stop);
    elapsedCut = elapsed(start:stop);
    
    thetaFit = csaps(elapsedCut, thetaCut, 0.99, elapsedCut);
    thetaCut = thetaFit;
    
    [maxes, indsMaxes] = findpeaks(thetaCut, 'MinPeakDistance',10); hold on;
    [mins, indsMins] = findpeaks(-thetaCut, 'MinPeakDistance',10); hold on;
    mins = -mins;
    
    maxHeight = maxes(end-2:end);
    minHeight = mins(end-2:end);
    avg = mean([maxHeight; minHeight]);
    %findpeaks(thetaCut, 'MinPeakDistance',10);
    %findpeaks(-thetaCut, 'MinPeakDistance',10);
    thetaCut = thetaCut - avg;
    
    veloCut = smooth(gradient(thetaCut, elapsedCut) * wheelRad, 21);
    distCut = cumsum(abs(gradient(thetaCut))) * wheelRad;
    
    pe = mass * g * -cos(thetaCut) * cg;
    
    [c, t] = rollfit(elapsedCut, thetaCut);
    %c = 1; t = 1;
    c = 1/c;
    
    figure;
    %findpeaks(max(thetaCut, 0), 'MinPeakDistance',10); hold on;
    plot(thetaCut); hold on;
    plot(-thetaCut);
    [peaks, inds] = findpeaks(max(thetaCut, 0), 'MinPeakDistance',10);
    crrHist = [];
    
    
    for j = 2:length(inds)
        forceDrag = (pe(inds(1)) - pe(inds(j))) / (distCut(inds(j)) - distCut(inds(1)));
        forceNormal = mass * g;
        crr = forceDrag / forceNormal;
        crrHist = [crrHist; j, crr];
    end
    
    figure(10);
    plot(crrHist(:, 1), crrHist(:, 2)); hold on;
    
    fprintf('%.4f %d crr %.5f\n', c, t, crr);
    log = [log; c, t, crr];
end

log ./ log(1, :)

mean(log(:, 3))