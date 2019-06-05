% Signal Processing Toolbox
% Robotics System Toolbox
% Aerospace Toolbox
% Curve Fitting Toolbox

clear; close all; format shortg

%% Import data

Data = importdata('madg2.txt');

t = Data(:, 1) ./ 1000; % Time
fSample = (length(t)-1) / (t(end)-t(1)); % Average sample rate

Yaw0   = Data(:, 2); % Z
Pitch0 = Data(:, 3); % Y
Roll0  = Data(:, 4); % X

%% Recover pitch data

Pitch0 = Pitch0 - mean(Pitch0(1338:2190));


% Flip the 'absolute values' to negative
[~, Indices] = findpeaks(-Pitch0, 'MinPeakDistance', 20);
Indices = Indices(Indices > 253 & Indices < 1261 ...
                | Indices > 2306 & Indices < 3333);
            
IndicesOdd  = Indices(1:2:end);
IndicesEven = Indices(2:2:end);

for N = 1:length(IndicesEven)
    Temp = Pitch0(IndicesOdd(N):IndicesEven(N));
    Pitch0(IndicesOdd(N):IndicesEven(N)) = -Temp;
end

%% Bandpass filters

for Angle = ["Yaw" "Pitch" "Roll"]
    eval(sprintf( ...
        '%1$s = bandpass(%1$s0, [0.25 0.55], fSample);', Angle));
end

%% Uncorrected vectors

Euler = deg2rad([Yaw Pitch Roll]);
Quat = eul2quat(Euler); 

r0 = [0 0 1];
Vec = quatrotate(Quat, r0);

PlotRange = 2558:3913;
tPlot = t(PlotRange);
VPlot = Vec(PlotRange, :);

figure(1)
plot(tPlot, VPlot(:, 1), tPlot, VPlot(:, 2), tPlot, VPlot(:, 3))
legend v_x v_y v_z

figure(2)
plot3(VPlot(:, 1), VPlot(:, 2), VPlot(:, 3))
xlabel x
ylabel y
zlabel z

%% Correction

% Project all vectors onto a rotating vertical plane
Vec(:, 1) = hypot(Vec(:, 1), Vec(:, 2));
Vec(:, 2) = zeros(length(Vec), 1);

% Flip the 'absolute values' to negative
[~, Indices] = findpeaks(-Vec(:, 1), 'MinPeakDistance', 10);
            
IndicesOdd  = Indices(1:2:end);
IndicesEven = Indices(2:2:end);

for N = 1:length(IndicesEven)
    Temp = Vec(IndicesOdd(N):IndicesEven(N), 1);
    Vec(IndicesOdd(N):IndicesEven(N), 1) = -Temp;
end

VPlot = Vec(PlotRange, :);

figure(3)
plot(tPlot, VPlot(:, 1), tPlot, VPlot(:, 2), tPlot, VPlot(:, 3))
legend v_x v_y v_z

%% Differential equation fit

% phi'' + c*phi' + k*sin(phi) = 0
dVx = bandpass(diff(Vec(:, 1)), [0.25 0.55], fSample);
dVz = bandpass(diff(Vec(:, 3)), 2*[0.25 0.55], fSample);

phi = bandpass(atan(dVz./dVx), [0.25 0.55], fSample);
phi = phi([400:1700 3500:4000]);

omega = bandpass(diff(phi), [0.25 0.55], fSample);
beta = bandpass(diff(omega), [0.25 0.55], fSample);

phi = phi(1:end-2);
omega = omega(1:end-1);

Fitting = fit([omega, sin(phi)], -beta, 'poly11');
Coeffs = coeffvalues(Fitting);
c = Coeffs(2);
k = Coeffs(3);
