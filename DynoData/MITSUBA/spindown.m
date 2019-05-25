clear; %close all;

ACCEL_WINDOW = 54;
ROT_INERTIA = 0.8489;
% ROT_INERTIA = 0.06175; % stock hub wheel

% data = importdata('spindown_VESCon_badSprocket.txt');
% data = importdata('spindown_noFlywheel/stockHubWheel_outofdyno_disconnected.txt');
data = importdata('spindown_noRotor.txt');

voltage = data(:, 1);
current = data(:, 2);
rpm = data(:, 4); % /16 is a manual correction

for i = 1:length(rpm) - 2%fix glitches in rpm readout
   if (rpm(i) > 0) && (rpm(i+2) > 0) && (rpm(i+1) == 0)
       rpm(i+1) = rpm(i);
   end
end

rpm = smooth(rpm, 16);

velo = rpm * 2 * pi / 60;
time = data(:, 6) ./ 1000;

accel = gradient(velo)./gradient(time);
accel = smooth(accel,16);
accel = smooth(accel,250,'sgolay');
power = accel * ROT_INERTIA .* velo;

startWindow = 250;
endWindow = length(rpm)-70;

for i = 50:length(velo)
   if ((velo(i) > velo(i - 49)) && (velo(i) > velo(i + 49)) && (current(i+100) < .1))
       startWindow = i + 250;
       break;
   end
end

for i = startWindow:length(velo)
   if velo(i - 49) < 8 && velo(i) < 8
       endWindow = i;
       break;
   end
end

figure(1); clf;
plot(rpm); hold on;
line([startWindow, startWindow], [0, 100], 'Color', 'black', 'LineWidth', 3);
line([endWindow, endWindow], [0, 100], 'Color', 'red', 'LineWidth', 3);
yyaxis right
plot(current.*voltage)
plot(power)
ylim([-10,5]);

veloCut = velo(startWindow:endWindow);
accelCut = accel(startWindow:endWindow);
rpmCut = rpm(startWindow:endWindow);
powerCut = power(startWindow:endWindow);

rpmCutVals = linspace(rpmCut(1),rpmCut(end),100);
indsCut = zeros(size(rpmCutVals));
for i = 1:length(rpmCutVals)
    [~,indsCut(i)] = min((rpmCutVals(i)-rpmCut).^2);
end
veloCut = veloCut(indsCut);
accelCut = accelCut(indsCut);
rpmCut = rpmCut(indsCut);
powerCut = powerCut(indsCut);

figure(2);clf;
plot(rpmCut, accelCut);
xlabel('v (RPM)');
ylabel('a (RPM/s)');

coeffs = polyfit(veloCut, accelCut, 3)
hold on;
plot(rpmCut, polyval(coeffs, veloCut));

coeffsLoss = polyfit(rpmCut, powerCut, 3);
yyaxis right;
plot(rpmCut, powerCut);
plot(rpmCut, polyval(coeffsLoss, rpmCut));
fprintf('Parasitic loss at 322RPM: %.5fW\n', polyval(coeffsLoss,322));
fprintf('%.5e\t',coeffsLoss);
fprintf('\n');

PARASITIC_LOSSES_ACC_OF_FLYWHEEL_RPS = coeffs;
PARASITIC_LOSSES_POWER_OF_FLYWHEEL_RPS = coeffsLoss;
save('spindown_noRotor',...
    'PARASITIC_LOSSES_ACC_OF_FLYWHEEL_RPS',...
    'PARASITIC_LOSSES_POWER_OF_FLYWHEEL_RPS');