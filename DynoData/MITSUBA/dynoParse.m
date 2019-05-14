clear; clc; % close all;

ROT_INERTIA = 0.8489;

load spindown_poweredOff_badSprocket % PARASITIC LOSSES
% data = importdata('16V_VESC/16V1A_FOC_sensorless.txt');
data = importdata('16V_VESC/16V_fullDuty2.txt');

data = data(data(:,2)>.1,:); % current > .1

voltage = data(:, 1)*24.67/24.62;
current = data(:, 2)*.999/.989;
rpm = data(:, 4);

for i = 1:length(rpm) - 2%fix glitches in rpm readout
   if (rpm(i) > 0) && (rpm(i+2) > 0) && (rpm(i+1) == 0)
       rpm(i+1) = rpm(i);
   end
end

rpm = smooth(rpm, 21);

velo = rpm * 2 * pi / 60;
throttle = data(:, 5);
time = data(:, 6) ./ 1000;

ePower = voltage .* current;
ePower = smooth(ePower, 50, 'sgolay');

accel = gradient(velo)./gradient(time);

% accel = smooth(accel, 21);

accelComp = accel - polyval(PARASITIC_LOSSES, velo);


torque = ROT_INERTIA .* accelComp;
mPower = torque .* velo;
eff = mPower ./ ePower;

figure(1);clf;
plot(rpm,ePower, 'DisplayName','Electrical Power');
hold on;
plot(rpm,mPower, 'DisplayName','Mechanical Power');
ylabel('Power (W)');
grid on;

figure(2);clf;
plot(rpm, eff);
ylim([0.6, 1]);
ylabel('efficiency');
xlabel('RPM');
grid on;
title('efficiency vs speed');