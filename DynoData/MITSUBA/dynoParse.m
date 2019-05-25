clear; clc; % close all;

ROT_INERTIA = 0.8489;

load spindown_noRotor % PARASITIC LOSSES
% data = importdata('16V_VESC/16V1A_FOC_sensorless.txt');
data = importdata('12V_DEVsensorless/12V_setupTest12.txt');

data = data(data(:,2)>.1,:); % current > .1

voltage = data(:, 1);
current = data(:, 2);
rpm = data(:, 4);

for i = 1:length(rpm) - 2%fix glitches in rpm readout
   if (rpm(i) > 0) && (rpm(i+2) > 0) && (rpm(i+1) == 0)
       rpm(i+1) = rpm(i);
   end
end

% rpm = smooth(rpm, 54);
rpm = smooth(rpm, 21);

omega = rpm * 2 * pi / 60;
rpm_motor = rpm * 54/72;
velo_mph = rpm_motor*(20/12/5280*pi)*60;
throttle = data(:, 5);
time = data(:, 6) ./ 1000;

ePower = voltage .* current;
% ePower = smooth(ePower, 50, 'sgolay');

accel = gradient(omega)./gradient(time);

% accel = smooth(accel, 101, 'sgolay');

accelComp = accel - polyval(PARASITIC_LOSSES_ACC_OF_FLYWHEEL_RPS, omega)/1000;


torque = ROT_INERTIA .* accelComp;
mPower = torque .* omega;
mPower = smooth(mPower, 50, 'sgolay');

eff = mPower ./ ePower;
% eff = smooth(eff, 101, 'sgolay');

figure(1);clf;
subplot(2,1,1);
plot(rpm_motor, voltage, 'DisplayName','Voltage');
ylabel('Voltage (V)'); yyaxis right
plot(rpm_motor, current, 'DisplayName','Current');
ylabel('Current (A)'); xlabel('RPM of motor');
legend show
subplot(2,1,2);
plot(rpm_motor,ePower, 'DisplayName','Electrical Power');
hold on;
plot(rpm_motor,mPower, 'DisplayName','Mechanical Power');
ylim([0,100]); ylabel('Power (W)'); xlabel('RPM of motor');
legend show;
grid on;

figure(2);clf;
plot(velo_mph, eff);
ylim([0.6, 1]); xlim([0,max(velo_mph)]);
ylabel('efficiency');
xlabel('speed (mph)');
grid on;
title('efficiency vs speed');

figure(3);clf;
plot(ePower, eff);
ylim([0.6, 1]); xlim([0,max(ePower)]);
ylabel('efficiency');
xlabel('Electrical power (W)');
grid on;
title('efficiency vs power');