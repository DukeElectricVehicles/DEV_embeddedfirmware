clear; clc; % close all;

ROT_INERTIA = 0.8489;

load ../spindown_noRotor % PARASITIC LOSSES
data = importdata('./12V_PS_D100_6.txt');

data = data(data(:,2)>.1,:); % current > .1

voltage = data(:, 1);
current = data(:, 2);
rpm = data(:, 4);
toShift = fix(1 / (min(rpm) * (data(2,6)-data(1,6))/1000 / 60));
for i=1:length(rpm)-toShift % compensate for moving average having phase lag
    rpm(i) = rpm(i+toShift);
    toShift = fix(1 / (rpm(i) * (data(2,6)-data(1,6))/1000 / 60));
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

accelComp = accel - polyval(PARASITIC_LOSSES_ACC_OF_FLYWHEEL_RPS, omega);


torque = ROT_INERTIA .* accelComp;
mPower = torque .* omega;
mPower = smooth(mPower, 50, 'sgolay');

eff = mPower ./ ePower;
eff = smooth(eff, 101, 'sgolay');

figure(1);clf;
subplot(3,1,1);
plot(time, rpm_motor);
xlabel('Time (s)'); ylabel('Motor RPM');
subplot(3,1,2);
plot(rpm_motor, voltage, 'DisplayName','Voltage');
ylabel('Voltage (V)'); yyaxis right
plot(rpm_motor, current, 'DisplayName','Current');
ylabel('Current (A)'); xlabel('RPM of motor');
legend show
subplot(3,1,3);
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
ylim([0.6, 1]); xlim([0,100]);
ylabel('efficiency');
xlabel('Electrical power (W)');
grid on;
title('efficiency vs power');