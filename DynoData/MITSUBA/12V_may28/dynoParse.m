clear; clc; % close all;

ROT_INERTIA = 0.8489;

load ../spindown/spindown_noRotor_may27_before % PARASITIC LOSSES
data = importdata('./DEV_-10advance_2.txt');

data = data(data(:,2)>.1,:); % current > .1
data = data(50:end,:);

time = data(:,6)/1000;
throttle = data(:, 5);
voltage = data(:, 1);
current = data(:, 2);
rpm = data(:, 4);

for i = 1:length(rpm) - 2%fix glitches in rpm readout
   if (rpm(i) > 0) && (rpm(i+2) > 0) && (rpm(i+1) == 0)
       rpm(i+1) = rpm(i);
   end
end
glitches = find(abs(diff(rpm))>5);
for glitch = glitches'
    rpm(glitch+1) = rpm(glitch);%mean(rpm([glitch,glitch+2]));
end
rpm = 1./smooth(time,1./rpm, 54);
% rpm = circshift(rpm, -27);
% rpm(end-27:end) = rpm(end-28);

rpm = smooth(time,rpm, 1001, 'sgolay', 5);

omega = rpm * 2 * pi / 60;
rpm_motor = rpm * 54/72;
velo_mph = rpm_motor*(20/12/5280*pi)*60;

ePower = voltage .* current;
ePower = smooth(time,ePower, 150, 'sgolay');

accel = gradient(omega)./gradient(time);

accel = smooth(time,accel, 401, 'sgolay');

accelComp = accel - polyval(PARASITIC_LOSSES_ACC_OF_FLYWHEEL_RPS, omega);


torque = ROT_INERTIA .* accelComp;
mPower = torque .* omega;
mPower = smooth(time,mPower, 150, 'sgolay');

eff = mPower ./ ePower;
eff = smooth(time,eff, 404, 'sgolay');

figure(1);clf;
subplot(3,1,1);
plot(time, rpm_motor);%, time, data(:,4)*54/72);
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
plot(rpm_motor, accelComp);
xlabel('RPM'); ylabel('Acceleration');
ylim([0,10]);
grid on;

figure(3);clf;
plot(velo_mph, eff);
ylim([0.6, 1]); xlim([0,max(velo_mph)]);
ylabel('efficiency');
xlabel('speed (mph)');
grid on;
title('efficiency vs speed');

figure(4);clf;
plot(ePower, eff);
ylim([0.6, 1]); xlim([0,100]);
ylabel('efficiency');
xlabel('Electrical power (W)');
grid on;
title('efficiency vs power');