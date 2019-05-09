clear; clc; %close all;
figure(1);clf;figure(2);clf;figure(3);clf;

ACCEL_WINDOW = 1;
ROT_INERTIA = 0.8489;

load spindown_poweredOff_badSprocket

filesStruct = dir('16V_VESC/*.txt');

for i = 1:numel(filesStruct)
    filename = filesStruct(i).name;
%     if (~contains(filename,'fullDuty'))
%         continue
%     end
    filePath = strcat(filesStruct(i).folder, '/', filename)
    
    data = importdata(filePath);
    data = data(data(:,2)>.1,:); % current > .1
    
    voltage = data(:, 1);
    current = data(:, 2);
    rpm = data(:, 4);% .* 53 ./ 54;

    for i = 1:length(rpm) - 2%fix glitches in rpm readout
       if (rpm(i) > 0) && (rpm(i+2) > 0) && (rpm(i+1) == 0)
           rpm(i+1) = rpm(i);
       end
    end

    rpm = smooth(rpm, 21);
    rpmMotor = rpm;% .* 60 ./ 72;

    velo = rpm * 2 * pi / 60;
    throttle = data(:, 5);
    time = data(:, 6) ./ 1000;

    ePower = voltage .* current;
    ePower = smooth(ePower, 50, 'sgolay');

    accel = gradient(velo)./gradient(time);

    accel = smooth(accel, 41);

    accelComp = accel - polyval(PARASITIC_LOSSES, velo);

    torque = ROT_INERTIA .* accelComp;
    mPower = torque .* velo;
    eff = mPower ./ ePower;
    
    filename = strrep(filename,'_',' ');
    filename = strrep(filename,',','.');
    
    figure(1);
%     yyaxis left
    plot(rpmMotor, eff, '-', 'DisplayName', filename); hold on;
%     yyaxis right
%     plot(rpmMotor, mPower, '-');
    
    figure(2);
    scatter3(rpmMotor, mPower, eff, '.', 'DisplayName', filename); hold on;
    
    figure(3);
    plot(rpmMotor, current, '.', 'DisplayName', filename); hold on;
    
    figure(4);
    plot(rpm,accel, 'DisplayName',filename); hold on;
end

figure(1);
legend(gca,'show','Location','South');
% yyaxis left
xlabel('RPM'); ylabel('efficiency'); title('efficiency vs speed');
grid on;
ylim([0.2, 1]);
% yyaxis right
% ylabel('Power');

figure(2);
legend(gca,'show');
xlabel('RPM'); ylabel('Power'); zlabel('Efficiency'); title('Efficiency Map');
grid on;
zlim([0.84, 0.9]);
%ylim([0, 5]);
%xlim([0, 1200]);

figure(3);
legend(gca,'show');
xlabel('RPM'); ylabel('Current'); title('Current vs Speed');
grid on;

figure(4);
legend show
xlabel('RPM'); ylabel('Acceleration');
grid on;