clear;close all;clc;

data = importdata('60to70.txt');

elapsed = data(:, 1) ./ 1000;
yaw = data(:, 2);
pitch = data(:, 3);
roll = data(:, 4);

angle = data(:, 2:4) ./ (180 / pi);
quat = eul2quat(angle);
r = [0 0 1];
vec = quatrotate(quat, r);
pos0 = vec(50, :);
pos1 = vec(230, :);
pos1 = pos1 - pos0 * dot(pos1, pos0);
pos1 = pos1 / norm(pos1);
theta = atan2(vec * pos0', vec * pos1');
theta = (theta - theta(1)) * 180 ./ pi;
theta = wrapTo180(theta);
%theta = roll;

plot(theta); hold on;
envel = envelope(theta, 100, 'peak');
plot(envel);

windows = [];
state = 0;
start = 0;
for i = 101:length(envel) - 201
    if state == 0
        if all(envel(i-100:i-50) < envel(i)) && envel(i) > 50 && all(envel(i+1:i+100) < envel(i))
            start = i + 100;
            state = 1;
        end
    else
        if std(envel(i-100:i+100)) < 0.2
            state = 0;
            windows = [windows; start, i];
            line([start, start], [-1, 1], 'Color', 'green', 'LineWidth', 1);
            line([i, i], [-1, 1], 'Color', 'red', 'LineWidth', 1);
        end
    end
end
log = [];
for i = 1:size(windows, 1)
    start = windows(i, 1);
    stop = windows(i, 2);
    [c, t] = rollfit(elapsed(start:stop), theta(start:stop));
    c = 1/c;
    fprintf('%.4f %d\n', c, t);
    log = [log; c, t];
end
fprintf('STD percent %f \n', std(log) ./ mean(log));
log ./ log(1, :)