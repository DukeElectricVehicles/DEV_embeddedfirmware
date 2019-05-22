clear;close all;clc;

data = importdata('madg2.txt');

elapsed = data(:, 1) ./ 1000;
yaw = data(:, 2);
pitch = data(:, 3);
roll = data(:, 4);

plot(roll); hold on;
envel = envelope(roll, 100, 'peak');
plot(envel);

windows = [];
state = 0;
start = 0;
for i = 201:length(envel) - 201
    if state == 0
        if all(envel(i-200:i-100) < envel(i)) && envel(i) > 30 && all(envel(i+1:i+100) < envel(i))
            start = i + 200;
            state = 1;
        end
    else
        if std(envel(i-100:i+100)) < 0.2
            state = 0;
            windows = [windows; start, i];
            line([start, start], [-100, 100], 'Color', 'green', 'LineWidth', 1);
            line([i, i], [-100, 100], 'Color', 'red', 'LineWidth', 1);
        end
    end
end
log = [];
for i = 1:size(windows, 1)
    start = windows(i, 1);
    stop = windows(i, 2);
    c = rollfit(elapsed(start:stop), roll(start:stop));
    fprintf('%.4f %d %d \n', c, start, stop);
    log = [log; c];
end
fprintf('%f \n', std(log) / mean(log));