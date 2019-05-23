clear;clc;close all;

data = importdata('data.txt');
%data = importdata('pendulumTestData2.tsv');
data = importdata('madg.txt');

elapsed = data(:, 1) ./ 1000;
yaw = data(:, 2);
pitch = data(:, 3);
roll = data(:, 4);
angle = data(:, 2:4) ./ (180 / pi);
accX = data(:, 5);
accY = data(:, 6);
accZ = data(:, 7);
acc = data(:, 5:7);
quat = eul2quat(angle);




r = [0 0 1];
vec = quatrotate(quat, r);

pos0 = vec(50, :);
pos1 = vec(230, :);

pos1 = pos1 - pos0 * dot(pos1, pos0);
pos1 = pos1 / norm(pos1);

pos2 = cross(pos0, pos1);
projBad = pos2 * vec';

theta = atan2(vec * pos1', vec * pos0');

figure
plot(theta)
hold on;
plot(abs(theta));
figure;
plot(projBad);