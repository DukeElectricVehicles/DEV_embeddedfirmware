%%  Gerry Chen

clear;
data = importdata('hallTimings_mitsuba_MR1.txt');
numpoles = 16;

figure(3);clf;
colors = {'k','b','c','g','r','m'};
for i = 1:fix(length(data)/numpoles)
    for j = 1:6
        plot(0:(numpoles-1),...
            data(numpoles*(i-1)+1:numpoles*i,j) /...
                  mean(data(numpoles*i,:)),...
            colors{j});
        hold on;
    end
end
legend('Hall transition 0','Hall transition 1','Hall transition 2','Hall transition 3','Hall transition 4','Hall transition 5');
xlabel('pole pair #'); ylabel('duration between hall ticks (relative to average tick duration)');
title('Hall Sensor Timing');

