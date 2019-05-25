%%  Gerry Chen

clear;
data = importdata('hallTimings_mitsuba_brokenMagneticRing.txt');
numpoles = 16;

figure(3);clf;
colors = {'k','b','c','g','r','m'};
for i = 1:fix(length(data)/numpoles)
    for j = 1:6
        plot(0:(numpoles-1),...
            data(numpoles*(i-1)+1:numpoles*i,j) /...
                    data(numpoles*i,1),...
            colors{j});
        hold on;
    end
end
legend('Hall pos 0','Hall pos 1','Hall pos 2','Hall pos 3','Hall pos 4','Hall pos 5');
xlabel('pole pair #'); ylabel('relative timing');
title('Hall Sensor Timing');

