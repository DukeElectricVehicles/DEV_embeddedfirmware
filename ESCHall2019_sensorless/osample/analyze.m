data = importdata('samplebad3.txt');

data = data(1:350, :);
time = data(:,6);
vsx = data(:, 1:3);
dhall = data(:,4);
dbemf = data(:,5);

time = time-time(1);
time = mod(time, 2^16);

vsx(:,1) = vsx(:,1)/2;

bemfchange = dbemf ~= circshift(dbemf,1);
bemfchange = mod(cumsum(bemfchange),2);

figure(6);clf;
yyaxis left
plot(time,vsx(:,1),'k.'); hold on;
plot(time,vsx(:,2),'r.');
plot(time,vsx(:,3),'g.');
ylim([-100,max(vsx(:))*1.1]);
yyaxis right
plot(time,bemfchange,'k-','LineWidth',5); hold on
plot(time,dhall/2+0.25)