%%
clear;

files = dir();
for i = 1:length(files)
    clear data;
    filename = files(i).name;
    if (startsWith(filename,'stockHubWheelGood') && endsWith(filename,'.txt'))
        data = importdata(filename);
        t = data(:,1) / 1e6;
        x = data(:,2) - mean(data(:,2));
        save([filename(1:end-4),'.mat'],'x','t');
    end
end

% files = dir();
% for i = 1:length(files)
%     clear data;
%     filename = files(i).name;
%     if (startsWith(filename,'stockHubWheel') && endsWith(filename,'.txt'))
%         data = importdata(['bak/',filename]);
%         diary([filename]);
%         t = data.t;
%         x = data.x;
%         data = [t,x];
%         for j = 1:length(t)
%             fprintf('%d\t%d\n',t(j)*1e6,x(j));
%         end
%         diary off
%         save(filename, 'data','-ascii','-tabs')
%     end
% end