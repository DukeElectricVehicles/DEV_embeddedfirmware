%% Gerry Chen
%   calculates moment of inertia from a setup where
%   a mass is attached to the wheel and the pendulum frequency is measured
clear;
g = 9.81;
% m = 0.515;
% d = (157.85+70 + 42.9/2) * 1e-3;
m = 0.123;
d = (140.49 - 10/2 - 38.4/2) * 1e-3;
m_STD = 0.001;
d_STD = 0.003;

allW = [];
allW_STD = [];
allI0 = [];
allI0_STD = [];

files = dir();
for i = 1:length(files)
    filename = files(i).name;
    if (startsWith(filename,'stockHubRotorSprocket') && endsWith(filename,'.mat'))
        load(filename)
        Fs = 1/mean(diff(t));
        xdft = fft(x);
        xdft = xdft(1:fix(length(x)/2+1));
        DF = Fs/length(x); % frequency increment
        freqvec = 0:DF:Fs/2;
        [v,ind] = max(abs(xdft));
        freq = freqvec(ind);

        figure(1);clf;
        subplot(2,1,1);
        plot(t,x);
        subplot(2,1,2);
        plot(freqvec,abs(xdft)); hold on;
        plot(freq, abs(xdft(ind)),'r^');

        fprintf('frequency: %.5fHz +/- %.5fHz\t',freq,DF);
        W = freq*2*pi;
        W_STD = DF * 2*pi;
        I0 = m*g*d/W^2 - m*d^2;
        I0_STD = m_STD*abs(g*d/W^2 - d^2) + ...
                 d_STD*abs(m*g/W^2 - 2*m*d) + ...
                 W_STD*abs(2*m*g*d/W^3);
        fprintf('I0: %.5f kg.m^2 +/- %.5f kg.m^2\n',I0,I0_STD);
        allI0(end+1) = I0;
        allI0_STD(end+1) = I0_STD;
        
        allW(end+1) = W;
        allW_STD(end+1) = W_STD;
    end
end

fprintf('I0 = %.5f +/- %.5f kg.m^2\n',mean(allI0),std(allI0));