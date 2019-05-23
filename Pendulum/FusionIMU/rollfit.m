function [c, t] = rollfit(elapsed,roll)
    
    roll = roll - roll(end);

    elapsed = elapsed - elapsed(1);
    envl = envelope(roll, 100, 'peak');
    figure;
    subplot(2, 1, 1);
    plot(roll, '.'); hold on;
    plot(envl);
    envl = envl ./ max(envl);
    subplot(2, 1, 2);
    plot(elapsed, envl, '.'); hold on;

    x0 = [0.03];
    fiteq = @(opt, x) exp(-opt(1).*x);

    loss = @(params) sum((fiteq(params, elapsed) - envl).^2);
    x = fminsearch(loss, x0);
    
    plot(elapsed, fiteq(x, elapsed));
    t = find(envl < 0.1, 1);
    
    c = x;
end

