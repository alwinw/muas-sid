function [] = plotLogFrequency(logData)

t = logData.timestamp;
dt = diff(t);
f = 1./dt;

figure(1); clf;
subplot(211);
plot(t(2:end), f, '-o');
xlabel('Time [s]');
ylabel('Logging Frequency [Hz]');
subplot(212);
histogram(f);
xlabel('Logging Frequency [Hz]');
ylabel('Count');
fprintf('mean=%.1f Hz, sd=%.1f Hz\n', mean(f), std(f));

end