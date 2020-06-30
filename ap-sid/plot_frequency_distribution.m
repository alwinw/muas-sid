function [] = plot_frequency_distribution(t,u)

assert(length(t) == length(u));

Fs = 1 / mean(diff(t)); % sampling frequency
T = 1 / Fs; % sampling period
L = length(u);
t = (0:L-1)*T;
y = fft(u);
P2 = abs(y/L);
P1 = P2(1:floor(L/2)+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;
plot(f,P1);
grid on;
set(gca, 'YScale', 'log');

end