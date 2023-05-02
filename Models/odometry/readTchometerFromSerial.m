%% DATA Aquation
clear all
ports = serialportlist
% measureAnalogIRtoSerial/
[x, y] = read_serial(1500, ports{end}, 115200)

plot(x,y)
%% Process signal, get rad/s
min_val=0
max_val=700
clamp_y = max(min(y, max_val), min_val);
% plot(x, clamp_y); hold; scatter(x, clamp_y)
inv_y = max_val - clamp_y  
% plot(x, y)
% plot(x,inv_y); hold; scatter(x, inv_y)

filter = [-1,4,-1]; % Gausian filter
edge_y = conv(clamp_y, filter, 'same');
threshold = 1600;
peaks_pos = find(edge_y > threshold)

spike_x = x(peaks_pos)
peaks = ones(1, length(peaks_pos)) * threshold 
% plot(x, edge_y ); hold; stem(spike_x, peaks)

d_spike_x = diff(spike_x)

spikes = 6;
rad_spike = deg2rad(360 / spikes)
omega =  arrayfun(@(x) (rad_spike * 1000 / x), d_spike_x)   % rad/s


omega_inter = interp1(spike_x(1:end-1), omega, x)
plot(x, normalize(inv_y)); hold; scatter(x, normalize(inv_y))

%plot(x, omega_inter)

window_size = 100;
running_avg = movmean(omega_inter, window_size);
plot(x, running_avg)
xlabel('Time');
ylabel('Signal / rad*s-1');
legend('Original', 'o','Running Average');
title('Time Series with Running Average');

%% END
