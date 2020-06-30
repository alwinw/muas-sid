function log_data = filter_log_data(log_data, filter_order, filter_cutoff_freq)

% FIR filter order
N = min([filter_order, length(log_data.imu.time)-1]);
% FIR filter cutoff frequency
f0 = filter_cutoff_freq;

% apply the filter to every 
fnames_1 = fieldnames(log_data);
for i=1:length(fnames_1)
    sampling_rate = 1 / mean(diff(log_data.(fnames_1{i}).time));
    norm_freq = f0 / (sampling_rate / 2);
    if norm_freq > 1
        % if the sampling rate was too low for the given cutoff frequency,
        % then just skip this one (otherwise fir1() rightfully throws an
        % error)
        continue
    end
    b = fir1(N, norm_freq);
    
    fnames_2 = fieldnames(log_data.(fnames_1{i}));
    for j=1:length(fnames_2) 
        if strcmp(fnames_2{j}, 'time')
            % don't filter the time data
            continue
        end
        log_data.(fnames_1{i}).(fnames_2{j}) = filter(b, 1, log_data.(fnames_1{i}).(fnames_2{j}));
    end
end

end