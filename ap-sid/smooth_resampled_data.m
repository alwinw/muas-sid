function resampled_data = smooth_resampled_data(resampled_data)

fnames = fieldnames(resampled_data);
for i=1:length(fnames)
    if strcmp(fnames{i}, 'time')
        % don't smooth the time data
        continue
    end
    
    resampled_data.(fnames{i}) = smooth(resampled_data.time, ...
        resampled_data.(fnames{i}), 10, 'loess');
end

end