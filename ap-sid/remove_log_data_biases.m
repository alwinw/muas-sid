function log_data = remove_log_data_biases(log_data)
% finds the first sequence of data that is relatively constant, then use
% the average as a bias to be subtracted

const_time = 5.0; 
const_frac_threshold = 0.90;
tol = 1e-2;

fields = {'gyro_x', 'gyro_y', 'gyro_z', 'acc_x', 'acc_y', 'acc_z'};

fprintf('<strong>%4s  %10s  %9s</strong>\n', 'Msg',  'Field',  'Bias');

fnames_1 = fieldnames(log_data);
for i=1:length(fnames_1)
    fnames_2 = fieldnames(log_data.(fnames_1{i}));
    for j=1:length(fnames_2)
        if ~any(strcmp(fnames_2{j}, fields))
            continue;
        end
        
        data = log_data.(fnames_1{i}).(fnames_2{j});
        abs_diff_2 = abs(diff(data, 2));
        const_diff_2 = abs_diff_2 < tol;
        % find the first sequence of 2nd order differences that are
        % constant for const_time seconds
        num_const_indices = ceil(const_time / mean(diff(log_data.(fnames_1{i}).time)));
        for k=1:length(const_diff_2)-num_const_indices
            if sum(const_diff_2(k:k+num_const_indices)) >= const_frac_threshold * num_const_indices
                % this is a constant sequence
                % take its average to use as the offset, then subtract it
                offset = mean(data(k:k+num_const_indices));
                log_data.(fnames_1{i}).(fnames_2{j}) = log_data.(fnames_1{i}).(fnames_2{j}) - offset;
                fprintf('%4s  %10s  %9.2e\n', fnames_1{i}, fnames_2{j}, offset);
                break;
            end
        end
    end
end

% add gravity back in (technically acceleration due to the reaction force)
log_data.imu.acc_z = log_data.imu.acc_z - 9.80665;

end