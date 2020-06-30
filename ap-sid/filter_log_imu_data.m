function log_data = filter_log_imu_data(log_data, filter_order, filter_cutoff_freq)

fir_filter_order = min([filter_order, length(log_data.imu.time)-1]);
fir_filter_cutoff_freq = filter_cutoff_freq;
sampling_rate = 1 / mean(diff(log_data.imu.time));
b = fir1(fir_filter_order, fir_filter_cutoff_freq / (sampling_rate / 2));

log_data.imu.gyro_x = filter(b, 1, log_data.imu.gyro_x);
log_data.imu.gyro_y = filter(b, 1, log_data.imu.gyro_y);
log_data.imu.gyro_z = filter(b, 1, log_data.imu.gyro_z);
log_data.imu.acc_x = filter(b, 1, log_data.imu.acc_x);
log_data.imu.acc_y = filter(b, 1, log_data.imu.acc_y);
log_data.imu.acc_z = filter(b, 1, log_data.imu.acc_z);

end