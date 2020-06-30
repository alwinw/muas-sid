classdef DataStore
properties (Access = public) 
    log  % Raw imported log data
    pd   % Processed data (resampled, filtered)
    spd  % Subdomain of processed data (reduced time domain for speed)
end

properties (Access = private)
    have_smoothed = false;
end

methods (Access = public)
    function obj = DataStore(log_prefix, varargin)
        % Create a Data Store instance
        % ds = DataStore(log_prefix)
        %
        % This may take some time for large logs
        %
        % log_prefix : The naming scheme used for CSV log files
        %   e.g. support_AOA.csv
        %   NB: Expects IMU, CTUN, NKF1, NKF2, AOA, RCOU, BAT
        %
        % Optional Arguments:
        % 'Directory' : The directory in which the log CSVs can be found.
        %   e.g. ds = DataStore('support', 'Directory', 'logs/');
        
        assert(ischar(log_prefix), 'Log prefix must be a character array');
        
        dir = '/';
        
        % Optional arguments
        for i=1:2:length(varargin)
            if strcmp(varargin{i}, 'Directory')
                dir = varargin{i+1};
                assert(ischar(dir), 'Directory must be a character array');
            else
                error('Unknown argument ''%s''', varargin{i});
            end
        end
        
        % Ensure directory has a /
        if dir(end) ~= '/'
            dir = [dir, '/'];
        end
        
        % Read in raw log data
        obj = obj.import_log_data([dir, log_prefix]);
    end
    
    function obj = process_log_data(obj, resample_freq, filter_cutoff_freq)
        % ds = ds.process_log_data(resample_freq, filter_cutoff_freq)
        %
        % Fills the class field 'pd' for the processed data
        %
        % resample_freq      : Log data is resampled to this common 
        %                      frequency [Hz]
        % filter_cutoff_freq : Frequency [Hz] to use for the lowpass filter
        
        t0_max = -Inf;
        t1_min = Inf;
        
        %%% Resample and filter the data
        fnames_1 = fieldnames(obj.log); % first level of struct field names
        for i=1:length(fnames_1)
            fnames_2 = fieldnames(obj.log.(fnames_1{i})); % second level of struct field names
            for j=1:length(fnames_2) 
                if strcmp(fnames_2{j}, 'time')
                    % update the time domain min/max
                    t0_max = max(t0_max, obj.log.(fnames_1{i}).time(1));
                    t1_min = min(t1_min, obj.log.(fnames_1{i}).time(end));
                    % don't filter the time data
                    continue
                end
                
                % frequencies
                Fs = resample_freq;
                Fcutoff = filter_cutoff_freq;

                % resample to a common frequency
                [pd.(fnames_1{i}).(fnames_2{j}), T] = ...
                    resample(obj.log.(fnames_1{i}).(fnames_2{j}), ...
                        obj.log.(fnames_1{i}).time, Fs);

                % lowpass filtering
                pd.(fnames_1{i}).(fnames_2{j}) = lowpass(pd.(fnames_1{i}).(fnames_2{j}), ...
                                                       Fcutoff, ...
                                                       Fs);
            end
            % use the resampled times
            pd.(fnames_1{i}).time = T;
        end
    
    %%% Constrict the time domain
    % Data constrained to the maximum initial time, and minimum final time
    % Times shifted to start at 0
    fnames_1 = fieldnames(pd);
    for i=1:length(fnames_1)
        fnames_2 = fieldnames(pd.(fnames_1{i}));
        idx = pd.(fnames_1{i}).time > t0_max & pd.(fnames_1{i}).time < t1_min;
        for j=1:length(fnames_2)
            pd.(fnames_1{i}).(fnames_2{j}) = subsref(pd.(fnames_1{i}).(fnames_2{j}), ...
                struct('type','()','subs',{{idx}}));
            
            if strcmp(fnames_2{j}, 'time')
                % shift to start at 0
                pd.(fnames_1{i}).time = pd.(fnames_1{i}).time - ...
                    pd.(fnames_1{i}).time(1);
            end
        end
    end
    
    %%% Pack the resulting data into more useful struct fields
    obj.pd.time = pd.imu.time;
    % IMU
    obj.pd.angular_velocity_x = pd.imu.gyro_x;
    obj.pd.angular_velocity_y = pd.imu.gyro_y;
    obj.pd.angular_velocity_z = pd.imu.gyro_z;
    obj.pd.acceleration_x = pd.imu.acc_x;
    obj.pd.acceleration_y = pd.imu.acc_y;
    obj.pd.acceleration_z = pd.imu.acc_z;
    % CTUN
    obj.pd.airspeed = pd.ctun.airspeed;
    % NKF1
    obj.pd.roll = pd.nkf1.roll;
    obj.pd.pitch = pd.nkf1.pitch;
    obj.pd.yaw = pd.nkf1.yaw;
    obj.pd.velocity_north = pd.nkf1.velocity_north;
    obj.pd.velocity_east = pd.nkf1.velocity_east;
    obj.pd.velocity_down = pd.nkf1.velocity_down;
    obj.pd.position_north = pd.nkf1.position_north;
    obj.pd.position_east = pd.nkf1.position_east;
    obj.pd.position_down = pd.nkf1.position_down;
    % NKF2
    obj.pd.wind_velocity_north = pd.nkf2.wind_velocity_north;
    obj.pd.wind_velocity_east = pd.nkf2.wind_velocity_east;
    % AOA
    obj.pd.angle_of_attack = pd.aoa.angle_of_attack;
    obj.pd.sideslip = pd.aoa.sideslip;
    % RCOU
    obj.pd.throttle = pd.rcou.throttle;
    obj.pd.aileron = pd.rcou.aileron;
    obj.pd.elevator = pd.rcou.elevator;
    obj.pd.rudder = pd.rcou.rudder;
    % BAT
    obj.pd.battery_current = pd.bat.current;
    obj.pd.battery_power = pd.bat.power;
    
    %%% Compute derived quantities
    % Body velocities
    % NB: Computed using wind-axis quantities as these account for wind
    % through the EKF
    [u,v,w] = obj.compute_velocity_wind_axes(obj.pd.airspeed, ...
        obj.pd.angle_of_attack, obj.pd.sideslip);
    obj.pd.velocity_x = u;
    obj.pd.velocity_y = v;
    obj.pd.velocity_z = w;
    % Altitude from down position
    obj.pd.height = -obj.pd.position_down;
    end
    
    function obj = generate_processed_data_subdomain(obj, t0, t1)
        % Stores a subset of the processed data in the class 'spd' field
        %
        % t0 : Start time
        % t1 : End time
        
        obj.spd = obj.get_processed_data_subdomain(t0, t1);
    end
    
    function spd = get_processed_data_subdomain(obj, t0, t1)
        % Returns a subset of the processed data
        %
        % t0 : Start time
        % t1 : End time
        
        assert(t0 < t1, 't0 < t1 required');
        
        idx = obj.pd.time >= t0 & obj.pd.time <= t1;
        
        fnames_1 = fieldnames(obj.pd);
        for i=1:length(fnames_1)
            spd.(fnames_1{i}) = subsref(obj.pd.(fnames_1{i}), ...
                struct('type','()','subs',{{idx}}));
        end
    end
    
    function obj = smooth_processed_data(obj)
        % Smooths the 'spd' field of the instance
        
        fnames = fieldnames(obj.spd);
        for i=1:length(fnames)
            if strcmp(fnames{i}, 'time')
                % don't try to smooth time data
                continue;
            end
            
            obj.spd.(fnames{i}) = smooth(obj.spd.time, obj.spd.(fnames{i}), 10, 'loess');
        end
        
        obj.have_smoothed = true;
    end
    
    function obj = compute_numerical_derivatives(obj)
        % Computes numerical derivates of p,q,r,alpha,beta and adds them to
        % the 'spd' field
        %
        % This should be called *after* smoothing
        
        if ~obj.have_smoothed
            fprintf('WARNING: Numerical derivatives should be computed after smoothing\n');
        end
        
        N = 10;
        obj.spd.angular_acceleration_x = obj.numerical_derivative(obj.spd.time, ...
            obj.spd.angular_velocity_x, N);
        obj.spd.angular_acceleration_y = obj.numerical_derivative(obj.spd.time, ...
            obj.spd.angular_velocity_y, N);
        obj.spd.angular_acceleration_z = obj.numerical_derivative(obj.spd.time, ...
            obj.spd.angular_velocity_z, N);
        obj.spd.angle_of_attack_rate = obj.numerical_derivative(obj.spd.time, ...
            obj.spd.angle_of_attack, N);
        obj.spd.sideslip_rate = obj.numerical_derivative(obj.spd.time, ...
            obj.spd.sideslip, N);
    end
    
    function obj = correct_accel_data_for_sensor_position(obj, x, y, z)
        % Updates the 'spd' field to correct for sensor position
        % effects on accelerometer measurements
        % ds = ds.correct_accel_data_for_sensor_position(x,y,z)
        %
        % x,y,z are in body axes with the origin at the aircraft's CG
        
        pos = [x; y; z];
        
        for i=1:length(obj.spd.time)
            acc = [obj.spd.acceleration_x(i); 
                   obj.spd.acceleration_y(i);
                   obj.spd.acceleration_z(i)];

            p = obj.spd.angular_velocity_x(i);
            q = obj.spd.angular_velocity_y(i);
            r = obj.spd.angular_velocity_z(i);
            pdot = obj.spd.angular_acceleration_x(i);
            qdot = obj.spd.angular_acceleration_y(i);
            rdot = obj.spd.angular_acceleration_z(i);
            A = [q.^2+r.^2 -(p.*q-rdot) -(p.*r+qdot);
                 -(p.*q+rdot) (p.^2+r.^2) -(q.*r-pdot);
                 -(p.*r-qdot) -(q.*r+pdot) (p.^2+q.^2)];
            acc = acc + A*pos;

            obj.spd.acceleration_x(i) = acc(1);
            obj.spd.acceleration_y(i) = acc(2);
            obj.spd.acceleration_z(i) = acc(3);
        end
    end
end

methods (Access = private)
    function obj = import_log_data(obj, file_base)
        % file_base is of the form "logs/support"
        
        %%% IMU
        imu_csv_file = [file_base, '_IMU.csv'];
        assert(exist(imu_csv_file, 'file')>0, 'IMU file not found');
        d = importdata(imu_csv_file, ',', 1);
        obj.log.imu.time = d.data(:,1);
        obj.log.imu.gyro_x = d.data(:,3);
        obj.log.imu.gyro_y = d.data(:,4);
        obj.log.imu.gyro_z = d.data(:,5);
        obj.log.imu.acc_x = d.data(:,6);
        obj.log.imu.acc_y = d.data(:,7);
        obj.log.imu.acc_z = d.data(:,8);

        %%% CTUN
        ctun_csv_file = [file_base, '_CTUN.csv'];
        assert(exist(ctun_csv_file, 'file')>0, 'CTUN file not found');
        d = importdata(ctun_csv_file, ',', 1);
        obj.log.ctun.time = d.data(:,1);
        obj.log.ctun.airspeed = d.data(:,10);

        %%% NKF1
        nkf1_csv_file = [file_base, '_NKF1.csv'];
        assert(exist(nkf1_csv_file, 'file')>0, 'NKF1 file not found');
        d = importdata(nkf1_csv_file, ',', 1);
        obj.log.nkf1.time = d.data(:,1);
        obj.log.nkf1.roll = d.data(:,3) * pi/180;
        obj.log.nkf1.pitch = d.data(:,4) * pi/180;
        obj.log.nkf1.yaw = d.data(:,5) * pi/180;
        obj.log.nkf1.velocity_north = d.data(:,6);
        obj.log.nkf1.velocity_east = d.data(:,7);
        obj.log.nkf1.velocity_down = d.data(:,8);
        obj.log.nkf1.position_north = d.data(:,10);
        obj.log.nkf1.position_east = d.data(:,11);
        obj.log.nkf1.position_down = d.data(:,12);
        obj.log.nkf1.gyro_bias_x = d.data(:,13) * pi/180;
        obj.log.nkf1.gyro_bias_y = d.data(:,14) * pi/180;
        obj.log.nkf1.gyro_bias_z = d.data(:,15) * pi/180;

        %%% NKF2
        nkf2_csv_file = [file_base, '_NKF2.csv'];
        assert(exist(nkf2_csv_file, 'file')>0, 'NKF2 file not found');
        d = importdata(nkf2_csv_file, ',', 1);
        obj.log.nkf2.time = d.data(:,1);
        obj.log.nkf2.acc_bias_z = d.data(:,3);
        obj.log.nkf2.wind_velocity_north = d.data(:,7);
        obj.log.nkf2.wind_velocity_east = d.data(:,8);

        %%% AOA
        aoa_csv_file = [file_base, '_AOA.csv'];
        assert(exist(aoa_csv_file, 'file')>0, 'AOA file not found');
        d = importdata(aoa_csv_file, ',', 1);
        obj.log.aoa.time = d.data(:,1);
        obj.log.aoa.angle_of_attack = d.data(:,3) * pi/180;
        obj.log.aoa.sideslip = d.data(:,4) * pi/180;

        %%% RCOU
        rcou_csv_file = [file_base, '_RCOU.csv'];
        assert(exist(rcou_csv_file, 'file')>0, 'RCOU file not found');
        d = importdata(rcou_csv_file, ',', 1);
        obj.log.rcou.time = d.data(:,1);
        obj.log.rcou.aileron = (d.data(:,3) - 1500) / 1000; % [-1,1]
        obj.log.rcou.elevator = (d.data(:,4) - 1500) / 1000; % [-1,1]
        obj.log.rcou.throttle = (d.data(:,5) - 1000) / 1000; % [0,1]
        obj.log.rcou.rudder = (d.data(:,6) - 1500) / 1000; % [-1,1]

        %%% BAT
        bat_csv_file = [file_base, '_BAT.csv'];
        assert(exist(bat_csv_file, 'file')>0, 'BAT file not found');
        d = importdata(bat_csv_file, ',', 1);
        obj.log.bat.time = d.data(:,1);
        obj.log.bat.current = d.data(:,5);
        obj.log.bat.power = d.data(:,3) .* d.data(:,5);
    end
    
    function [u,v,w] = compute_velocity_body_axes(obj, vN, vE, vD, phi, theta, psi)
        
        u = zeros(length(phi), 1);
        v = zeros(length(phi), 1);
        w = zeros(length(phi), 1);

        for i=1:length(phi)
            sin_phi = sin(-phi(i));
            cos_phi = cos(-phi(i));
            sin_theta = sin(-theta(i));
            cos_theta = cos(-theta(i));
            sin_psi = sin(-psi(i));
            cos_psi = cos(-psi(i));

            R1 = [1, 0, 0;
                  0, cos_phi, sin_phi;
                  0, -sin_phi, cos_phi]';
            R2 = [cos_theta, 0, -sin_theta;
                  0, 1, 0;
                  sin_theta, 0, cos_theta]';
            R3 = [cos_psi, sin_psi, 0;
                  -sin_psi, cos_psi, 0;
                  0, 0, 1]';

            y = R3*R2*R1*[vN(i); vE(i); vD(i)];
            u(i) = y(1);
            v(i) = y(2);
            w(i) = y(3);
        end
    end
    
    function [u,v,w] = compute_velocity_wind_axes(obj, V, alpha, beta)
        u = V .* cos(alpha) .* cos(beta);
        v = V .* sin(beta);
        w = V .* sin(alpha) .* cos(beta);
    end
    
    function df_dt = numerical_derivative(obj, t, f, N)
        dT = (t(2)-t(1)) / N;
        T = (t(1) : dT : t(end) + 2*dT)';
        f_interp = interp1(t, f, T, 'spline');
        df_dt = diff(f_interp, 1) / dT;
        df_dt = df_dt(1:N:end);
    end
end
end