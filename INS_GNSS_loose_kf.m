% INS_GNSS
% Loosely coupled INS/GNSS
%  imu_data.mat
%  Aviation-grade IMU
%
% 15/03/17 by Ren

% Constants
R = 6378245;

% Configuration
% Input truth motion profile 
load trj10ms;
% Input imu data & pure inertial profile
load imu_data;


% Accelerometer biases 
IMU_errors.b_a = SINS.imuerr.db;
% Gyro biases 
IMU_errors.b_g = SINS.imuerr.eb;

% Accelerometer noise root PSD
IMU_errors.accel_noise_root_PSD = SINS.imuerr.wdb*ones(1,3);
% Gyro noise root PSD 
IMU_errors.gyro_noise_root_PSD = SINS.imuerr.web*ones(1,3);


% GNSS noise root PSD
GNSS_config.noise_root_PSD = SINS.davp0(4:9)*ones(1,6);
% Interval between GNSS epochs(s)
GNSS_config.epoch_interval = 1;



% Initial attitude uncertainty per axis(deg)
LC_KF_config.init_att_unc = 10*SINS.davp0(1:3);
% Initial velocity uncertainty per axis(m/s)
LC_KF_config.init_vel_unc = 10*SINS.davp0(4:6);
% Initial position uncertainty per axis(m)
LC_KF_config.init_pos_unc = 10*SINS.davp0(7:9);
% Initial accelerometer bias uncertainty per instrument
LC_KF_config.init_b_a_unc = 10*SINS.imuerr.db;
% Initial gyro bias uncertainty per instrument
LC_KF_config.init_b_g_unc = 10*SINS.imuerr.eb;

% Gyro noise PSD               
LC_KF_config.gyro_noise_PSD = 1e3*SINS.imuerr.web.^2*ones(1,3);
% Accelerometer noise PSD               
LC_KF_config.accel_noise_PSD = 1e3*SINS.imuerr.wdb.^2*ones(1,3);
% Accelerometer bias random walk PSD
LC_KF_config.accel_bias_PSD = SINS.imuerr.db.^2*ones(1,3);
% Gyro bias random walk PSD
LC_KF_config.gyro_bias_PSD = SINS.imuerr.eb.^2*ones(1,3);

% Interval between KF epochs(s)
LC_KF_config.epoch_interval = 1;

% Loosely coupled NED Inertial navigation and GNSS integrated navigation
% simulation
[out_profile, out_errors, out_IMU_bias_est] = Loosely_coupled_INS_GNSS(trj,...
    SINS, GNSS_config, LC_KF_config);


% Plot errors profile
close all;
Plot_profile(out_profile);
Plot_errors(out_errors);
