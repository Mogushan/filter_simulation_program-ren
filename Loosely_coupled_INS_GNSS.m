function [out_profile, out_errors, out_IMU_bias_est] = Loosely_coupled_INS_GNSS(True_motion, ...
    INS_data_orig,  GNSS_config, LC_KF_config)
% Loosely_coupled_INS_GNSS - Simulates inertial navigation using NED
% navigaiton equations and error model, GNSS and loosely coupled
% INS/GNSS integration.
%
% Created 17/03/17 by Ren
% Last edited 
%
% Inputs:
% True_motion
%
% INS_data     inertial navigation profile array
%
% IMU_data    imu data 
     
%
% GNSS_config
%   .noise_root_PSD          GNSS noise root PSD
%
% LC_KF_config
%    .init_att_unc                 Initial attitude uncertainty per axis
%    .init_vel_unc                 Initial velocity uncertainty per axis
%    .init_pos_unc                Initial position uncertainty per axis
%    .init_b_a_unc                Initial accelerometer bias uncertainty per instrument
%    .init_b_g_unc                Initial gyro bias uncertainty per instrument
%    .gyro_noise_PSD         Gyro noise PSD 
%    .accel_noise_PSD        Accelerometer noise PSD 
%    .accel_bias_PSD          Accelerometer bias random walk PSD
%    .gyro_bias_PSD           Gyro bias random walk PSD
%    .epoch_interval             Interval between KF epochs(s)
%
% Outputs:
%    out_profile            Navigation solution as a motion profile array
%    out_errors             Navigation solution error array
%    out_IMU_bias_est        Kalman filter IMU bias estimate array
%
% Format of motion profiles:
% Column 1: roll angle of body w.r.t ENU 
% Column 2: pitch angle of body w.r.t ENU
% Column 3: yaw angle of body w.r.t ENU
% Column 4: east velocity (m/s)
% Column 5: north velocity (m/s)
% Column 6: up velocity (m/s)
% Column 7: latitude (m)
% Column 8: longitude (m)
% Column 9: height (m)
% Column 10: time(s)
%
% Format of errors array:
% Column 1: roll component of ENU attitude error (deg)
% Column 2: pitch component of ENU attitude error (deg)
% Column 3: yaw component of ENU attitude error (deg)
% Column 4: east velocity error(m/s)
% Column 5: north velocity error(m/s)
% Column 6: up velocity error(m/s)
% Column 7: east position error (m)
% Column 8: north position error (m)
% Column 9: up position error (m)
% Column 10: time(s)
%
% Format of IMU biases array:
%  Column 1: estimated X gyro bias (rad/s)
%  Column 2: estimated Y gyro bias (rad/s)
%  Column 3: estimated Z gyro bias (rad/s)
%  Column 4: estimated X accelerometer bias (m/s^2)
%  Column 5: estimated Y accelerometer bias (m/s^2)
%  Column 6: estimated Z accelerometer bias (m/s^2)
%  Column 7: time (sec)

% Begins

% Ratio of filtering interval and sensor interval
NUM = LC_KF_config.epoch_interval/(INS_data_orig.avp(2,10) - INS_data_orig.avp(1,10));

% Process INS data& IMU data
INS_data = INS_data_orig.avp(1:NUM:end,:);
IMU_data = INS_data_orig.imu(1:NUM:end,:);
no_epochs = size(INS_data,1);

% Generate GNSS measurements
GNSS_measurements = Generate_GNSS_measurements(True_motion, GNSS_config);

% Initialize estimated attitude solution
out_profile = zeros(no_epochs, 10);
out_errors = zeros(no_epochs, 10);
out_IMU_bias_est = zeros(no_epochs,7);


% Initialize Kalman filter P matrix and IMU bias states
P_matrix = Initialize_LC_P_matrix(LC_KF_config);


% Initialize inputs
x_est = zeros(15,1);
x_est(10:15) = [INS_data_orig.imuerr.eb;INS_data_orig.imuerr.db];%17/04/11
out_profile(1,:) = INS_data(1,:);
est_IMU(1,:) = IMU_data(1,1:6);
time = 0;

% Main Loop
for epoch = 2 : no_epochs
    % Input data from ins profile&imu profile
%     att.e = out_profile(epoch-1,1);%17/03/30 feedback correct
%     att.n = out_profile(epoch-1,2);
%     att.u = out_profile(epoch-1,3);
%     vel.e = out_profile(epoch-1,4);
%     vel.n = out_profile(epoch-1,5);
%     vel.u = out_profile(epoch-1,6);
%     pos.e = out_profile(epoch-1,7);
%     pos.n = out_profile(epoch-1,8);
%     pos.u = out_profile(epoch-1,9);
%     accel.e = est_IMU(epoch-1,4);
%     accel.n = est_IMU(epoch-1,5);
%     accel.u = est_IMU(epoch-1,6);

    att.e = INS_data(epoch-1,1);%17/04/10 output correct
    att.n = INS_data(epoch-1,2);
    att.u = INS_data(epoch-1,3);
    vel.e = INS_data(epoch-1,4);
    vel.n = INS_data(epoch-1,5);
    vel.u = INS_data(epoch-1,6);
    pos.e = INS_data(epoch-1,7);
    pos.n = INS_data(epoch-1,8);
    pos.u = INS_data(epoch-1,9);
    accel.e = IMU_data(epoch-1,4);
    accel.n = IMU_data(epoch-1,5);
    accel.u = IMU_data(epoch-1,6);
    
    % Run Integration Kalman Filter
    [x_est, P_matrix] = LC_KF_PROCESS(att, vel, pos, accel, x_est,...
         GNSS_measurements(:,epoch), INS_data(epoch,:), GNSS_config, P_matrix, ...
        LC_KF_config);
    
    time = time + LC_KF_config.epoch_interval;
   
    %Generate output errors profile
    out_errors(epoch,1:9) = x_est(1:9)';
    out_errors(epoch,10) = time;
    
    % Generate output IMU bias profile
    out_IMU_bias_est(epoch,1:6) = x_est(10:15)';
    out_IMU_bias_est(epoch,7) = time;
    
    % Generate output profile
    out_profile(epoch,1:9) = INS_data(epoch,1:9) - out_errors(epoch,1:9);
    out_profile(epoch,10) = time;
    
    %Generate estimate IMU profile
    est_IMU(epoch,:) = IMU_data(epoch,1:6) + out_IMU_bias_est(epoch,1:6);
     
    
end