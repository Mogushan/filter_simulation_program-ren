function P_matrix = Initialize_LC_P_matrix(LC_KF_config)
%Initialize_LC_P_matrix - Initializes the loosely coupled INS/GNSS KF
%error covariance matrix
%
% This function created 27/3/2017 by Ren
%
% Inputs:
%   LC_KF_config
%     .init_att_unc           Initial attitude uncertainty per axis (rad)
%     .init_vel_unc           Initial velocity uncertainty per axis (m/s)
%     .init_pos_unc           Initial position uncertainty per axis (m)
%     .init_b_g_unc           Initial gyro. bias uncertainty (rad/s)
%     .init_b_a_unc           Initial accel. bias uncertainty (m/s^2)
% Outputs:
%   P_matrix              state estimation error covariance matrix



% Begins

% Initialize error covariance matrix
P_matrix =  zeros(15);
P_matrix(1:3,1:3) = LC_KF_config.init_att_unc.^2 * ones(1,3);
P_matrix(4:6,4:6) =  LC_KF_config.init_vel_unc.^2 * ones(1,3);
P_matrix(7:9,7:9) = LC_KF_config.init_pos_unc.^2 * ones(1,3);
P_matrix(10:12,10:12) = LC_KF_config.init_b_g_unc.^2 * ones(1,3);
P_matrix(13:15,13:15) = LC_KF_config.init_b_a_unc.^2 * ones(1,3);


% Ends