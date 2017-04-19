function GNSS_measurements = Generate_GNSS_measurements(True_motion,...
    GNSS_config)
%Generate_GNSS_measurements - Generates  positions and velocities to the datesets. 
%
% This function created 27/3/2017 by Ren
%
% Inputs:
%  True_motion
%    .vel                                        velocity of true motion
%    .pos                                       position of true motion
%   no_epochs                            number of epochs
%   GNSS_config
%       .noise_root_PSD               velocity and position GNSS measurements noise root PSD
%       .epoch_interval                  GNSS measurements epoch interval
%
% Outputs:
%   GNSS_measurements     GNSS measurement data:
%     Line 1-3                     velocity (m/s)
%     Line 4-6                     position (m)
% Begins
 no_epochs = size(True_motion.avp,1);
 GNSS_measurements = zeros(6,no_epochs);
% Loop 
for j = 1:no_epochs
    
        % Calculate velocity measurement
        GNSS_measurements(1:3,j) = True_motion.avp(j,4:6)' + GNSS_config.noise_root_PSD(1:3,1:3)*randn(3,1);
    
        % Calculate position measurement
        GNSS_measurements(4:6,j) = True_motion.avp(j,7:9)' + GNSS_config.noise_root_PSD(4:6,4:6)*randn(3,1);
        
end % for j
NUM = GNSS_config.epoch_interval/True_motion.ts;
GNSS_measurements = GNSS_measurements(:,1:NUM:end);
% Ends