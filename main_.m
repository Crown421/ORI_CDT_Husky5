% central script

%% setup and init
setup

global config  %#ok
%%
seq = 0;
state = zeros(3, 1); % initial position
P = zeros(3,3);
% estMotion = zeros(1,3);

bufferLength = 10;

while notAtTarget
    seq = seq + 1;
    % collect sensors
    scan = GetLaserScans(mailbox, config.laser_channel, true);
    wheel_odometry = GetWheelOdometry(mailbox, config.wheel_odometry_channel, true);
    
    i = mod(seq, bufferLength);
    wheel_str(i) = wheel_odometry;
    % time stamp of the scan 
    scan_time_stamp = scan.timestamp;
    % estiamtes the change in pose as input velocities using wheel odometry
    u_odom = u_estimat_odom(wheel_str, scan_time_stamp);

    if mod(seq, 10)==0
        stereo_images = GetStereoImages(mailbox, config.stereo_channel, true);
        targetLocation = FindTarget(stereo_images);
    end
    
    % lidar processing
    range_bearing_Poles = FindPoles(scan);
    
    % slam
%     observedPoles = SLAMDataAssociations(x, candidatePoles);
%     [estState, estP] = SLAMMeasurement(observedPoles, state, P);
    [state, P] = SLAMUpdate(u_odom, range_bearing_Poles, state, P);
    
    % planner
    goal = nextGoal(state, targetLocation, mode);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % controller added
    max_gain_intercpt = 0.8;
    max_gain_forward = 0.8;
    % p contrller
    uctrl = pid_ctrl(goal, state(1:3, 1), max_gain_intercpt, max_gain_forward);
    % other things
    SendSpeedCommand(uctrl(1, 1), uctrl(2, 1), husky_config.control_channel)
    
end

%% shutdown
