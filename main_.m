% central script

%% setup and init
setup

global config  %#ok
%%
seq = 0;
state = zeros(1,3); % initial position
P = zeros(3,3);
estMotion = zeros(1,3);
while notAtTarget
    seq = seq + 1;
    % collect sensors
    scan = GetLaserScans(mailbox, config.laser_channel, true);
    wheel_odometry = GetWheelOdometry(mailbox, config.wheel_odometry_channel, true);
    
    if mod(seq, 10)==0
        stereo_images = GetStereoImages(mailbox, config.stereo_channel, true);
        targetLocation = FindTarget(stereo_images);
    end
    
    % lidar processing
    candidatePoles = FindPoles(scan);
    
    % slam
    observedPoles = SLAMDataAssociations(x, candidatePoles);
    [estState, estP] = SLAMMeasurement(observedPoles, state, P);
    [state, P] = SLAMUpdate(estMotion, observedPoles, estState, estP);
    
    % planner
    goal = nextGoal(state, targetLocation, mode);
    
    
    % other things
    SendSpeedCommand()
end

%% shutdown
