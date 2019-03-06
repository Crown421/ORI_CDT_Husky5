% write a thing robot integration, while loop with long pause, 
% grabbing stuff from sensors, 
% run slam
% run planner
% display poles, tree (faint), and path, and robot as point
% display robot and poles, maybe with confidence circle?, lidar bits
% display images,

%% setup and init
setup

global config  %#ok

%%
seq = 0;
state = zeros(3, 1); % initial position
P = zeros(3,3);
% estMotion = zeros(1,3);

bufferLength = 10;
wheel_odom_buffer = repmat(struct('source_timestamp', 0,'destination_timestamp',0, 'x', 0, 'y', 0, 'yaw', 0), 10, 1);


while true
    seq = seq + 1;
    % collect sensors
    scan = GetLaserScans(mailbox, config.laser_channel, true);
    wheel_odometry = GetWheelOdometry(mailbox, config.wheel_odometry_channel, true);
    
    i = mod(seq, bufferLength);
    wheel_odom_buffer(i) = wheel_odometry;
    % time stamp of the scan 
    scan_time_stamp = scan.timestamp;
    % estimates the change in pose as input velocities using wheel odometry
    u_odom = u_estimat_odom(wheel_odom_buffer, scan_time_stamp);
%store
    if mod(seq, 10)==0
        stereo_images = GetStereoImages(mailbox, config.stereo_channel, true);
        targetLocation = FindTarget(stereo_images);
    end
    
    range_bearing_Poles = FindPoles(scan);
    
    % slam
%     observedPoles = SLAMDataAssociations(x, candidatePoles);
%     [estState, estP] = SLAMMeasurement(observedPoles, state, P);
    [state, P] = SLAMUpdate(u_odom, range_bearing_Poles, state, P);
    
    
    
    
    pause(2);
    
    
    
    
end