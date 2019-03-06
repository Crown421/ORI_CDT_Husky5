%% adapted for Wicked Team Name


% Example glue-logic file from which you call your implementation of:
%  (1) Pole Detector
%  (2) Target Detector
%  (3) Planner
%  (4) Controller
%  -------------------
%  (5) SLAM [Note: full implementation is provided]

% Add MRG helper functions
% addpath('mrg'); % COMMENTED OUT

husky_id = 5; % Modify for your Husky

% Get the channel names and sensor IDs for this Husky
config = GetHuskyConfig(husky_id);

% Initialise mex-moos and register channels
% clear mexmoos % not needed b/c matlab will check if it has already been
                % initalised UNLESS you changed one of the five lines below
client = ['ExampleCdtClient' num2str(int32(rand*1e7))];
mexmoos('init', 'SERVERHOST', config.host, 'MOOSNAME', client);
mexmoos('REGISTER', config.laser_channel, 0.0);
mexmoos('REGISTER', config.stereo_channel, 0.0);
mexmoos('REGISTER', config.wheel_odometry_channel, 0.0);
pause(3); % Give mexmoos a chance to connect (important!)

% n = 100000;
% scans = cell(1, n);
% st_images = cell(1,n);
% wheel_od = cell(1,n);
i = 0;
record = 1;

state = zeros(3, 1);
P = diag([0.1, 0.1, 0.1]);
bufferLength = 20;
% Main loop
while true % <n
    % make a counter
    i = i+1;
    % Fetch latest messages from mex-moos
    mailbox = mexmoos('FETCH');
    scan = GetLaserScans(mailbox, config.laser_channel, true);
    stereo_images = GetStereoImages(mailbox, config.stereo_channel, true);
    wheel_odometry = GetWheelOdometry(mailbox, ...
                                      config.wheel_odometry_channel, ...
                                      true);
    if record                           
        
        scans{i} = scan;
        st_images{i} = stereo_images;
        wheel_od{i} = wheel_odometry;
        disp(wheel_odometry)
    end
    %%%%%%%%%%%%%% Do processing here %%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Display laser scan
    subplot(1, 3, 1);
    ShowLaserScan(scan);
    
    % Display stereo image
    subplot(1, 3, 2);
    ShowStereoImage(stereo_images)
    
    % Display undistorted stereo image
    subplot(1, 3, 3);
    ShowStereoImage(UndistortStereoImage(stereo_images, ...
                                         config.camera_model));
   
    % collect sensors
    i_modded = mod(i, bufferLength-1)+1;
    wheel_str(i_modded) = wheel_odometry;
    
    if i <= 1
        wheel_str = repmat(wheel_odometry, 1, bufferLength);
    end
    % time stamp of the scan 
    scan_time_stamp = scan.timestamp;
    % estiamtes the change in pose as input velocities using wheel odometry
    u_odom = u_estimat_odom(wheel_str, scan_time_stamp);

    if mod(i, 10)==0
        stereo_images = GetStereoImages(mailbox, config.stereo_channel, true);
        % targetLocation = FindTarget(stereo_images);
    end
    
    % lidar processing
    range_bearing_Poles = get_Poles_from_SCAN(scan);
    
    % slam
%     observedPoles = SLAMDataAssociations(x, candidatePoles);
%     [estState, estP] = SLAMMeasurement(observedPoles, state, P);
    [state, P] = SLAMUpdate(u_odom, range_bearing_Poles, state, P);
    
    state_cell{i} = state;
    P_cell{i} = P;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    pause(0.1); % don't overload moos w/commands
    
end

%%
% save('Test_measurements', 'scans', 'wheel_od', 'st_images')

