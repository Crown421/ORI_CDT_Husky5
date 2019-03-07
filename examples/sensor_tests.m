%% adapted for Wicked Team Name


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
% Main loop
while true % <n
    % Fetch latest messages from mex-moos
    mailbox = mexmoos('FETCH');
    scan = GetLaserScans(mailbox, config.laser_channel, true);
    stereo_images = GetStereoImages(mailbox, config.stereo_channel, true);
    wheel_odometry = GetWheelOdometry(mailbox, ...
                                      config.wheel_odometry_channel, ...
                                      true);
    if record                           
        i = i+1;
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
    
    pause(0.1); % don't overload moos w/commands
end

%%
% save('Test_measurements', 'scans', 'wheel_od', 'st_images')

