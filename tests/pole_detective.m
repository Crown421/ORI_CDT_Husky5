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

% Main loop
while true
    % Fetch latest messages from mex-moos
    mailbox = mexmoos('FETCH');
    scan = GetLaserScans(mailbox, config.laser_channel, true);
    stereo_images = GetStereoImages(mailbox, config.stereo_channel, true);
    wheel_odometry = GetWheelOdometry(mailbox, ...
                                      config.wheel_odometry_channel, ...
                                      true);

    %%%%%%%%%%%%%% Do processing here %%%%%%%%%%%%%%%
    
    if ~isempty(scan)
        angles = (10:0.5:170)';
        xs = scan.ranges(111:431).*sind(angles);
        ys = scan.ranges(111:431).*cosd(angles);
        distances = sqrt((xs(1:321-1) - xs(2:321)).^2 + (ys(1:321-1) - ys(2:321)).^2);
        objects_delims = [0;find(distances > 0.8);320]+1;
        num_delims = numel(objects_delims);
        object_ranges = [objects_delims(1:num_delims-1)'; objects_delims(2:num_delims)'-1]';
        object_ranges_midpoints = round((object_ranges(:,1) + object_ranges(:,2))/2);
        object_reflectances = ones(num_delims-1, 1);
        for i = 1:(num_delims-1)
            object_reflectances(i) = mean(scan.reflectances((110+object_ranges(i,1)):(110+object_ranges(i,2))));
        end
        pole_indices = find(object_reflectances > 600);
        poles = ones(2, numel(pole_indices));
        for j = 1:numel(pole_indices)
           midpoint = object_ranges_midpoints(pole_indices(j));
           poles(1,j) = xs(midpoint);
           poles(2,j) = ys(midpoint);
        end
        figure(1); clf;
        scatter(xs,ys, 10, 'blue')
        xlim([-2 8])
        ylim([-8 8])
        axis equal
        hold on
        scatter(poles(1,:), poles(2,:), 20, 'red')
    end
    
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %figure(2);
    %if ~isempty(stereo_images)
    %    undistorted = UndistortStereoImage(stereo_images, config.camera_model);
    %    ellipseParams = findEllipse(undistorted.left.rgb);
    %    imshow(undistorted.left.rgb);
    %    drawEllipse(ellipseParams);

    %    [true_dist, bear1, bear2] = targetDetector(stereo_images)
    %end
    pause(0.1); % don't overload moos w/commands
end