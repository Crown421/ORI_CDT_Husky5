%% adapted for Wicked Team Name

setup


seq = 0;
record = 1;
scanSeq = 0;
odomSeq = 0;
camSeq = 0;

state = zeros(3, 1);
P = diag([0.1, 0.1, 0.1]);
bufferLength = 20;

% Main loop
while true % <n
    % make a counter
    seq = seq+1;
    % Fetch latest messages from mex-moos
    mailbox = mexmoos('FETCH');
    scan = GetLaserScans(mailbox, config.laser_channel, true);
    cam = GetStereoImages(mailbox, config.stereo_channel, true);
    odom = GetWheelOdometry(mailbox, ...
                                      config.wheel_odometry_channel, ...
                                      false);
                                  

                                  
    if ~isempty(scan)
        if record
            scans(scanSeq) = scan;
            scanSeq = scanSeq + 1;
        end 
    end
    
    if ~isempty(cam)
        if record
            cams(camSeq) = cam;
            camSeq = camSeq + 1;
        end 
    end
    
    modIodom = mod(odomSeq, bufferLength-1)+1;
    if ~isempty(odom)
        buffer_odom(modIodom) = odom;
        if record
            odoms(odomSeq) = odom;
            odomSeq = odomSeq + 1;
        end 
    end
   
    %%%%%%%%%%%%%% Do processing here %%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     % Display laser scan
%     subplot(1, 3, 1);
%     ShowLaserScan(scan);
%     
%     % Display stereo image
%     subplot(1, 3, 2);
%     ShowStereoImage(stereo_images)
%     
%     % Display undistorted stereo image
%     subplot(1, 3, 3);
%     ShowStereoImage(UndistortStereoImage(stereo_images, ...
%                                          config.camera_model));
   
    
    if ~isempty(odom)
        
        trafo = eye(3);
        for iOdom = length(odom):-1:1
            xyyaw = [odom(iOdom).x, odom(iOdom).y, odom(iOdom).yaw];
            se2 = BuildSE2Transform(xyyaw);
            trafo = se2*trafo;
        end
        u_odom = SE2ToComponents(trafo);
                        
    end

    
    range_bearing_Poles = get_Poles_from_SCAN(scan);

%     if mod(seq, 10)==0
%         cam = GetStereoImages(mailbox, config.stereo_channel, true);
%         % targetLocation = FindTarget(stereo_images);
%     end
    
    % slam
    [state, P] = SLAMUpdate(u_odom, range_bearing_Poles, state, P);
    
    state_cell{seq} = state;
    P_cell{seq} = P;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    pause(0.05); % don't overload moos w/commands
    
end

%%
% save('Test_measurements', 'scans', 'wheel_od', 'st_images')

