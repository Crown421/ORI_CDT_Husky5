%% adapted for Wicked Team Name

setup


seq = 0;
record = 1;
scanSeq = 0;
odomSeq = 0;
camSeq = 0;

state = zeros(3, 1);
P = diag([0.1, 0.1, 1.0]);
bufferLength = 20;
target_pose = [3; -1; 0];
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
            scanSeq = scanSeq + 1;
            scans(scanSeq) = scan;
        end 
    end
    
    if ~isempty(cam)
        if record
            camSeq = camSeq + 1;
            cams(camSeq) = cam;
        end 
    end
    
    if ~isempty(odom)
        if record
            odomSeq = odomSeq + 1;
            odoms{odomSeq} = odom;
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

    if ~isempty(cam)
        [true_dist, bearing] = targetDetector(cam);
        [b_target, a_target] = pol2cart(bearing, true_dist);
        
        if ~isnan(true_dist)
        
            target_pose = state(1:2) + robot_coords_world(state(3))*[-a_target; b_target];
            target_pose = [target_pose; 0];
        end
        
    end
    
    % slam build map first
    [state, P] = SLAMUpdate(u_odom', range_bearing_Poles, state, P);
    
     % put the route planner here:
    if false
        
        robotloc = state(1:2);
        robotYaw = state(3);
        flatPoles = state(4:end);
        poles = reshape(flatPoles, 2, []);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %planner
        replan = false;
        %update goal if new target found
        if targetDistace < Inf
            %TO DO: calculate world position of target
            newGoalState = ?targetPose
            %update goal and replan
            if (newGoalState(1) ~= goalState(1) || newGoalState(2) ~= goalState(2))
                goalState=newGoalState;
                rPlan.updateGoal(goalState);
                replan = true;
            end
        end
        
        %replan every 10 loops
        if (mod(seq, 10)==5 || replan )
            rPlan.replanConverge(poles);
            rPlan.Astar(state);
        end
        % find next goal
        goal = rPlan.findGoal(state);
       
    end
    
    
    % control stuff 
    uctrl = pid_ctrl(target_pose, state(1:3), 1.0, 0.5);
    u_contrl_cell{seq} = uctrl;
    
    vel = uctrl(1, 1);
    omega = uctrl(2, 1);
    
    SendSpeedCommand( vel, omega, config.control_channel)
    
    state_cell{seq} = state;
    P_cell{seq} = P;
    
    SLAMvisualization(state, P, target_pose)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    disp(seq);
    pause(0.05); % don't overload moos w/commands
    
end

%%
% save('data_Wed1', 'state_cell', 'P_cell', 'scans', 'cams', 'odoms' )

