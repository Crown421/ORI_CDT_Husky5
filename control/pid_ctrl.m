function [uctrl, gamma, for_error] = pid_ctrl(target_pose, current_pose_wrld, max_gain_intercpt, max_gain_forward)
%
% take a target pose in the world co-ordinates and give linear and yaw
% velocities in the robot's frame of reference
% current_pose_wrld: 
% 
% target_pose in world

% the angle of the robot wrt to the world
rot_robot = current_pose_wrld(3);
% x difference in world co ords
% diff_x = target_pose(1) - current_pose_wrld(1);
% y differecen in world co ords
% diff_y = target_pose(2) - current_pose_wrld(2);
%% Find the angle to directly intercept the target pose

rot_mat_robot_wrld = robot_coords_world(rot_robot);
% vector to target from robot in world
RT_w = target_pose(1:2, 1) - current_pose_wrld(1:2, 1);
% vec to targe from robot in robot co ords
RT_r = rot_mat_robot_wrld' * RT_w;
% intercept angle
gamma = atan2(RT_r(2, 1), RT_r(1, 1));
% 
% forward error
for_error = RT_r(1, 1);
% put a buffer intercept angle
if abs(gamma) > 0.1
    
    gain_forward = 0.1;
    gain_intercpt = max_gain_intercpt;
    
else
    gain_intercpt = 0.1;
    gain_forward = max_gain_forward;
end

if abs(for_error) < 0.2
    gain_intercpt = 0.01;
    
end

ang_vel = gain_intercpt * gamma;
%% Find the forward velocity command

% gain_forward = 0.5;
for_vel = gain_forward * for_error;

% return the forward input
uctrl = [for_vel; ang_vel];
end

