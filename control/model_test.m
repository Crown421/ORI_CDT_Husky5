
Ts = 0.1;

time_out = 5;
no_steps = time_out/Ts;

x_robot = zeros(3, no_steps);
x_robot(3, 1) = 0.0;

u_robot = x_robot(1:2, :);
gamma = x_robot(1, :);
for_error = gamma;

target_pose = [1; 2; 0.1];

figure
for k = 1 : no_steps-1
    
    [u_ctrl, gamma(:, k), for_error(:, k)] = pid_ctrl(target_pose, x_robot(:, k), 3.0, 1.0);
%     u_ctrl = [0.1; 0.1];
    
    u_robot(:, k) = u_ctrl;
    [x_robot(:, k+1), u_odom] = basic_robot_plant(x_robot(:, k), u_ctrl, Ts);
    % update odoemtry
    
    rot_rob = robot_coords_world(x_robot(3, k));
    arrow_head = x_robot(1:2, k) + rot_rob*[-0.2; 0];
    
    plot([x_robot(2, k), arrow_head(2, 1)], [x_robot(1, k), arrow_head(1, 1)], 'k')
    axis equal
    hold on
    grid on
    plot(target_pose(2), target_pose(1), 'bx')
    
    % hold off
    pause(0.1)
    
   
end

end_error = x_robot(:, no_steps) - target_pose;
