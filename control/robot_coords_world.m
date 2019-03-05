function rot_mat = robot_coords_world(theta_r)

% pose is defined as [a b theta]'
% a is forward 
% b is right
% theta is in the positive anti-clockwise direction


rot_mat = [cos(theta_r), -sin(theta_r);
           sin(theta_r), cos(theta_r)];
       
end