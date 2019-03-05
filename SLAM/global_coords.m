function world_frame_coords = global_coords(x, z_raw)

xVehicle = x(1:3, 1);

% in the robots frame of reference
z_rawXY = [z_raw(:,1).*cos(z_raw(:,2)) z_raw(:,1).*sin(z_raw(:,2))];

rot_mx = [cos(xVehicle(3)) sin(xVehicle(3));
                -sin(xVehicle(3)) cos(xVehicle(3))];
  
% added the robot co ords to make world co ords
world_frame_coords = rot_mx * z_rawXY' + xVehicle(1:2, 1);
end

