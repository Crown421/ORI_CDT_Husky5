function z_raw = sysnth_z_raw(ground_truth, x_rob)

x_robot = x_rob(1:3, 1);

no_sensed_beacons = length(ground_truth(1, :));
xy_rob_to_pole = ground_truth(:) - repmat(x_robot(1:2, 1), no_sensed_beacons, 1);

x_rob_pole = xy_rob_to_pole(1:2:end, 1)';
y_rob_pole = xy_rob_to_pole(2:2:end, 1)';

z_raw = zeros(2, no_sensed_beacons);

z_raw(1, :) = sqrt(x_rob_pole.^2 + y_rob_pole.^2);
z_raw(2, :) = atan2(x_rob_pole, y_rob_pole);

end

