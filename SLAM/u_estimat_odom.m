function u_odom = u_estimat_odom(wheel_odometry_buffer, scan_time_stamp)

%odomIdx = interp1(double([wheel_odometry_buffer.source_timestamp]), 1:length(wheel_odometry_buffer), double(scan_time_stamp), 'nearest');

[~, odomIdx] = min([wheel_odometry_buffer.source_timestamp] - scan_time_stamp);

closest_wheel_odom_match = wheel_odometry_buffer(odomIdx);

delta_x = double(closest_wheel_odom_match.x);
delta_y = double(closest_wheel_odom_match.y);
delta_theta = double(closest_wheel_odom_match.yaw);
delta_t = double(closest_wheel_odom_match.source_timestamp - ...
    closest_wheel_odom_match.destination_timestamp)*1e-6;

u_odom = [delta_x; delta_y; delta_theta]/ delta_t;
