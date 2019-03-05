function u_odom = u_estimat_odom(wheel_odometry_buffer, scan_time_stamp)

odomIdx = interp1([wheel_odometry_buffer.source_timestamp], 1:10, scan_time_stamp, 'nearest');

closest_wheel_odom_match = wheel_odometry_buffer(odomIdx);

delta_x = closest_wheel_odom_match.x;
delta_y = closest_wheel_odom_match.y;
delta_theta = closest_wheel_odom_match.yaw;
delta_t = closest_wheel_odom_match.source_timestamp - closest_wheel_odom_match.destination_timestamp;

u_odom = [delta_x; delta_y; delta_theta]./ delta_t;
