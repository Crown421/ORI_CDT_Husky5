%% test the slam with a toy example

% x(1) = x
% x(2) = y
% x(3) = theta (radians)
x_robot = [0; 0; 0];

u_ctrl = [0.1; 0.0];

Ts = 0.1;

[xpred, u_odom] = basic_robot_plant(x_robot, u_ctrl, Ts);

z_raw = [2.0, 0.3*pi;
         1.5, 0.5*pi;
         0.7, -0.2*pi;
         3.4, 0.0*pi]';
% no. of beacons found
no_sensed_beacons = length(z_raw(1, :));

% transpose the z_raw data to make it repmat([range, bearing]; [range, bearing]...)
% initial guess for the beacons in world coords
init_guess_beacon = global_coords(x_robot, z_raw');
% make beacon ground truth in world xy
true_beacon_pos = init_guess_beacon + 0.01*randn(size(init_guess_beacon));

P = diag([1, 1, 0.1, 0.01*ones(1, no_sensed_beacons*2)]);

x = [x_robot; init_guess_beacon(:)];
% z_raw is r theta from the world origin
% u: odometry input: this is the relative movement
u = u_odom;
[x_new, P_new] = SLAMUpdate(u, z_raw, x, P);

%% run through a loop 

time_out = 10;
no_steps = time_out/Ts;

% xStates = zeros(length(x_new(:, 1)), no_steps);
% xStates(:, 1) = x_new;

x_cell = cell(1, no_steps);
x_cell{1, 1} = x_new;

x_robot = zeros(3, no_steps);
x_robot(:, 1) = xpred;

figure;
for k = 1 : no_steps-1
    
    u_ctrl = [0.1; 0.1];
    [x_robot(:, k+1), u_odom] = basic_robot_plant(x_robot(:, k), u_ctrl, Ts);
    % update odoemtry
    u = u_odom;
    % get noisy robot measurements
    z_raw = sysnth_z_raw(true_beacon_pos, x_robot(:, k));
    z_raw = z_raw + 0.01*randn(size(z_raw));
    
    [x_new, P_new] = SLAMUpdate(u, z_raw, x_cell{1, k}, P_new);
    x_cell{1, k+1} = x_new;
    
    plot(true_beacon_pos(1, :), true_beacon_pos(2, :), 'ro')
    hold on
    plot(x_new(4:2:end), x_new(5:2:end), 'b+')
    plot(x_new(1), x_new(2), 'rx')
    grid on;
    plot(x_robot(1, k+1), x_robot(2, k+1), 'kx')
    
    % hold off
    
    pause(0.1)
    
end


%%

% figure; plot(true_beacon_pos(1, :), true_beacon_pos(2, :), 'ro')
% hold on
% xy_cell_data = x_cell{1, 100};
% plot(xy_cell_data(4:2:end), xy_cell_data(5:2:end), 'b+')
% grid on;


