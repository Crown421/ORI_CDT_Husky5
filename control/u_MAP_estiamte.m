function u_MAP = u_MAP_estiamte(u_odom, uctrl, state, Ts)

% Ts is the difference in time between the two time stamps

% put speed limits on
if abs(uctrl(1)) > 0.5
    uctrl(1) = 0.5 * (uctrl(1)/abs(uctrl(2)));
end

if abs(uctrl(2)) > 0.5
    uctrl(2) = 0.5 * (uctrl(2)/abs(uctrl(2)));
end

rob_state = state(1:3, 1);
[~, u_f, ~] = basic_robot_plant(rob_state, uctrl, Ts);
%% make the covariance matrices

sig_f = diag([0.2, 0.2, 0.5]);

sig_odom = diag([0.2, 0.2, 1.0]);

u_MAP = (sig_odom.^(-1) + sig_f.^(-1)) \ (u_odom'*sig_odom.^(-1) + u_f'*sig_f.^(-1));

end

    
