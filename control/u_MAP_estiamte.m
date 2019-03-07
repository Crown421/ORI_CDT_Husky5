function u_MAP = u_MAP_estiamte(u_odom, uctrl, state, Ts)

if u_odom == zeros(3, 1)
    u_MAP = u_odom;
    return
end

% Ts is the difference in time between the two time stamps

% put speed limits on
if abs(uctrl(1)) > 0.5
    uctrl(1) = 0.5 * (uctrl(1)/abs(uctrl(2)));
end

if abs(uctrl(2)) > 0.5
    uctrl(2) = 0.5 * (uctrl(2)/abs(uctrl(2)));
end

rob_state = state(1:3, 1);
[~, u_f] = basic_robot_plant(rob_state, uctrl, Ts);
%% make the covariance matrices

inv_sig_f = diag([0.1, 0.1, 0.1].^(-1));

inv_sig_odom = diag([0.2, 0.2, 1.0].^(-1));

denom = (inv_sig_odom + inv_sig_f);
nunom = (inv_sig_odom*u_odom + inv_sig_f*u_f);

u_MAP =  denom \ nunom;



    
