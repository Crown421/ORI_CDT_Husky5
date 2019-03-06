% extract map and robot

% load('slam_run_0')

run_steps = length(state_cell);
robot_state = zeros(3, run_steps);

for k = 1: length(state_cell)
    
    
   state_data = state_cell{k};
   robot_state(:, k) = state_data(1:3, 1);
   
   map_data = state_data(4:end, 1);
   
   P_data = P_cell{k};
   P_data_map = P_data(4:end, 4:end);
   
   figure(1);
   clf;
   plot(robot_state(1, k), robot_state(2, k), 'rx')
   hold on
   plot(map_data(1:2:end, 1), map_data(2:2:end, 1), '.', 'MarkerSize', 6)
   scale = 2;
   for index = 1:2:length(map_data)
      
       rectangle('Position', [map_data(index)-0.5*P_data(index, index)*scale, ...
           map_data(index+1)-0.5*P_data(index+1, index+1)*scale, ...
           P_data(index, index)*scale, P_data(index+1, index+1)*scale], ...
           'Curvature', 1)
       
   end
   
   ylim([-5, 5])
   xlim([-5, 5])
   hold off
   
   pause(0.1)
    
end