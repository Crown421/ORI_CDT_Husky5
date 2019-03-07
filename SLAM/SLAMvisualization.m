function SLAMvisualization(state, P, target, fig)
%SLAMVISUALIZATION Summary of this function goes here
%   Detailed explanation goes here
    
    if ~exist('igh', 'var')
        fig = 4;
    end
    
   state_rob = state(1:3);
   state_pol = state(4:end);
   
   P_rob = P(1:3, 1:3);
   P_map = P(4:end, 4:end);
   
   scale = 1;
   
   figure(fig); clf;
   plot(state_rob(1), state_rob(2), 'rx')
   set(gca, 'YDir', 'reverse');
   hold on
   if exist('target', 'var')
       rectangle('Position', [target(1)-0.2, ...
           target(2)-0.2, ...
           0.4, 0.4], ...
           'Curvature', [1 1], 'FaceColor','red')
   end
   rot_rob = robot_coords_world(state_rob(3));
   arrow_head = state_rob(1:2) + rot_rob*[-0.4; 0];
    
   plot([state_rob(1), arrow_head(1, 1)], [state_rob(2), arrow_head(2, 1)], 'k', 'LineWidth', 3)
    
   % uncertainty of robot position
   rectangle('Position', [state_rob(1)-0.5*P_rob(1, 1)*scale, ...
           state_rob(2)-0.5*P_rob(2, 2)*scale, ...
           P_rob(1, 1)*scale, P_rob(2, 2)*scale], ...
           'Curvature', [1 1])
       
   plot(state_pol(1:2:end, 1), state_pol(2:2:end, 1), '.', 'MarkerSize', 12)
   
   for index = 1:2:length(state_pol)
      
       rectangle('Position', [state_pol(index)-0.5*P_map(index, index)*scale, ...
           state_pol(index+1)-0.5*P_map(index+1, index+1)*scale, ...
           P_map(index, index)*scale, P_map(index+1, index+1)*scale], ...
           'Curvature', [1 1])
       
   end
   %title(num2str(k))
   
   ylim([-5, 5])
   xlim([-1, 8])
   hold off
   
   
end

