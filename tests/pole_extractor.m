load('6_feb_test_1')

for k = 1 : length(scans)

    if ~isempty(scan)
        angles = (10:0.5:170)';
        xs = scan.ranges(111:431).*sind(angles);
        ys = scan.ranges(111:431).*cosd(angles);
        distances = sqrt((xs(1:321-1) - xs(2:321)).^2 + (ys(1:321-1) - ys(2:321)).^2);
        objects_delims = [0;find(distances > 0.8);320]+1;
        num_delims = numel(objects_delims);
        object_ranges = [objects_delims(1:num_delims-1)'; objects_delims(2:num_delims)'-1]';
        object_ranges_midpoints = round((object_ranges(:,1) + object_ranges(:,2))/2);
        object_reflectances = ones(num_delims-1, 1);
        for i = 1:(num_delims-1)
            object_reflectances(i) = mean(scan.reflectances((110+object_ranges(i,1)):(110+object_ranges(i,2))));
        end
        pole_indices = find(object_reflectances > 600);
        poles = ones(2, numel(pole_indices));
        for j = 1:numel(pole_indices)
           midpoint = object_ranges_midpoints(pole_indices(j));
           poles(1,j) = xs(midpoint);
           poles(2,j) = ys(midpoint);
        end
        figure(1); clf;
        scatter(xs,ys, 10, 'blue')
        xlim([-2 8])
        ylim([-8 8])
        axis equal
        hold on
        scatter(poles(1,:), poles(2,:), 20, 'red')
    end
    
end
