function [best_ellipse] = findEllipse(rgb)

dist_threshold = 2;
best_ellipse = zeros(5,1);
best_ellipse_count = 0;
[~, maskedImage] = createMask(rgb);
smoothedImage = medfilt2(rgb2gray(maskedImage));
edges = edge(smoothedImage);
[edgeRows, edgeCols] = find(edges);
num_edges = numel(edgeRows);
count_threshold = 0.1*num_edges;

accumulator = zeros(512,1);
for i = 1:num_edges
   for j = 1:num_edges
       dist0 = sqrt((edgeRows(i) - edgeRows(j)).^2 + (edgeCols(i) - edgeCols(j)).^2);
       if dist0 < dist_threshold
          continue 
       end
       x0 = (edgeRows(i) + edgeRows(j))/2;
       y0 = (edgeCols(i) + edgeCols(j))/2;
       a = dist0/2;
       alpha = atan2(edgeCols(j)-edgeCols(i), edgeRows(j)-edgeRows(i));
       for k = 1:num_edges
           dist3 = sqrt((edgeRows(k) - x0).^2 + (edgeCols(k) - y0).^2);
           if dist3 < dist_threshold || dist3 > a
               continue
           end
           f = sqrt((edgeRows(k) - edgeRows(j)).^2 + (edgeCols(k) - edgeCols(j)).^2);
           ct = ((a.^2 + dist3.^2 - f.^2)/(2*a*dist3)).^2;
           st = 1 - ct;
           b = round(sqrt((a.^2 * dist3.^2 * st)/(a.^2-dist3.^2 * ct)));
           if b >=1 && b <= 384
               accumulator(b) = accumulator(b) + 1;
           end
       end
       [count, maxEl] = max(accumulator);
       if count > max(count_threshold, best_ellipse_count)
           best_ellipse_count = count;
           best_ellipse = [x0, y0, a, maxEl, alpha];
       end
       accumulator = zeros(384,1);
   end
end