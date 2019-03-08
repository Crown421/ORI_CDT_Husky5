function [best_ellipse] = findEllipse(rgb)

dist_threshold = 5;
best_ellipse = zeros(5,1);
best_ellipse_count = 0;
%[~, maskedImage] = createMask(rgb);
%smoothedImage = medfilt2(rgb2gray(maskedImage));
rgb = rgb(193:384,:,:);
X = uint8(RGB2IlluminationInvariant(double(rgb)/255, 0.3975)*255);
BW = X > 80;
smoothed_BW = medfilt2(BW);
edges = edge(smoothed_BW);
[edgeRows, edgeCols] = find(edges);

num_edges = numel(edgeRows);
count_threshold = 0.05*num_edges;
k=50;

indices = randsample(num_edges^2, k*num_edges);

accumulator = zeros(512, 1);
for i = 1:(k*num_edges)
   first_index = floor(indices(i)/num_edges)+1;
   second_index = rem(indices(i), num_edges)+1;
   dist0 = sqrt((edgeRows(first_index) - edgeRows(second_index)).^2 + (edgeCols(first_index) - edgeCols(second_index)).^2);
   if dist0 < dist_threshold
      continue 
   end
   x0 = (edgeRows(first_index) + edgeRows(second_index))/2;
   y0 = (edgeCols(first_index) + edgeCols(second_index))/2;
   a = dist0/2;
   alpha = atan2(edgeCols(second_index)-edgeCols(first_index), edgeRows(second_index)-edgeRows(first_index));
   for k = 1:num_edges
       dist3 = sqrt((edgeRows(k) - x0).^2 + (edgeCols(k) - y0).^2);
       if dist3 < dist_threshold || dist3 > a
           continue
       end
       f = sqrt((edgeRows(k) - edgeRows(second_index)).^2 + (edgeCols(k) - edgeCols(second_index)).^2);
       ct = ((a.^2 + dist3.^2 - f.^2)/(2*a*dist3)).^2;
       st = 1 - ct;
       b = round(sqrt((a.^2 * dist3.^2 * st)/(a.^2-dist3.^2 * ct)));
       if b >=1 && b <= 512
           accumulator(b) = accumulator(b) + 1;
       end
   end
   [count, maxEl] = max(accumulator);
   if count > max(count_threshold, best_ellipse_count)
       best_ellipse_count = count;
       best_ellipse = [192+x0, y0, a, maxEl, alpha];
   end
   accumulator = zeros(512,1);
end