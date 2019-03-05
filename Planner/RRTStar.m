function [points, adjGraph] = RRTStar(poles, r, initialState, iters, near, goal, goalBias) %traversable, spaceSample, goalSample)
% Plan (or replan) with RRT
%   poles = [x1,y1;x2,y2;....], initalState = [x,y]
points = [initialState];
adjGraph = [0];
for i = 1:iters
    %sample a random point
    [x,y] = sample(goalBias, goal);
    randPoint= [x,y];
    %find the closest point in the graph
    dist = sqrt(sum((points-randPoint).^2,2));
    [M, nearestPointIndex] = min(dist);
    nearestPoint = points(nearestPointIndex,:);
    % steer to find new point for the graph
    steerToward = near/norm(randPoint-nearestPoint)*(randPoint-nearestPoint);
    newPoint = nearestPoint + steerToward;
    if ~checkCollision(nearestPoint, newPoint, poles', r)
       points = [points; newPoint];
       [n,m] = size(points);
       newCol = zeros(n-1,1);
       newCol(nearestPointIndex,1)=1;
       newRow = zeros(1,n);
       newRow(1, nearestPointIndex,1)=1;
       adjGraph = [adjGraph, newCol; newRow];
    end
end
figure(1); 
clf;
scatter(points(:,1), points(:,2))
[I,J] = ind2sub(size(adjGraph), find(triu(adjGraph,1)));
x = [points(I, 1)'; points(J,1)'];
y = [points(I, 2)'; points(J,2)'];
line(x, y, 'Color', [0.4 0.4 0.4 0.4]);
hold on 
viscircles(poles, r*ones(1,length(poles')));
xlim([0 11])
ylim([0 11])
end


function [x,y] = sample(goalBias, goal)
    sampleChoice = rand;
    if sampleChoice < goalBias 
        [x,y] = goalSample(goal);
    else
        [x,y] = spaceSample();
    end
end

function [x,y] = spaceSample()
    x = 10*rand();
    y = 10*rand();
end

function [x,y] = goalSample(goal)
    [m,n] = size(goal);
    which = randi(m);
    point = goal(which,:);
    x= point(1);
    y = point(2);
end


