function [points, adjGraph,distFromInitial] = RRTStar(poles, r, initialState, iters, near, goal, goalBias) %traversable, spaceSample, goalSample)
% Plan (or replan) with RRT
%   poles = [x1,y1;x2,y2;....], initalState = [x,y]
points = [initialState];
adjGraph = [0];
distFromInitial = [0];
parent=[0];
figure(1); 
clf;
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
    newDist = sqrt(sum((points-newPoint).^2,2));
    closest = min(newDist);
    if (closest>near/2 && ~checkCollision(nearestPoint, newPoint, poles', r))
       %SELECT BEST PARENT
       %pick smartestPointIndex
       nearPoints = newDist<1.5*near;
       M = min(distFromInitial(nearPoints));
       smartestPoints = find(distFromInitial==M);
       smartestPointIndex = smartestPoints(1) ;
       smartestPoint = points(smartestPointIndex,:);
       if(~checkCollision(smartestPoint, newPoint, poles', r))
           newDist(smartestPointIndex);
           %add newPoint to points, adjGraph and distFromIntial
           points = [points; newPoint];
           [n,m] = size(points);
           newCol = zeros(n-1,1);
           newCol(smartestPointIndex,1)=1;
           newRow = zeros(1,n);
           newRow(1, smartestPointIndex,1)=1;
           adjGraph = [adjGraph, newCol; newRow];
           parent = [parent;smartestPointIndex];
           parentDist = distFromInitial(smartestPointIndex);
           edgeDist = parentDist + sqrt(sum((smartestPoint-newPoint).^2,2));
           distFromInitial = [distFromInitial; edgeDist];
       else
           newDist(nearestPointIndex);
           %add newPoint to points, adjGraph and distFromIntial
           points = [points; newPoint];
           [n,m] = size(points);
           newCol = zeros(n-1,1);
           newCol(nearestPointIndex,1)=1;
           newRow = zeros(1,n);
           newRow(1, nearestPointIndex,1)=1;
           adjGraph = [adjGraph, newCol; newRow];
           parent = [parent;nearestPointIndex];
           parentDist = distFromInitial(nearestPointIndex);
           edgeDist = parentDist + sqrt(sum((nearestPoint-newPoint).^2,2));
           distFromInitial = [distFromInitial; edgeDist];
       end
       %REFACTOR NEAR NODES\
       proposedDist = edgeDist + newDist;
       betterOptions = (proposedDist < distFromInitial(1:end-1));
       newChildren = betterOptions & nearPoints;
       [len1,len2]=size(adjGraph);
       if(sum(newChildren)>0)
           for j=1:len1-1
               if newChildren(j) & ~ checkCollision(points(j,:), newPoint, poles', r)
                   adjGraph(j,len1)=1;
                   adjGraph(len1,j)=1;
                   adjGraph(parent(j),j)=0;
                   adjGraph(j,parent(j))=0;
                   %x = [points(j, 1)'; points(len1,1)'];
                   %y = [points(j, 2)'; points(len1,2)'];
                   %line(x, y, 'Color', 'red','LineWidth',2);
                   %hold on
                   parent(j)=len1;
                   distFromInitial(j)=proposedDist(j);
               end    
           end
       end
       
    end
end
scatter(points(:,1), points(:,2))
[I,J] = ind2sub(size(adjGraph), find(triu(adjGraph,1)));
x = [points(I, 1)'; points(J,1)'];
y = [points(I, 2)'; points(J,2)'];
line(x, y, 'Color', [0.4 0.4 0.4 0.4]);
hold on 
viscircles(poles, r*ones(1,length(poles')));
xlim([-5 5])
ylim([-1 6])
end





