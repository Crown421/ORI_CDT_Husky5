classdef plan < handle
    %PLAN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties%(Access = private)
        tree
        path
        target
        mode %initialise as targetMode enum
        area
        radius = 0.2;
        iters = 500;
        goalBias = 0.4;
        ptHorizon = 0.4;
        %tooClose = 0.2;
    end
    
    methods
        
        function self = plan(state, area)
            %plan Construct an instance of this class
            %   Detailed explanation goes here
            self.tree.points = state(1:2);
            self.tree.adj = sparse(0);
            self.tree.nrP = 1;
            self.tree.distFromInit = 0;
            self.tree.parents = 0;
            self.mode = targetMode.findTarget;
            self.area = area;
            self.path = [];
            nTargets = 6;
            self.target.coords = [self.area(1,2)*ones(nTargets,1),linspace(self.area(2,1)*0.7, self.area(2,2)*0.7, nTargets)'] ;
            self.target.idx = zeros(1,nTargets);
            self.mode = targetMode.findTarget;

        end
        
        function nextGoal = findGoal(self,state)
            %findGoal Find the next goal given the state between replanning
            %   Detailed explanation goes here
            % TODO: better tracking of which goals were already passed
            candGoalsIdx = self.path(self.tree.points(self.path,1)>state(1));
            candGoals = self.tree.points(candGoalsIdx, :);
            dist = sqrt(sum((candGoals - state(1:2)).^2,2));
            [~, nextGoalIdx] = min(dist);
            nextGoal = candGoals(nextGoalIdx);
        end
        
        
        function updateGoal(self, goals)
            self.target.coords = goals;
            self.target.idx = zeros(1,length(goals));
            
        end
        
        function [] = replan(self, state, poles )
            %replan Do replanning
            %   Replan if new poles or the target have shown up
            
            self.tree.points = state(1:2);
            self.tree.adj = sparse(0);
            self.tree.nrP = 1;
            self.buildTreeStar(poles)
            
            self.Astar(state);
            
            
            
            %possibly seperate full replanning method after mode change
        end
       
        
        
        function buildTree(self, poles)
            [row,col,v] = find(self.tree.adj);
            nP = self.tree.nrP;
            self.tree.adj = sparse(row,col,v, nP+self.iters, nP+self.iters, nP+self.iters^2);
            
            for i = 1:self.iters
                candPoint = self.sample(self.goalBias, []);
                
                dist = sqrt(sum((self.tree.points - candPoint).^2,2));
                [~, nearestPointIndex] = min(dist);
                nearestPoint = self.tree.points(nearestPointIndex, :);
                % steer to find new point for the graph
                steerToward = self.ptHorizon/norm(candPoint-nearestPoint)*(candPoint-nearestPoint);
                newPoint = nearestPoint + steerToward;
                newDist = sqrt(sum((self.tree.points - newPoint).^2,2));
                closest = min(newDist);
                
%                 tarDist = sqrt(sum((self.target.coords - newPoint).^2,2));
%                 [minTarDist, minTarIdx] = min(tarDist);
%                 if minTarDist < self.ptHorizon*0.2
%                     newPoint = self.target.coords(minTarIdx, :);
%                     closest = self.ptHorizon;
%                     self.target.idx(minTarIdx) = self.tree.nrP;
%                 end
                
                if (closest > self.ptHorizon*0.5 ) && ~checkCollision(nearestPoint, newPoint, poles, self.radius)
                    self.tree.nrP = self.tree.nrP + 1;
%                     if minTarDist < self.ptHorizon*0.2
%                         self.target.idx(minTarIdx) = self.tree.nrP;
%                     end
                    
                    self.tree.points = [self.tree.points; newPoint];
                    self.tree.adj(nearestPointIndex, self.tree.nrP)  = 1;
                    self.tree.adj(self.tree.nrP, nearestPointIndex)  = 1;
                    
                end   
            end
            for i = 1:length(self.target.coords)
                targPoint = self.target.coords(i,:);
                newDist = sqrt(sum((self.tree.points - targPoint).^2,2));
                [tarDist, nearestPointIndex] = min(newDist);
                if tarDist < 2*self.ptHorizon
%                     self.tree.nrP = self.tree.nrP + 1;
%                     if minTarDist < self.ptHorizon*0.2
%                         self.target.idx(minTarIdx) = self.tree.nrP;
%                     end
                    
                    self.tree.points = [self.tree.points; targPoint];
                    self.tree.nrP = self.tree.nrP + 1;
                    self.tree.adj(nearestPointIndex, self.tree.nrP)  = 1;
                    self.tree.adj(self.tree.nrP, nearestPointIndex)  = 1;
                    self.target.idx(i) = self.tree.nrP;
                end   
            end    
        end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function buildTreeStar(self, poles)
            [row,col,v] = find(self.tree.adj);
            nP = self.tree.nrP;
            self.tree.adj = sparse(row,col,v, nP+self.iters, nP+self.iters, nP+self.iters^2);
            
            distFromInit = 0;
            parents = 0;
            
%             %%%
%             figure(40); clf;
%             viscircles(poles', self.radius*ones(1,length(poles)));
%             axis equal
%             hold on
%             scatter(self.tree.points(:,1), self.tree.points(:,2), 10, 'filled');
%             xlim(self.area(1,:)+[0,1])
%             ylim(self.area(2,:))
%             %%%
            
            for i = 1:self.iters
                candPoint = self.sample(self.goalBias, []);
                
                % find closest oint
                dist = sqrt(sum((self.tree.points - candPoint).^2,2));
                [~ , nearestPointIndex] = min(dist);
                nearestPoint = self.tree.points(nearestPointIndex, :);
                % steer to find new point for the graph
                steerToward = self.ptHorizon/norm(candPoint-nearestPoint)*(candPoint-nearestPoint);
                newPoint = nearestPoint + steerToward;
                newDist = sqrt(sum((self.tree.points - newPoint).^2,2))';
                closest = min(newDist);
                
                if (closest > self.ptHorizon*0.5 ) && ~checkCollision(nearestPoint, newPoint, poles, self.radius)
                    
%                     %%%
%                     plot(newPoint(1), newPoint(2), 'ro');
%                     %pause(0.2);
%                     %%%
                
                    % find points within distance
                    nearPointsLdx = newDist < 1.5*self.ptHorizon;
                    
%                     %%%
%                     figure(40);
%                     plot(self.tree.points(nearPointsLdx,1), self.tree.points(nearPointsLdx,2), 'yo')
%                     % pause(0.2)
%                     %%%
                    
                    % find nearby point with smallest distance to start
                    [minDist] = min(distFromInit(nearPointsLdx));
                    %[~, minDistIdx] = min(distFromInit(nearPointsLdx))
                    
                    %%% this is bad, and connects to some other point,
                    %%% which happens to have the same distance
                    smartPointIdx = find(distFromInit == minDist & nearPointsLdx); % TODO maybe improve
                    % TODO maker smarter
                    smartPointIdx = smartPointIdx(1);
                    smartPoint = self.tree.points(smartPointIdx, :);
                    
                    if ~checkCollision(smartPoint, newPoint, poles, self.radius)
                        nearestPointIndex = smartPointIdx;
                        nearestPoint = smartPoint;
                        %disp(sqrt(sum((nearestPoint - newPoint).^2,2)))
                    end
                    
                    
                    
                    %%% add to graph
                    self.tree.nrP = self.tree.nrP + 1;
                    
                    self.tree.points = [self.tree.points; newPoint];
                    self.tree.adj(nearestPointIndex, self.tree.nrP)  = 1;
                    self.tree.adj(self.tree.nrP, nearestPointIndex)  = 1;
                    
                    % update parents and distances
                    parents = [parents, nearestPointIndex];
                    newPointDistInit = distFromInit(nearestPointIndex) + newDist(nearestPointIndex);
                    distFromInit = [distFromInit, newPointDistInit];
                    
%                     %%%
%                     figure(40)
%                     plot(nearestPoint(1), nearestPoint(2), 'rx');
%                     
%                     x = [self.tree.points(nearestPointIndex, 1)'; self.tree.points(self.tree.nrP,1)'];
%                     y = [self.tree.points(nearestPointIndex, 2)'; self.tree.points(self.tree.nrP,2)'];
%                     line(x, y, 'Color', [0.4 0.4 0.4 0.9]);
%                     %pause(0.2)
%                     
%                     %%%
                    
                    
                    
                    % refactor nearby points now
                    proposedDistance = newPointDistInit + newDist;
                    
                    betterOptions = proposedDistance < distFromInit(1:end-1);
                    newChildren = betterOptions & nearPointsLdx;
                    
                    if any(newChildren)
                        for toChangeIdx = find(newChildren)
                            if ~checkCollision(self.tree.points(toChangeIdx,:), newPoint,poles, self.radius)
                                
%                                 %%%
%                                 x = [self.tree.points(toChangeIdx, 1)'; self.tree.points(self.tree.nrP,1)'];
%                                 y = [self.tree.points(toChangeIdx, 2)'; self.tree.points(self.tree.nrP,2)'];
%                                 line(x, y, 'Color', 'red', 'LineWidth', 2);
%                                 pause(0.2)
%                                 %%%
                                
                                self.tree.adj(toChangeIdx, self.tree.nrP) = 1;
                                self.tree.adj(self.tree.nrP, toChangeIdx) = 1;
                                
                                self.tree.adj(toChangeIdx, parents(toChangeIdx)) = 0;
                                self.tree.adj(parents(toChangeIdx), toChangeIdx) = 0;
                                parents(toChangeIdx) = self.tree.nrP;
                                distFromInit(toChangeIdx) = proposedDistance(toChangeIdx);
                            end
                        end
                        
                        
                    end
                    plot(self.tree.points(:,1), self.tree.points(:,2), 'ro')
                    
                end
            end
            
            % include targets in point list
            for i = 1:length(self.target.coords)
                targPoint = self.target.coords(i,:);
                newDist = sqrt(sum((self.tree.points - targPoint).^2,2));
                [tarDist, nearestPointIndex] = min(newDist);
                if tarDist < 2*self.ptHorizon
%                     self.tree.nrP = self.tree.nrP + 1;
%                     if minTarDist < self.ptHorizon*0.2
%                         self.target.idx(minTarIdx) = self.tree.nrP;
%                     end
                    
                    self.tree.points = [self.tree.points; targPoint];
                    self.tree.nrP = self.tree.nrP + 1;
                    self.tree.adj(nearestPointIndex, self.tree.nrP)  = 1;
                    self.tree.adj(self.tree.nrP, nearestPointIndex)  = 1;
                    self.target.idx(i) = self.tree.nrP;
                end   
            end    
        end







%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function Astar(self, state)
            % accessing columns is much faster
            
            nTargets = size(self.target.coords,1);
            pathCands = cell(1,nTargets);
            pathCandsLength = nan(1,nTargets);
            
            for candTargetIdx = find(self.target.idx>0)
                
                dist = sqrt(sum((self.tree.points - state(1:2)).^2,2));
                [~, startIdx] = min(dist);
                
                closed = [];
                %open = self.tree.points(minIdx, :);
                open = startIdx; %idx on full points list
                
                cameFrom = nan(1, self.tree.nrP);
                
                Gscore = nan(1, self.tree.nrP);
                Gscore(startIdx) = 0;
                % heuristic, simple distance to target
                Fscore = nan(1, self.tree.nrP);
                Fscore(startIdx) = norm(self.target.coords(candTargetIdx, :)- self.tree.points(startIdx) );
                
                while ~isempty(open)
                    % current should index on whole point list
                    [~, minIdx] = min(Fscore(open));
                    current = open(minIdx);
                    
                    if current == self.target.idx(candTargetIdx)
                        flPath = current;
                        while flPath(end)~=startIdx
                            flPath = [flPath, cameFrom(flPath(end))];
                        end
                        pathCands{candTargetIdx} = flip(flPath);
                        pathCandsLength(candTargetIdx) = Gscore(current);
                        
                        break
                    end
                    
                    open(open==current) = [];
                    closed = [closed, current];
                    
                    for neighbour = find(self.tree.adj(:, current))'
                        if ismember(neighbour, closed)
                            continue
                        end
                        
                        prelimScore = Gscore(current) + norm(self.tree.points(current, :) - self.tree.points(neighbour, :));
                        
                        if ~ismember(neighbour, open)
                            open = [open, neighbour];
                        elseif prelimScore >= Gscore(neighbour)
                            continue
                        end
                        
                        cameFrom(neighbour) = current;
                        Gscore(neighbour) = prelimScore;
                        Fscore(neighbour) = Gscore(neighbour) + norm(self.target.coords(candTargetIdx, :)-self.tree.points(neighbour, :));
                        
                        
                    end
                end
            end
            [~, minIdx] = min(pathCandsLength);
            self.path = pathCands{minIdx};
        end
            
            
        
        
        
        
        % possibly static?
        function candPoint = sample(self, goalBias, goal)
            sampleChoice = rand;
            if sampleChoice < goalBias
                candPoint = self.exploreGoalSample();
            else
                candPoint = self.spaceSample();
            end
        end
        
        function candPoint = spaceSample(self)
            x = rand(1)*diff(self.area(1,:)) + self.area(1,1);
            y = rand(1)*diff(self.area(2,:)) + self.area(2,1);
            candPoint = [x,y];
        end
        
        function candPoint = exploreGoalSample(self)
            m = size(self.target.coords,1);
            which = randi(m);
            candPoint = self.target.coords(which,:);
        end
        
    end
end

