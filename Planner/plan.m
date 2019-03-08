classdef plan < handle
    %PLAN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties%(Access = private)
        tree
        path
        pathLength
        target
        nTargets
        sampleArea
        mode %initialise as targetMode enum
        area
        radius = 0.3;
        iters = 800;
        goalBias = 0.4;
        ptHorizon = 0.8;
        converge = 0.9999;
        goalTrigger = .1;
        goalCounter = 2;
        %tooClose = 0.2;
    end
    
    methods
        
        function self = plan(state, area)
            %plan Construct an instance of this class
            %   Detailed explanation goes here
            self.tree.points = state(1:2);
            self.tree.adj = sparse(0);
            self.tree.nrP = 1;
            self.tree.distFromInit = [0];
            self.tree.parents = [0];
            self.mode = targetMode.findTarget;
            self.area = area;
            self.sampleArea = area;
            self.path = [];
            self.pathLength = -1;
            self.nTargets = 6;
            self.target.coords = [self.area(1,2)*ones(self.nTargets,1),linspace(self.area(2,1)*0.7, self.area(2,2)*0.7, self.nTargets)'] ;
            self.target.idx = zeros(1,self.nTargets);
            self.mode = targetMode.findTarget;
            self.goalCounter = 2;

        end
        
        function nextGoal = findGoal(self,state)
            %findGoal Find the next goal given the state between replanning
            %   Detailed explanation goes here
            % TODO: better tracking of which goals were already passed
            %candGoalsIdx = self.path(self.tree.points(self.path,1)>state(1));
            %candGoals = self.tree.points(candGoalsIdx, :);
            %dist = sqrt(sum((candGoals - state(1:2)).^2,2));
            %[~, nextGoalIdx] = min(dist);
            %nextGoal = candGoals(nextGoalIdx);
            robotLoc = [state(1), state(2)];
            lastGoalIndex=self.path(self.goalCounter-1);
            lastGoal = self.tree.points(lastGoalIndex,:);
            currentGoal = self.tree.points(self.path(self.goalCounter),:);
            distToCurrentGoal = sqrt(sum((currentGoal - robotLoc).^2,2));
            if distToCurrentGoal < self.goalTrigger*(sqrt(sum((lastGoal - currentGoal).^2,2)))
                nextGoal=self.tree.points(self.path(self.goalCounter+1),:);
                self.goalCounter=self.goalCounter+1;
            else
                nextGoal=currentGoal;
            end
            %self.goalCounter
        end
        
        
        function updateGoal(self, goals)
            [self.nTargets,~] = size(goals);
            self.target.coords = goals;
            self.target.idx = zeros(1,self.nTargets);
            
        end
        
        function [] = replan(self, state, poles )
            %replan Do replanning
            %   Replan if new poles or the target have shown up
            self.goalCounter = 2;
            self.tree.points = state(1:2);
            self.tree.adj = sparse(0);
            self.tree.nrP = 1;
            self.buildTreeStar(poles)
            self.Astar(state);

            %possibly seperate full replanning method after mode change
        end
        
        function [] = replanConverge(self, state, poles )
            %replan Do replanning
            %   Replan if new poles or the target have shown up
            
            %initalPlan
            
            self.tree.points = state(1:2);
            self.tree.adj = sparse(0);
            self.tree.nrP = 1;
            widthScale = .2;
            prevPathLength = intmax;
            iterCount = 1;
            self.tree.distFromInit = [0];
            self.tree.parents = [0];
            self.goalCounter = 2;
            self.updateSampleArea(widthScale)
            
            
            
            self.buildTreeStar(poles)
            self.Astar(state);

              

            %improve until convergence
            while(self.pathLength/prevPathLength<self.converge && iterCount<20)
                
                iterCount=iterCount+1;
                prevPathLength = self.pathLength;
                self.updateSampleArea(widthScale*iterCount)
                self.buildTreeStar(poles)
                self.Astar(state);
                self.pathLength;
                self.pathLength/prevPathLength;
            end
            
            
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
            for i = 1:self.nTargets
                targPoint = self.target.coords(i,:);
                newDist = sqrt(sum((self.tree.points - targPoint).^2,2));
                [tarDist, nearestPointIndex] = min(newDist);
                if tarDist < 2*self.ptHorizon
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
                    %disp(size(self.tree.points))
                    %disp(size(self.tree.distFromInit))
                    
                    % find nearby point with smallest distance to start
                    [minDist] = min(self.tree.distFromInit(nearPointsLdx));
                    %[~, minDistIdx] = min(self.tree.distFromInit(nearPointsLdx))
                    
                    %%% this is bad, and connects to some other point,
                    %%% which happens to have the same distance
                    smartPointIdx = find(self.tree.distFromInit == minDist & nearPointsLdx); % TODO maybe improve
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
                    self.tree.parents = [self.tree.parents, nearestPointIndex];
                    newPointDistInit = self.tree.distFromInit(nearestPointIndex) + newDist(nearestPointIndex);
                    self.tree.distFromInit = [self.tree.distFromInit, newPointDistInit];
                    
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
                    
                    betterOptions = proposedDistance < self.tree.distFromInit(1:end-1);
                    newChildren = betterOptions & nearPointsLdx;

                    if any(newChildren)
                        for toChangeIdx = find(newChildren)
                            if  ~checkCollision(self.tree.points(toChangeIdx,:), newPoint,poles, self.radius)
                                
%                                 %%%
%                                 x = [self.tree.points(toChangeIdx, 1)'; self.tree.points(self.tree.nrP,1)'];
%                                 y = [self.tree.points(toChangeIdx, 2)'; self.tree.points(self.tree.nrP,2)'];
%                                 line(x, y, 'Color', 'red', 'LineWidth', 2);
%                                 pause(0.2)
%                                 %%%
                                
                                self.tree.adj(toChangeIdx, self.tree.nrP) = 1;
                                self.tree.adj(self.tree.nrP, toChangeIdx) = 1;
                                
                                self.tree.adj(toChangeIdx, self.tree.parents(toChangeIdx)) = 0;
                                self.tree.adj(self.tree.parents(toChangeIdx), toChangeIdx) = 0;
                                self.tree.parents(toChangeIdx) = self.tree.nrP;
                                self.tree.distFromInit(toChangeIdx) = proposedDistance(toChangeIdx);
                            end
                        end
                        
                        
                    end
                    
                end
            end
            
            % include targets in point list
            for i = 1:length(self.nTargets)
            targPoint = self.target.coords(i,:);
                newDist = sqrt(sum((self.tree.points - targPoint).^2,2));
                [tarDist, nearestPointIndex] = min(newDist);
                if tarDist < 2*self.ptHorizon
                    self.tree.points = [self.tree.points; targPoint];
                    self.tree.nrP = self.tree.nrP + 1;
                    self.tree.adj(nearestPointIndex, self.tree.nrP)  = 1;
                    self.tree.adj(self.tree.nrP, nearestPointIndex)  = 1;
                    self.target.idx(i) = self.tree.nrP;
                    self.tree.parents = [self.tree.parents, nearestPointIndex];
                    newPointDistInit = self.tree.distFromInit(nearestPointIndex) + newDist(nearestPointIndex);
                    self.tree.distFromInit = [self.tree.distFromInit, newPointDistInit];
                end
            end    
        end







%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        function Astar(self, state)
            % accessing columns is much faster
            
            self.nTargets = size(self.target.coords,1);
            pathCands = cell(1,self.nTargets);
            pathCandsLength = nan(1,self.nTargets);
            
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
           
            [self.pathLength, minIdx] = min(pathCandsLength);
            self.path = pathCands{minIdx};
            
            figure(5); clf;
            hold on
            targetLine = plot(self.area(1,[2,2]), self.area(2,:), 'LineWidth',6);
            targetLine.Color(4) = 0.5;
            %plot(poles(1,:), poles(2,:), 'o')
            axis equal
            xlim(self.area(1,:)+[0,1])
            ylim(self.area(2,:))
            
            scatter(self.tree.points(:,1), self.tree.points(:,2), 10, 'filled');
            [I,J] = ind2sub(size(self.tree.adj), find(triu(self.tree.adj,1)));
            x = [self.tree.points(I, 1)'; self.tree.points(J,1)'];
            y = [self.tree.points(I, 2)'; self.tree.points(J,2)'];
            line(x, y, 'Color', [0.4 0.4 0.4 0.4]);
            plot(self.target.coords(1), self.target.coords(2), 'o')
            
            %
            srcidx = self.path(1:end-1);
            tgtidx = self.path(2:end);
            x = [self.tree.points(srcidx, 1)'; self.tree.points(tgtidx,1)'];
            y = [self.tree.points(srcidx, 2)'; self.tree.points(tgtidx,2)'];
            line(x, y, 'Color', [0.4 0.4 0.4 1], 'LineWidth', 3);
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
            x = rand(1)*diff(self.sampleArea(1,:)) + self.sampleArea(1,1);
            y = rand(1)*diff(self.sampleArea(2,:)) + self.sampleArea(2,1);
            candPoint = [x,y];
        end
        
        function candPoint = exploreGoalSample(self)
            m = size(self.target.coords,1);
            which = randi(m);
            candPoint = self.target.coords(which,:);
        end
        
        function updateSampleArea(self,widthScale)
            if widthScale < 1
                self.sampleArea(2,:)=widthScale*self.area(2,:);
            else 
                self.sampleArea(2,:)=self.area(2,:);
            end
        end
        
    end
end

