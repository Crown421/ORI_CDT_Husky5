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
        iters = 200;
        goalBias = 0.2;
        ptHorizon = 0.4;
        tooClose = 0.2;
    end
    
    methods
        
        function self = plan(state, area)
            %plan Construct an instance of this class
            %   Detailed explanation goes here
            self.tree.points = state(1:2);
            self.tree.adj = sparse(0);
            self.tree.nrP = 1;
            self.mode = targetMode.findTarget;
            self.area = area;
            self.path = [];
            self.target = [];
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
        
        function [] = replan(self, state, poles )
            %replan Do replanning
            %   Replan if new poles or the target have shown up
            
            self.buildTree(poles)
            
            
            
            %possibly seperate full replanning method after mode change
        end
        
        
        function buildTree(self, poles)
            [row,col,v] = find(self.tree.adj)
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
                if ~checkCollision(nearestPoint, newPoint, poles, self.radius)
                    self.tree.points = [self.tree.points; newPoint];
                    self.tree.nrP = self.tree.nrP + 1;
                    self.tree.adj(nearestPointIndex, self.tree.nrP)  = 1;
                    self.tree.adj(self.tree.nrP, nearestPointIndex)  = 1;
                    
                    if newPoint(1) > self.area(1,2)
                        self.target.coords = newPoint;
                        self.target.idx = self.tree.nrP;
                        break
                    end
                    
                end   
            end

        end
        
        
        function Astar(self, state)
            % accessing columns is much faster
            
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
            Fscore(startIdx) = norm(self.target.coords- self.tree.points(startIdx) );
            
            while ~isempty(open)
                % current should index on whole point list
                [~, minIdx] = min(Fscore(open));
                current = open(minIdx);
                
                if current == self.target.idx
                    flPath = current;
                    while flPath(end)~=startIdx
                        flPath = [flPath, cameFrom(flPath(end))];
                    end
                    self.path = flip(flPath);
                    return
                end
                
                open(open==current) = [];
                closed = [closed, current];
                
                for neighbour = find(self.tree.adj(:, current))'
                    if ismember(neighbour, closed)
                        continue
                    end
                    
                    prelimScore = Gscore(current) + norm(self.tree.points(current) - self.tree.points(neighbour));
                    
                    if ~ismember(neighbour, open)
                        open = [open, neighbour];
                    elseif prelimScore >= Gscore(neighbour)
                        continue
                    end
                    
                    cameFrom(neighbour) = current;
                    Gscore(neighbour) = prelimScore;
                    Fscore(neighbour) = Gscore(neighbour) + norm(self.target.coords-self.tree.points(neighbour));
                    
                    
                end
                
                
                
            end
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
            x = self.area(1,2) + rand(1)*0.05*diff(self.area(1,:));
            y = rand(1)*diff(self.area(2,:)) + self.area(2,1);
            candPoint = [x,y];
        end
        
    end
end

