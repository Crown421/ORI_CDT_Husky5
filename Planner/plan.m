classdef plan
    %PLAN Summary of this class goes here
    %   Detailed explanation goes here
    
    properties%(Access = private)
        tree
        path
        target
        mode %initialise as targetMode enum
    end
    
    methods
        
        function self = plan(state)
            %plan Construct an instance of this class
            %   Detailed explanation goes here
            self.tree = [];
            self.path = [];
            self.target = [];
            self.mode = targetMode.findTarget;
        end
        
        function nextGoal = findGoal(self,state)
            %findGoal Find the next goal given the state between replanning
            %   Detailed explanation goes here
            nextGoal = [];
        end
        
        function [] = replan(self,state)
            %replan Do replanning
            %   Replan if new poles or the target have shown up
            self.tree = [];
            self.path = [];
            
            %possibly seperate full replanning method after mode change
        end
    end
end

