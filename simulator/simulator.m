classdef simulator
    %SIMULATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        poles
        robot
        trajectory
    end
    
    methods
        function self = simulator(inputArg1,inputArg2)
            %SIMULATOR Construct an instance of this class
            %   Detailed explanation goes here
            self.Property1 = inputArg1 + inputArg2;
            % define finish line, place random poles, place robot, possibly
            % places walls
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
        
        % methods: 
        % first return only pole pisition, later maybe more realistic lidar
        % (expand a number of lines from the robot and check collisions
        % image return, can' fake it, maybe only return target location
        % at a certain min distance from it, with increasingly less noise
        % odometry data, difference from trajectory
        % need integrator for the robot, possibly at a separate frequency
        % from the data update frequency
        % send control signal, keep constant
        
        % work out frequency of things
    end
end

