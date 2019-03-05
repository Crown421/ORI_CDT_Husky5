classdef simulator
    %SIMULATOR Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        poles
        robot
    end
    
    methods
        function self = simulator(inputArg1,inputArg2)
            %SIMULATOR Construct an instance of this class
            %   Detailed explanation goes here
            self.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

