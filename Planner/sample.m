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


