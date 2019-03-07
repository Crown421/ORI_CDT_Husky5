function collision = checkCollision(sourceNode, targetNode, poles, r)
%CHECKOBSTRUCTION Check whether a line given by source and target is
%colliding with any of the poles
%   Assumes poles list as (2,n), and sourceNode, targetNode as 2d row
%   vectors (1,2)
%   Also, pole radius is currently hardcoded to 0.1

if isempty(poles)
    collision = 0 ;
    return
end

lineSeg = [sourceNode; targetNode];
% assume poles in (2,n)
% TODO check radius
d = diff(lineSeg,1,1);
f = lineSeg(1,:) - poles';

a = d*d'; % length 1 (line segment)
b = 2*sum(f .* d, 2); % length n
c = sum(f.^2,2)-r^2; % length n

discriminant = b.^2 - 4*a*c;

potCollision = discriminant > 0;


%
% TODO: possibly refactor, default to collision, for sake of safety, and
% only change to 0?
if ~any(potCollision)
    collision = 0;
else
    discriminant = sqrt(discriminant);
    t1 = (-b(potCollision) - discriminant(potCollision))/(2*a);
    t2 = (-b(potCollision) + discriminant(potCollision))/(2*a);
    % 3x HIT cases:
    %          -o->             --|-->  |            |  --|->
    % Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit)
    % (hit between 0, 1)
    if any((t1 >= 0 & t1 <= 1) | (t2 >= 0 & t2 <= 1))
        % first check tests impale, poke, second checks exit wound
        collision = 1;
    else
        collision = 0;  
    end
end


end

