%% save the entry from the 

%%% pole generator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
area = [0,5; -4,4]; % first row x-boundaries, second row y-boundaries
%%
nPoles = 10;
poles = zeros(2,nPoles);
poles(1, :) = rand(1, nPoles)*diff(area(1,:)) + area(1,1);
poles(2, :) = rand(1, nPoles)*diff(area(2,:)) + area(2,1);

%
polesTooClose = 1;

closenessTresh = 0.8;
while polesTooClose
    D = squareform(pdist(poles'));
    linIdx = find(triu(D,1) < closenessTresh & triu(D,1) >0);
    if isempty(linIdx)
        polesTooClose = 0;
        continue
    end
    %poles(:, probPoleIdx) = [];
    [probPoleIdx, ~] = ind2sub(size(D), linIdx);
    nNewPoles = numel(probPoleIdx);
    poles(1, probPoleIdx) = rand(1, nNewPoles)*diff(area(1,:)) + area(1,1);
    poles(2, probPoleIdx) = rand(1, nNewPoles)*diff(area(2,:)) + area(2,1);
end

%%
figure(1); clf;
viscircles(poles', 0.1*ones(1,length(poles)))
hold on
targetLine = plot(area(1,[2,2]), area(2,:), 'LineWidth',6)
targetLine.Color(4) = 0.5;
%plot(poles(1,:), poles(2,:), 'o')
axis equal
xlim(area(1,:)+[0,1])
ylim(area(2,:))

lineSeg = [1,1; 3.8,0];
plot(lineSeg(:,1), lineSeg(:,2), 'LineWidth', 4)
%% %%%%%%%%%%%%
%%% collision test
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% assume poles in (2,n)
r = 0.1;
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
    disp('here')
else
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
collision
%% testing
t1 = (-b(potCollision) - discriminant(potCollision))/(2*a);
t2 = (-b(potCollision) + discriminant(potCollision))/(2*a);
idx = (t1 >= 0 & t1 <= 1) | (t2 >= 0 & t2 <= 1);

colPoles = poles(:, potCollision);

plot(colPoles(1, idx), colPoles(2, idx), 'x');
