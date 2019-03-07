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
r = 0.5;
figure(1); clf;
viscircles(poles', r*ones(1,length(poles)))
hold on
targetLine = plot(area(1,[2,2]), area(2,:), 'LineWidth',6);
targetLine.Color(4) = 0.5;
%plot(poles(1,:), poles(2,:), 'o')
axis equal
xlim(area(1,:)+[0,1])
ylim(area(2,:))

%lineSeg = [1,1; 3.8,0];

lineSeg(1, :) = rand(1, 2)*diff(area(1,:)) + area(1,1);
lineSeg(2, :) = rand(1, 2)*diff(area(2,:)) + area(2,1);

plot(lineSeg(:,1), lineSeg(:,2), 'LineWidth', 4)
% %%%%%%%%%%%%
%%% collision test
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% assume poles in (2,n)
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
collision
% testing
t1 = (-b(potCollision) - discriminant(potCollision))/(2*a);
t2 = (-b(potCollision) + discriminant(potCollision))/(2*a);
idx = (t1 >= 0 & t1 <= 1) | (t2 >= 0 & t2 <= 1);

colPoles = poles(:, potCollision);

plot(colPoles(1, idx), colPoles(2, idx), 'x');

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% route planning
%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%
state = [0,0,0];
rPlan = plan(state, area);
%
tic
rPlan.buildTreeStar(poles);
toc
%%
tic
rPlan.Astar([2, -1, 0])
toc
%%
%
%
figure(4); clf;
viscircles(poles', rPlan.radius*ones(1,length(poles)));
hold on
targetLine = plot(area(1,[2,2]), area(2,:), 'LineWidth',6);
targetLine.Color(4) = 0.5;
%plot(poles(1,:), poles(2,:), 'o')
axis equal
xlim(area(1,:)+[0,1])
ylim(area(2,:))

scatter(rPlan.tree.points(:,1), rPlan.tree.points(:,2), 10, 'filled');
[I,J] = ind2sub(size(rPlan.tree.adj), find(triu(rPlan.tree.adj,1)));
x = [rPlan.tree.points(I, 1)'; rPlan.tree.points(J,1)'];
y = [rPlan.tree.points(I, 2)'; rPlan.tree.points(J,2)'];
line(x, y, 'Color', [0.4 0.4 0.4 0.4]);
plot(rPlan.target.coords(1), rPlan.target.coords(2), 'o')

%
srcidx = rPlan.path(1:end-1);
tgtidx = rPlan.path(2:end);
x = [rPlan.tree.points(srcidx, 1)'; rPlan.tree.points(tgtidx,1)'];
y = [rPlan.tree.points(srcidx, 2)'; rPlan.tree.points(tgtidx,2)'];
line(x, y, 'Color', [0.4 0.4 0.4 1], 'LineWidth', 3);

%%


%%
% write a thing robot integration, while loop with long pause, 
% grabbing stuff from sensors, 
% run slam
% run planner
% display poles, tree (faint), and path, and robot as point
% display robot and poles, maybe with confidence circle?, lidar bits
% display images,

% simple controller test, give single goal, with time limit for how long
% the signal is non-zero
%% %%%%%%%%%%%%%%%%%%%%%%%%
% wheel odom tests
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n = 10;
-wheel_od{n}.destination_timestamp+ wheel_od{n}.source_timestamp
%%
for i = 1:5
wheel(i)=wheel_od{i}
end

%%
bufferLength = 10;

i = mod(counter, buggerLength);
wheel_str(i) = getodom()% whatever

odomIdx = interp1([wheel_str.source_timestamp], 1:10, scan.timestamp, 'nearest');
 
