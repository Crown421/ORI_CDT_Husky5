function poles = generatePoles(nPoles, area, seed)
%GENERATEPOLES Summary of this function goes here
%   Detailed explanation goes here

if exist('seed','var')
    rng(seed)
end

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

end

