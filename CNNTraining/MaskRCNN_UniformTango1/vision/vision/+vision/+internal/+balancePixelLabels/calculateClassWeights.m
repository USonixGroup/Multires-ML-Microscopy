function classWeights = calculateClassWeights(populationCounts)
% calculateClassWeights Calculate class weights from population counts

%   Copyright 2020 The MathWorks, Inc.

populationFrequency = populationCounts ./ sum(populationCounts(:));
classWeights = 1 ./ populationFrequency;

end