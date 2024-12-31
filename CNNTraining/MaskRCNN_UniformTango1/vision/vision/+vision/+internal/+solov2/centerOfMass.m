function [centerH, centerW] = centerOfMass(mask)

[h, w] = ndgrid(1:size(mask, 1), 1:size(mask, 2));

center = mean([h(logical(mask)), w(logical(mask))]);

centerH = center(1);
centerW = center(2);
