function tf = isValidPointObj(points)
% isValidPointObj returns true if points are a MSERRegions or a
% FeaturePoints object

%   Copyright 2014-2020 The MathWorks, Inc.

%#codegen
tf = isa(points, 'MSERRegions') || ...
    isa(points, 'vision.internal.MSERRegions_cg') || ...
    isa(points, 'vision.internal.FeaturePointsImpl');
