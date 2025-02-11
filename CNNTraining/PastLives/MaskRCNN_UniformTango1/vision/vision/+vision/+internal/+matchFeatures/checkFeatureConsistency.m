function checkFeatureConsistency(features1, features2)
%checkFeatureConsistency Check if two features are of the same dimension

%   Copyright 2020 The MathWorks, Inc.

%#codegen
coder.internal.errorIf(size(features1, 2) ~= size(features2, 2), ...
    'vision:matchFeatures:featuresNotSameDimension');