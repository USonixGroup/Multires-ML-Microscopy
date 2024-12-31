function matchedValue = checkMetric(value, fileName)
%checkMetric Check the metric used in feature matching

%   Copyright 2020 The MathWorks, Inc.

%#codegen
list = {'ssd', 'normxcorr', 'sad'};
validateattributes(value, {'char', 'string'}, {'nonempty'}, fileName, 'Metric');
matchedValue = validatestring(value, list, fileName, 'Metric');