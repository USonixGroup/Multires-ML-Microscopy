function classifier = trainImageCategoryClassifier(imgSet, bag, varargin)

% Copyright 2014-2023 MathWorks, Inc.

if nargin > 2
    [varargin{:}] = convertStringsToChars(varargin{:});
end

vision.internal.requiresStatisticsToolbox(mfilename);

classifier = imageCategoryClassifier.create(imgSet, bag, varargin{:});

