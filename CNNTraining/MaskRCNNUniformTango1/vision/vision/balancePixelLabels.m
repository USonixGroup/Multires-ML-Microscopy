function locationSet = balancePixelLabels(bigLabeledImages, varargin)

validateattributes(bigLabeledImages, ["bigimage", "blockedImage"], {'nonempty', 'vector'}, mfilename, "bigLabeledImages", 1)

if isa(bigLabeledImages,'blockedImage')
    narginchk(3,12);
    blockSize = varargin{1};
    numObservations = varargin{2};
    locationSet = vision.internal.balancePixelLabels.blockedImages(bigLabeledImages, blockSize, numObservations, varargin{3:end});
else
    narginchk(4,6);
    levels = varargin{1};
    blockSize = varargin{2};
    numObservations = varargin{3};
    locationSet = vision.internal.balancePixelLabels.bigImages(bigLabeledImages, levels, blockSize, numObservations, varargin{4:end});
end
end

%   Copyright 2019-2023 The MathWorks, Inc.
