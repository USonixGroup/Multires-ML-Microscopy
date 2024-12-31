function opts = balanceBoxLabelsDefaultInternalOptions
%Internal Options for balanceBoxLabels.
%
%   'DuplicateBlocks'       Logical to specify whether to duplicate blocks to attain the
%                           exact value of numObservations.
%                           Valid options: true or false.
%
%                           true : The number of block locations will be equal to
%                                  the input numObservations. This is achieved by duplicating
%                                  the balanced block locations.
%                           false: The number of block locations might be lesser than
%                                  the input numObservations.
%
%                           Default: true
%
%   'NumObservationsMultiple'
%                           Multiple factor to oversample blocks with constant number of blocks
%                           for each class.
%                           Valid options: A scalar positive real value.
%
%                           Default: 1.5
%                           With experiments done on xView dataset and hand curated datasets,
%                           with multiple values of 1,1.5,5 and 10, the balanced histogram and
%                           the Coefficient of Variation values look better with value 1.5.
%                           Users can experiment with other values, for their own datasets where
%                           1.5 does not provide a good balance.
%
%   'NumQuadTreeLevels'     Specify the number of quad tree levels for the
%                           sampling algorithm.
%                           Valid options: 'auto', or a positive scalar
%                                           integer.
%
%                           'auto': The sampling algorithm uses the blockSize
%                                   input to determine the deepest quad tree level
%                                   needed.
%
%                           Default: 'auto'
%
%   'BlocksPerWindow'       Specify the number of blocks per window during
%                           block selection from windows (macro blocks).
%                           This is the baseline for the number blocks per
%                           macro block. Increase this value to get more
%                           blocks from the same macro block. Decrease this
%                           value if the box labels are spread across
%                           images, and so different macro blocks can be
%                           used to achieve numObservations per class.
%
%                           Valid options: a positive scalar integer.
%
%                           Default: 20
%
% This function is for internal use only and is likely to change in future
% releases.

% Copyright 2019 The MathWorks, Inc.
    opts.NumQuadTreeLevels = 'auto';
    opts.DuplicateBlocks = true;
    opts.NumObservationsMultiple = 1.5;
    opts.BlocksPerWindow = 20;
end
