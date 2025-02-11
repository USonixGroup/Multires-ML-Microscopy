function boxLabels = bboxcropUsingBlockSet(boxLabels,bSet,overlapThreshold)
%bboxcropUsingBlockSet Crop box labels using the blockLocationSet object.
%
%   boxLabels = bboxcropUsingBlockSet(boxLabels,bSet) crops the box labels
%   using the information in the blockLocationSet object, bSet. The output
%   is a M-by-2 cell array containing the cropped boxes, where M is the
%   height of the ImageNumber property of blockLocationSet object.
%   bboxcrop in this function uses an OverlapThreshold of 0.1.
%
%   This function is for internal use only and is likely to change in future
%   releases.
%   See also boxLabelDatastore, blockLocationSet.

%   Copyright 2019-2021 The MathWorks, Inc.

    if isempty(boxLabels) || isempty(bSet)
        return;
    end
    bboxes = boxLabels(:,1);
    labels = boxLabels(:,2);
    h = size(bSet.ImageNumber,1);
    boxLabels = cell(h,2);
    for ii = 1:h
        blockOrigin = get2DBlockOrigin(bSet.BlockOrigin(ii,:));
        imNum = bSet.ImageNumber(ii);

        % blockLocationSet has BlockSize as [numrows, numcols]
        win = [blockOrigin,bSet.BlockSize([2,1])];

        [boxLabels{ii,1}, indices] = bboxcrop(bboxes{imNum},win,...
            'OverlapThreshold',overlapThreshold);
        boxLabels{ii,2} = labels{imNum}(indices);
    end
end

function blockOrigin = get2DBlockOrigin(blockOrigin)
    if any(blockOrigin(3:end) ~= 1)
        error(message('vision:boxLabelDatastore:invalidNDBlockOrigin'));
    end

    % blockOrigin can be n-D, select just the 2-D spatial origin values.
    blockOrigin = blockOrigin(1:2);
end
