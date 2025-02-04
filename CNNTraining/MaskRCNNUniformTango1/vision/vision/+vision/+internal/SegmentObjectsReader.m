function out = SegmentObjectsReader(filename)
% SegmentObjectsReader is a custom mat file reader for results of segmentObjects()
% stored as mat-files. This custom reader when used with file datastore returns
% a 1x3 or, 1x4 cell array containing the masks, labels, scores (and boxes) (as generated
% by segmentObjects(detector, ds)) 

    data = load(filename);

    out{1} = data.masks;
    out{2} = data.boxLabel;
    out{3} = data.boxScore;
    if(isfield(data, 'boxes'))
        out{4} = data.boxes;
    end

end
