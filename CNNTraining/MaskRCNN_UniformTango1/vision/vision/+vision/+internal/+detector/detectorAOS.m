function [aos, ap, similarity, precision, recall] = ...
    detectorAOS(Y, thetaDiff, numExpected, scores, returnNaNforMissingClass)
% Compute average orientation similarity and average precision for
% evaluating rotated bounding box detections. 
%
% Inputs:
% -------
% Y           - an M-by-1 vector, where M is the number of detections. Y
%               values greater than 0 indicate a true positive. Values <= 0
%               are false positives.
%
% thetaDiff   - an M-by-1 vector the same size as Y. Each element is the
%               difference between the predicted rotation and the ground
%               truth rotation angle. thetaDiff(Y <= 0) are equal to NaN.
%
% numExpected - The number of expected detections. This is the number of
%               ground truth objects. 
%
% scores      - an M-by-1 vector the same size as Y. This is typically the
%               detection score associated with each detection.
%
% returnNaNforMissingClass - a logical scalar specifying whether to return
%                            NaNs for missing classes.
%

% Copyright 2019 The Mathworks, Inc.

% Reference
% ---------
% Geiger, Andreas, Philip Lenz, and Raquel Urtasun. "Are we ready for
% autonomous driving? the kitti vision benchmark suite." 2012 IEEE
% Conference on Computer Vision and Pattern Recognition. IEEE, 2012.

arguments
    Y
    thetaDiff
    numExpected (1,1) {mustBeNumeric, mustBeInteger, mustBeScalarOrEmpty}
    scores
    returnNaNforMissingClass (1,1) {logical} = false
end

function iValidateInputSizes(Y, thetaDiff, scores)
    if (size(Y,1) ~= size(thetaDiff,1)) || (size(Y,1) ~= size(scores,1))
        error("vision:maskrcnn:incorrectInputSize",...
            "''Y'', ''thetaDiff'' and ''scores'' must all be M-by-1 vectors.")
    end
end
iValidateInputSizes(Y, thetaDiff, scores);

if (returnNaNforMissingClass && isempty(Y) && numExpected == 0)
    ap = NaN;
    precision = NaN;
    recall = NaN;
    similarity = NaN;
    aos = NaN;
    return;
end

if (isempty(Y) || numExpected == 0)
    ap = 0;
    aos = 0;
    precision = 1;
    recall = 0;
    similarity = 1;
    return;
end

[~, idx] = sort(scores, 'descend');
Y = Y(idx);
thetaDiff = thetaDiff(idx);

tp = Y > 0;
fp = Y <= 0;

tp = cumsum(tp);
fp = cumsum(fp);

precision = tp ./ (tp + fp);
recall = tp ./ numExpected;

similarity = cumsum(( 1+cosd(thetaDiff))/2 .* (Y > 0) ) ./ (tp + fp);

% Compute AP and AOS at the 11 discrete recall levels.
recallLevels = 0:0.1:1;
ap = 0;
aos = 0;
for ri = 1:numel(recallLevels)
    
    idx = recall >= recallLevels(ri);
   
    p = max(precision(idx));
    s = max(similarity(idx));
    
    if isempty(p)
        p = 0;
        s = 0;
    end
    
    ap  = ap + p;
    aos = aos + s;
    
end
ap = ap/11;
aos = aos/11;

% By convention, start precision and orientation similarity at 1 and recall
% at 0.
precision  = [1; precision];
similarity = [1; similarity];
recall     = [0; recall];

end