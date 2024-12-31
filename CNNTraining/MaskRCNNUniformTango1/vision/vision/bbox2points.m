function points = bbox2points(bbox)

%   Copyright 2013-2023 The MathWorks, Inc.

%#codegen

numBboxes = size(bbox, 1);
sizeBboxes = size(bbox, 2);
checkInput(bbox, sizeBboxes);

if sizeBboxes == 4
    % Bounding boxes are axis-aligned rectangle bounding boxes.
    points = zeros(4, 2, numBboxes, "like", bbox);

    % upper-left
    points(1, 1, :) = bbox(:, 1);
    points(1, 2, :) = bbox(:, 2);
    
    % upper-right
    points(2, 1, :) = bbox(:, 1) + bbox(:, 3);
    points(2, 2, :) = bbox(:, 2);
    
    % lower-right
    points(3, 1, :) = bbox(:, 1) + bbox(:, 3);
    points(3, 2, :) = bbox(:, 2) + bbox(:, 4);
    
    % lower-left
    points(4, 1, :) = bbox(:, 1);
    points(4, 2, :) = bbox(:, 2) + bbox(:, 4);
else
    % Bounding boxes are rotated rectangle bounding boxes.
    points = zeros(4, 2, numBboxes, 'like', bbox);

    r = bbox(:, 5);
    u = [bbox(:,3)/2 bbox(:,3)/2].* [ cosd(r) sind(r)];
    v = [bbox(:,4)/2 bbox(:,4)/2].* [-sind(r) cosd(r)];

    % upper-left (as seen at 0 degrees of rotation).
    points(1, 1, :) = bbox(:, 1) - u(:, 1) - v(:, 1);
    points(1, 2, :) = bbox(:, 2) - u(:, 2) - v(:, 2);

    % upper-right (as seen at 0 degrees of rotation).
    points(2, 1, :) = bbox(:, 1) + u(:, 1) - v(:, 1);
    points(2, 2, :) = bbox(:, 2) + u(:, 2) - v(:, 2);

    % lower-right (as seen at 0 degrees of rotation).
    points(3, 1, :) = bbox(:, 1) + u(:, 1) + v(:, 1);
    points(3, 2, :) = bbox(:, 2) + u(:, 2) + v(:, 2);

    % lower-left (as seen at 0 degrees of rotation).
    points(4, 1, :) = bbox(:, 1) - u(:, 1) + v(:, 1);
    points(4, 2, :) = bbox(:, 2) - u(:, 2) + v(:, 2);
end

function checkInput(bbox, sizeBboxes)
if sizeBboxes == 5
    posSize = 5;
    coder.internal.errorIf(~isfloat(bbox), 'vision:bbox:InvalidRotatedDataType');
else
    posSize = 4;
end
validateattributes(bbox, ...
    {'int16', 'uint16', 'int32', 'uint32', 'single', 'double'}, ...
    {'real', 'nonsparse', 'nonempty', 'finite', 'size', [NaN, posSize]}, ...
    'bbox2points', 'bbox'); %#ok<EMCA>

validateattributes(bbox(:, [3,4]), {'numeric'}, ...
    {'>=', 0}, 'bbox2points', 'bbox(:,[3,4])'); %#ok<EMCA>


