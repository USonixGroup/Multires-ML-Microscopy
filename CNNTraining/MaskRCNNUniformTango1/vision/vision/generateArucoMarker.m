function imgs = generateArucoMarker(markerFamily, ids, markerSize, options)

%   Copyright 2023-2024 The MathWorks, Inc.

    arguments
        markerFamily (1,1) string
        ids (:,1) {mustBeNumeric, mustBeInteger}
        markerSize (1,1) {mustBeNumeric, mustBeInteger, mustBePositive}
        options.NumBorderBits (1,1) {mustBeNumeric, mustBeInteger, mustBePositive} = 1
    end
    
    % Perform additional validation on the input family and ids.
    validFamily = validateMarkerFamily(markerFamily);
    validateMarkerIDs(validFamily, ids);

    numMarkers = length(ids);
    imgs = zeros(markerSize, markerSize, numMarkers, "uint8");
    for i = 1:numMarkers
        imgs(:,:,i) = generateAruco(char(validFamily), ids(i), markerSize, options.NumBorderBits);
    end
end

%--------------------------------------------------------------------------
function validFamily = validateMarkerFamily(markerFamily)
    validFamily = validatestring(markerFamily, vision.internal.supportedArucoMarkerFamilies);
end

%--------------------------------------------------------------------------
function validateMarkerIDs(familyName, ids)
    
    nameParts = strsplit(familyName, "_");

    if strcmpi(nameParts(end),"ORIGINAL")
        familySize = 1024;
    else
        familySize = str2double(nameParts(end));
    end
    
    invalidIDs = ids < 0 | ids >= familySize;
    if any(invalidIDs)
        error(message("vision:aruco:invalidMarkerID", familyName, familySize-1));
    end
end