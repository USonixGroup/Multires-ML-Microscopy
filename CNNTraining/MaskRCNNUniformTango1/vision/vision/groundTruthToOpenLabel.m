%groundTruthToOpenLabel Convert groundTruth object to OpenLABEL JSON file
%
%   groundTruthToOpenLabel(gTruth, jsonFilename) converts input groundTruth
%   object, gTruth, to ASAM OpenLABEL format and writes the result to
%   JavaScript Object Notation (JSON) file called jsonFilename. The .json
%   extension is automatically appended to the input filename string.
%
%   groundTruthToOpenLabel(..., PrettyPrint=true) adds indentations to the 
%   output file to make it easier to read. By default, PrettyPrint is set 
%   to false to make the output file size smaller.    
%
%   Notes:
%   ------
%   - ASAM OpenLABEL specification can be found here: 
%     https://www.asam.net/index.php?eID=dumpFile&t=f&f=4566&token=9d976f840af04adee33b9f85aa3c22f2de4968dd
%   - Not all aspect of the data stored in groundTruth object can be represented in 
%     OpenLABEL format. The following customizations were applied to
%     OpenLABEL specification to compensate for any differences and
%     ambiguities in the ASAM specification.
%     - For an image collection used by Image Labeler each object in the
%       OpenLABEL "object_data" section of the JSON uses a "text" field to point
%       to images on the file system.
%     - Pixel label images are not embedded into the "image" OpenLABEL
%       object but instead are referenced in the "image" object using the 
%       "text" field in the "object_data_pointer" section of the JSON file.
%     - Projected cuboids from groundTruth object are stored as eight
%       points in a poly2d object since there is no equivalent 
%       representation in the OpenLABEL specification. These special poly2d
%       objects are tagged with a boolean field "matlab_projected_cuboid"
%       set to true.
%     - Information stored in gTruth.LabelDefinitions such as Group,
%       Description, LabelColor is placed in the custom "metadata" section 
%       of the OpenLABEL JSON file under "matlab_label_definitions" field.
%     - Pixel label values to class name correspondences are stored in the
%       custom "matlab_label_definitions" field of the JSON file.
%     - Closed polygons contain "hierarchy" field with the top-most polygon
%       listed first, index zero.  In the case of overlapping polygons, the
%       top polygons with lower index numbers cover the polygons below.
%
%   Class support
%   -------------
%   gTruth must be a groundTruth object containing image labels.  jsonFilename
%   must be a string.
%
%   Example
%   -------
%   % Add the folder containing images to the MATLAB path.
%   imageDir = fullfile(matlabroot, "toolbox", "vision", "visiondata", "vehicles");
%   addpath(imageDir);
%
%   % Load groundTruth data containing labeled vehicles. It was produced 
%   % by the Image Labeler app. Note that the ground truth from the Video Label app 
%   % can also be converted to OpenLABEL format.
%   data = load("vehicleTrainingGroundTruth.mat");
%   gTruth = data.vehicleTrainingGroundTruth;
%
%   % Convert the groundTruth object to JSON file in ASAM OpenLABEL format
%   groundTruthToOpenLabel(gTruth,"vehicleGtruthOpenLABEL", PrettyPrint=true);
%
%   % Inspect the resulting JSON file
%   edit vehicleGtruthOpenLABEL.json
%
%   See also: groundTruth, jsonencode, imageLabeler

% Copyright 2023 The MathWorks, Inc.

function groundTruthToOpenLabel(gTruth, jsonFilename, options)

% Parse function inputs
arguments
    gTruth (1,1) groundTruth
    jsonFilename (1,1) string
    options.PrettyPrint (1,1) logical = false
end

% To be very thorough, check for hand-created ground truth with Cuboid 
% label type. This would never be produced by Image Labeler
hasCuboidType = ismember("Cuboid", gTruth.LabelDefinitions.Type);

if hasCuboidType
    error(message("vision:gtruthOpenLabel:unsupportedGtruthObject"));
end

% Detect if we are processing gTruth from Video Labeler. If the data came 
% from Video Labeler, it would be a "timetable".
isVideo = isa(gTruth.LabelData, "timetable");

% String prefix used for numeric struct fields which are not permitted in
% MATLAB. It's a string that's later removed from the JSON file. It's
% simply an artifact of how jsonencode and MATLAB structures work.
numPrefix = "xyxyxyx";

%% Create initial struct with OpenLABEL spec version 
root  = getRootStruct; % start with most basic OpenLABEL metadata

% From this point onwards, we keep appending to the root

%% Add custom metadata
metadataStruct = getCustomMetadata(gTruth);
root.openlabel.metadata = appendStruct(root.openlabel.metadata, metadataStruct);

%% Formulate core Open LABEL struct for images or for video
root = processData(isVideo, root, gTruth, numPrefix);

% If there were no ROIs, remove the objects section
if isempty(root.openlabel.objects)
    root.openlabel = rmfield(root.openlabel, "objects");
end

%% Convert the MATLAB struct to the final JSON format
% Convert our struct to JSON format
jsonText = jsonencode(root, PrettyPrint=options.PrettyPrint);

%% Write OpenLABEL encoded ground truth to a file
jsonText = strrep(jsonText, numPrefix, ""); % remove the prefix to get clean numeric output
writeJsonFile(jsonText, jsonFilename);

end

%--------------------------------------------------------------------------
function root = processData(isVideo, root, gTruth, numPrefix)

%% Create streams section for video
if isVideo
    streamsStruct = getStreams(gTruth);
    root.openlabel.streams = streamsStruct;
end

% Initialize required fields
root.openlabel.objects = [];
if isVideo
    root.openlabel.frames  = [];
end

roiID = 0; % counter for ROIs

% Let's begin with bounding boxes
[objects,frames,nextRoiID] = getBoundingBoxes(isVideo,gTruth,roiID,numPrefix);
root.openlabel.objects = objects;
if isVideo
    root.openlabel.frames  = frames;
end

% Rotated bounding boxes
[objects,frames,nextRoiID] = getRotatedBoundingBoxes(isVideo,gTruth,nextRoiID,numPrefix);
root.openlabel.objects = appendStruct(root.openlabel.objects, objects);
if isVideo
    root.openlabel.frames  = mergeFrames(root.openlabel.frames, frames);
end

% Append points
[objects,frames,nextRoiID] = getPoints(isVideo,gTruth,nextRoiID,numPrefix);
root.openlabel.objects = appendStruct(root.openlabel.objects,objects);
if isVideo
    root.openlabel.frames  = mergeFrames(root.openlabel.frames, frames);
end

% Append lines
isPolyClosed = false;
[objects,frames,nextRoiID] = getPolyLines(isVideo,gTruth,nextRoiID,isPolyClosed,numPrefix);
root.openlabel.objects = appendStruct(root.openlabel.objects,objects);
if isVideo
    root.openlabel.frames  = mergeFrames(root.openlabel.frames, frames);
end

% Append projected cuboids
% NOTE: projected cuboid isn't a format supported by OpenLABEL, so instead,
%       we'll store it as a collection of points packed into a poly-line
[objects,frames,nextRoiID] = getProjectedCuboid(isVideo,gTruth,nextRoiID,numPrefix);
root.openlabel.objects = appendStruct(root.openlabel.objects,objects);
if isVideo
    root.openlabel.frames  = mergeFrames(root.openlabel.frames, frames);
end

% Append polygons
isPolyClosed = true;
[objects,frames,nextRoiID] = getPolyLines(isVideo,gTruth,nextRoiID,isPolyClosed,numPrefix);
root.openlabel.objects = appendStruct(root.openlabel.objects,objects);
if isVideo
    root.openlabel.frames  = mergeFrames(root.openlabel.frames,frames);
end

% Append pixel labels
[objects, frames] = getPixelLabels(isVideo,gTruth,nextRoiID,numPrefix);
root.openlabel.objects = appendStruct(root.openlabel.objects,objects);
if isVideo
    root.openlabel.frames  = mergeFrames(root.openlabel.frames,frames);
end

% If at this stage, frames section is empty, it must be removed since
% schema validation will fail. Frames section can't be empty
if isVideo
    if isempty(root.openlabel.frames)
        root.openlabel = rmfield(root.openlabel, "frames");
    end
end

%% Add OpenLABEL contexts (per-scene context data, aka scene labels)
contexts = getSceneLabels(isVideo,gTruth, numPrefix);
if ~isempty(contexts)
    root.openlabel.contexts = contexts;
end

end

%--------------------------------------------------------------------------
function streamsStruct = getStreams(gTruth)

% We'll use "Video" as UID, but anything here should work well
streamsStruct.Video.type = "camera";
streamsStruct.Video.uri = fixPathDelimiter(gTruth.DataSource.Source); % file location
streamsStruct.Video.description = "Video data labeled by the Video Labeler app";

% Note that the stream_properties section can be added to the stream and it can
% contain things such as camera calibration parameters.  Currently, 
% Video Labeler does not facilitate handling of camera calibration data.

end

%--------------------------------------------------------------------------
% This routine populates the custom metadata which adds additional
% information useful to MathWorks and the MathWorks labeler apps. This
% includes:
%    * version
%    * LabelDefinitions of the groundTruth object
%--------------------------------------------------------------------------
function metadataStruct = getCustomMetadata(gTruth)

metadataStruct.mathworks_schema_version = "1.0.0";
metadataStruct.matlab_release = matlabRelease; % Embed MATLAB release information

if ~gTruth.hasValidDataSource
    error(message("vision:gtruthOpenLabel:invalidSourceObject"));    
end

if isa(gTruth.DataSource.Source,"matlab.io.datastore.ImageDatastore")
    metadataStruct.matlab_read_function.description = "Read function used by the ImageDatastore";
    metadataStruct.matlab_read_function.val = func2str(gTruth.DataSource.Source.ReadFcn);
end

% Go over class names
numLabels = height(gTruth.LabelDefinitions);
if numLabels ~= 0
    metadataStruct.matlab_label_definitions = {};
    tableColumnNames = gTruth.LabelDefinitions.Properties.VariableNames;

    % For efficiency, cache information about which fields are available
    hasLabelColor   = ismember("LabelColor", tableColumnNames);
    hasPixelLabelID = ismember("PixelLabelID", tableColumnNames);
    hasGroup        = ismember("Group", tableColumnNames);
    hasDescription  = ismember("Description", tableColumnNames); 
    hasHierarchy    = ismember("Hierarchy", tableColumnNames);

end

for labelIdx = 1:numLabels   % write out extra information for every label definition

    tempNode.description = "Custom metadata holding each label definition similarly to groundTruth's LabelDefinitions property"; 
    tempNode.name = string(gTruth.LabelDefinitions.Name(labelIdx));

    % name is repeated for consistency to replicate the LabelDefinitions
    % table as closely as possible
    tempNode.val.Name = string(gTruth.LabelDefinitions.Name(labelIdx));    
    tempNode.val.Type  = string(gTruth.LabelDefinitions.Type(labelIdx));

    if hasLabelColor
        tempNode.val.LabelColor = gTruth.LabelDefinitions.LabelColor(labelIdx,:);
    end

    if hasPixelLabelID
        temp = gTruth.LabelDefinitions.PixelLabelID(labelIdx,:);
        tempNode.val.PixelLabelID = temp{:};
    end
        
    if hasGroup
        tempNode.val.Group = string(gTruth.LabelDefinitions.Group(labelIdx));
    end

    if hasDescription
        tempNode.val.Description = string(gTruth.LabelDefinitions.Description(labelIdx));
    end

    if hasHierarchy
        temp = gTruth.LabelDefinitions.Hierarchy{labelIdx};
        if isstruct(temp) % avoid processing pure empty entries
            temp = includeClassInDefaultValues(temp);
        end
        tempNode.val.Hierarchy = temp;
    end   

    % fill the cell with the assembled value
    metadataStruct.matlab_label_definitions{end+1} = tempNode;

end

    function structOut = includeClassInDefaultValues(structIn)
        % This function recursively scans for 'DefaultValue' fields 
        % in the input structure and converts them to strings that besides
        % the value also specify class to that, for example, logical([]) 
        % and double([]) are distinct.

        % Initialize the output structure
        structOut = structIn;

        % Iterate over each field of the structure
        fields = fieldnames(structIn);
        for i = 1:length(fields)
            fieldName = fields{i};
            fieldValue = structIn.(fieldName);

            % Check if the current field is 'DefaultValue'
            if strcmp(fieldName, 'DefaultValue')
                % Convert char to string if it is a char array
                structOut.(fieldName) = mat2str(fieldValue,'class');
            elseif isstruct(fieldValue)
                % If the field is a structure, recursively call this function
                if isscalar(fieldValue)
                    % Single struct
                    structOut.(fieldName) = includeClassInDefaultValues(fieldValue);
                else
                    % Array of structs
                    for j = 1:numel(fieldValue)
                        structOut.(fieldName)(j) = includeClassInDefaultValues(fieldValue(j));
                    end
                end
            end
        end
    end

end

%--------------------------------------------------------------------------
function [objects, frames, roiID] = getBoundingBoxes(isVideo, gTruth, startingRoiID, numPrefix)

[objects, frames, roiID] = encodeObject(isVideo, gTruth, startingRoiID, ...
    numPrefix, "bbox", "Rectangle", @bbox2openlabel);

end

%--------------------------------------------------------------------------
function [objects, frames, roiID] = getRotatedBoundingBoxes(isVideo, gTruth, startingRoiID, numPrefix)

[objects, frames, roiID] = encodeObject(isVideo,gTruth, startingRoiID, ...
    numPrefix, "rbbox", "RotatedRectangle", @rbbox2openlabel);

end

%--------------------------------------------------------------------------
function [objects,frames,roiID] = getPoints(isVideo,gTruth,startingRoiID,numPrefix)

[objects,frames,roiID] = encodeObject(isVideo,gTruth, startingRoiID, ...
    numPrefix, "point2d", "Point", @points2openlabel);

end

%--------------------------------------------------------------------------
function [objects,frames,roiID] = getPolyLines(isVideo,gTruth,startingRoiID,...
    isPolyClosed,numPrefix)

if isPolyClosed
    mwType = "Polygon";
else
    mwType = "Line";
end

[objects,frames,roiID] = encodeObject(isVideo,gTruth,startingRoiID, ...
    numPrefix, "poly2d", mwType, @points2openlabel);

end

%--------------------------------------------------------------------------
% There is no equivalent of projected cuboid in OpenLABEL. We'll store it
% as 8 points in a poly-line. Order of encoding will be: foreground 
% rectangle points starting with the upper left corner and then background 
% rectangle points also starting with the upper left corner.
%
% "name" field will indicate that it's a projected cuboid.
%--------------------------------------------------------------------------
function [objects,frames,roiID] = getProjectedCuboid(isVideo,gTruth,startingRoiID,numPrefix)

[objects,frames,roiID] = encodeObject(isVideo, gTruth, startingRoiID, ...
    numPrefix, "poly2d", "ProjectedCuboid", @projcuboid2openlabel);

end

%--------------------------------------------------------------------------
function [objects,frames,roiID] = getPixelLabels(isVideo, ...
    gTruth,startingRoiID, numPrefix)

objects = [];
frames  = [];

% extract logical index to all rows in label definition table corresponding to pixel labels
pixLabelRowIdx  = gTruth.LabelDefinitions.Type == 'PixelLabel';

if ~any(pixLabelRowIdx)
    return
end

if isVideo
    numImages = size(gTruth.LabelData, 1);
else
    imageFilenames  = getImageFilenames(gTruth);
    numImages       = numel(imageFilenames);
end

roiID = startingRoiID;

for imgIdx = 1:numImages
    frameIdx = imgIdx-1; 

    % Encode pixel label PNG filenames; pix-label PNG filenames are 
    % listed in the gTruth.LabelData column named "PixelLabelData"
    pixFilename = gTruth.LabelData{imgIdx,"PixelLabelData"}{:}; 

    % If the row is non-empty, it means that the pixel labels exist for 
    % this image
    if ~isempty(pixFilename)

            % name and type of object
            labelName = "PixelLabel";
            objects.(numPrefix+roiID).name = "labels";
            objects.(numPrefix+roiID).type = labelName;
            
            % write out pixel label filename    
            tmp.name = "semantic mask";
            tmp.val  = fixPathDelimiter(pixFilename);
            if isVideo
                frames.(numPrefix+frameIdx).objects.(numPrefix+roiID).object_data.text = {tmp};
            else
                objects.(numPrefix+roiID).object_data.text = {tmp};
            end
            
            % pointer to data
            objects.(numPrefix+roiID).object_data_pointers.labels.type = "text";                      
            if isVideo
                % We could have an array but since each frame is labeled
                % independently, we are marking up an interval corresponding 
                % to one frame.  This may change in the future.
                tmp.frame_start = frameIdx;
                tmp.frame_end = frameIdx;
                objects.(numPrefix+roiID).object_data_pointers.labels.frame_intervals = {tmp};

                frames.(numPrefix+frameIdx).frame_properties.time_stamp = string(gTruth.LabelData.Time(imgIdx));
            else
                objects.(numPrefix+roiID).object_data_pointers.labels.frame_intervals = [];

                objects.(numPrefix+roiID).object_data_pointers.labels.uri = ...
                    fixPathDelimiter(imageFilenames{imgIdx}); % write image filename
            end

            % increment global ROI ID
            roiID = roiID + 1;
    end
end

end

%--------------------------------------------------------------------------
function contexts = getSceneLabels(isVideo, gTruth, numPrefix)

contexts = [];

% rectangle is a bounding box
sceneRowIdx = gTruth.LabelDefinitions.Type == 'Scene';           % which definitions correspond to scene labels?
numClasses  = sum(sceneRowIdx);                                  % how many scene label classes are there?
sceneNames  = string(gTruth.LabelDefinitions.Name(sceneRowIdx)); % what are the class names associated with rect ROI?

if isVideo
    numImages = size(gTruth.LabelData, 1);
else
    imageFilenames = getImageFilenames(gTruth); % list of all image filenames
    numImages = numel(imageFilenames);          % how many images do we have?
end

sceneLabelID = 0;

for imgIdx = 1:numImages   % loop over all images
    frameIdx = imgIdx-1;

    for classIdx = 1:numClasses % go over all possible scene labels
        sceneLabelName = sceneNames(classIdx);
        sceneLabelVal  = gTruth.LabelData{imgIdx, sceneLabelName};

        % name and type of object
        contexts.(numPrefix+sceneLabelID).name = sceneLabelName + sceneLabelID;
        contexts.(numPrefix+sceneLabelID).type = sceneLabelName;
        
        % scene markup
        temp.name = sceneLabelName;
        temp.val  = sceneLabelVal;
        contexts.(numPrefix+sceneLabelID).context_data.boolean = {temp};

        if isVideo
            % We could have an array but since each frame is labeled
            % independently, we are marking up an interval corresponding
            % to one frame.  This may change in the future.
            tmp.frame_start = frameIdx;
            tmp.frame_end = frameIdx;
            % We are writing contexts in the same way as for Image Labeler.
            contexts.(numPrefix+sceneLabelID).frame_intervals = {tmp};
        else
            contexts.(numPrefix+sceneLabelID).frame_intervals = [];
            tmp.name = "Image filename for this context";
            tmp.val  = fixPathDelimiter(imageFilenames{imgIdx}); % Unable to use "uri" field without violating the test against schema
            contexts.(numPrefix+sceneLabelID).context_data.text = {tmp};
        end
                
        sceneLabelID = sceneLabelID + 1;
    end
end

end

%--------------------------------------------------------------------------
% This routine handles encoding of all objects except for pixel labels.
% Scene labels are packed using getSceneLabels function. 
% Explanation for prefixes used for ROI types:
%   * mw - MathWorks type, e.g. Rectangle
%   * ol - OpenLABEL type, e.g. bbox
%--------------------------------------------------------------------------
function [objects, frames, roiID] = encodeObject(isVideo, ...
    gTruth, startingRoiID, numPrefix, olType, mwType, conversionFcn)

objects = [];
frames = [];

% rectangle is a bounding box
roidRowIdx = gTruth.LabelDefinitions.Type == mwType;  % which definitions correspond to mwType of ROI?
numClasses = sum(roidRowIdx);                         % how many classes of mwType are there?
classNames  = string(gTruth.LabelDefinitions.Name(roidRowIdx)); % what are the class names associated with rect ROI?

if isVideo
    numImages = size(gTruth.LabelData, 1);
else
    imageFilenames = getImageFilenames(gTruth); % list of all image filenames
    numImages = numel(imageFilenames);          % how many images do we have?
end

roiID = startingRoiID;

for imgIdx = 1:numImages   % loop over all images
    frameIdx = imgIdx-1;

    for classIdx = 1:numClasses % go over all bbox class names

        className = classNames{classIdx};
        if ~any(contains(gTruth.LabelData.Properties.VariableNames,className))
            % This is for the case when there is a definition but it was 
            % never used to label the data.
            continue; 
        end

        rois  = gTruth.LabelData{imgIdx, className}{:}; % grab all ROIs of mwType for this image

        numRois = getNumRois();
        
        for roiIdx = 1:numRois % loop over all rectangles of class rectNames{classIdx}

            metadata = [];
            roiValue = getRoiValue; % extract single ROI coordinates

            % skip empties
            if isempty(roiValue)
                continue
            end

            roiValue = conversionFcn(roiValue); % convert to OpenLABEL format

            % name and type of object
            objects.(numPrefix+roiID).name = classNames(classIdx) + roiID; % this is optional and could have been left empty
            objects.(numPrefix+roiID).type = classNames(classIdx); % type in OpenLabel holds class of the object: Person, Car, etc.
            
            % roi values, e.g. coordinates for a bounding box
            temp.name = "shape";
            temp.val  = roiValue;
            if olType == "poly2d" % handle polygon case which is different from other types
                temp = encodePoly2d(temp);
            end

            if isVideo
                frames.(numPrefix+frameIdx).objects.(numPrefix+roiID).object_data.(olType) = {temp};
            else                
                objects.(numPrefix+roiID).object_data.(olType) = {temp};
            end

            % data pointer, which is like our label definition in groundTruth
            objects.(numPrefix+roiID).object_data_pointers.shape.type = olType;
            if isVideo
                % We could have an array but since each frame is labeled
                % independently, we are marking up an interval corresponding
                % to one frame.  This may change in the future.
                tmp.frame_start = frameIdx;
                tmp.frame_end = frameIdx;
                objects.(numPrefix+roiID).object_data_pointers.shape.frame_intervals = {tmp};
            else
                objects.(numPrefix+roiID).object_data_pointers.shape.frame_intervals = [];
            end
            
            if isVideo
                frames.(numPrefix+frameIdx).frame_properties.time_stamp = string(gTruth.LabelData.Time(imgIdx));
            else
                % store main image filename for the image where this ROI occurs
                objects.(numPrefix+roiID).object_data_pointers.shape.uri = fixPathDelimiter(imageFilenames{imgIdx});
            end

            % process attributes and sub-labels
            if ~isempty(metadata)
                if isVideo
                    subStruct = ...
                        frames.(numPrefix+frameIdx).objects.(numPrefix+roiID).object_data.(olType);
                else
                    subStruct = objects.(numPrefix+roiID).object_data.(olType);
                end
                className = objects.(numPrefix+roiID).type;
                [subStructOut, objDataPtr] = appendAttributesAndSublabels(subStruct,metadata,...
                    className, gTruth.LabelDefinitions);
                if isVideo
                    frames.(numPrefix+frameIdx).objects.(numPrefix+roiID).object_data.(olType) = subStructOut;
                else
                    objects.(numPrefix+roiID).object_data.(olType) = subStructOut;
                end
                objects.(numPrefix+roiID).object_data_pointers.shape = ...
                    appendStruct(objects.(numPrefix+roiID).object_data_pointers.shape, objDataPtr);
            end
            
            % increment roi UID
            roiID = roiID + 1;
        end
    end
end

    % This function inserts additional content that's unique to encoding
    % poly2d objects, including hierarchy
    function temp = encodePoly2d(temp)
        isPolyClosed = (mwType == "Polygon");
        temp.mode   = "MODE_POLY2D_ABSOLUTE";
        temp.closed = isPolyClosed;
        % Definition of OpenLABEL hierarchy:
        %    https://docs.opencv.org/4.5.2/d9/d8b/tutorial_py_contours_hierarchy.html

        % Labelers encode hierarchy only for closed polygons and
        % the ordering of the polygons in the cell array
        % corresponds to the hierarchy where the first entry is at
        % the top level.
        if isPolyClosed
            % we can just use roiIdx because OpenLABEL uses 0-based indexing
            if roiIdx < numRois
                nextPoly = roiIdx;
            else
                nextPoly = -1;
            end

            prevPoly = roiIdx-2; % -1 for 0-based indexing and -1 for previous

            % The hierarchy is structured as:
            % [next, previous, firstChild, parent].  Since labelers
            % don't encode parent-child relations, the last two
            % elements are unused and are always -1.
            temp.hierarchy = [nextPoly prevPoly -1 -1];
        end
        % Tag a projected cuboid
        if mwType == "ProjectedCuboid"
            temp.matlab_projected_cuboid = true;
        else
            temp.matlab_projected_cuboid = false;
        end
    end

    % This nested function determines number or ROIs
    function numRois = getNumRois
        if ~isstruct(rois) % no attributes or sub-labels present
            if olType == "poly2d"
                if iscell(rois) % multiple polygons or lines
                    numRois = numel(rois);
                elseif mwType == "ProjectedCuboid"
                    % projected cubes are always packed in a matrix
                    numRois = size(rois,1);
                else % projected cuboid
                    % one polygon or line is not wrapped in a cell array
                    % but an array of them is wrapped (that's the top
                    % case)
                    numRois = 1; 
                end
            else
                % non-poly ROIs are packed in simple matrices
                numRois = size(rois, 1);
            end
        else % has attributes or sub-labels
            numRois = length(rois); % column vector for video and row vector for images
        end
    end

    % Extract numerical quantities describing each ROI
    function roiValue = getRoiValue
        if ~isstruct(rois) % pure ROI without additional hierarchy
            if olType == "poly2d"
                if iscell(rois) % multi-line
                    roiValue = rois{roiIdx};
                elseif mwType == "ProjectedCuboid"
                    roiValue = rois(roiIdx,:);
                else
                    % Handle single line or polygon which are not packed 
                    % in a cell array
                    roiValue = rois;
                end
            else
                roiValue = rois(roiIdx,:);
            end
        else % holds ROIs and other content such as attributes
            if ~isempty(rois)
                metadata = rois(roiIdx);
                roiValue = metadata.Position;
                if iscell(roiValue)
                    assert(false, "Malformed groundTruth object that was not produced by the Image Labeler");
                end
            else
                roiValue = []; % indicate an empty row to skip
            end
        end
    end

end

%--------------------------------------------------------------------------
function [structOut, objDataPtr] = ...
    appendAttributesAndSublabels(structIn,metadata,...
    className, labelDefinitions, varargin)
%--------------------------------------------------------------------------

fn = fieldnames(metadata);
structOut = structIn; % assign input to output and start modifying it
objDataPtr = [];

% This roiIdx can only be pushed by sub-label attributes (code below). 
% It is passed in only when there are multiple sub-ROIs of the same type.
if nargin == 5 % we are dealing with sub-label attribute
    roiIdx = varargin{1};
else % attribute on primary ROI
    roiIdx = 1;
end

% process all attributes
for i=1:numel(fn)
    field = string(fn{i});
    if field ~= "Position" % We already parsed position earlier
        value  = metadata.(field);        
        isAttribute = ~isstruct(value);

        if isAttribute

            if isempty(value)
                continue % skip empty attributes
            end

            % push the data into output structure
            tmpStructSub.name = field;
            tmpStructSub.val  = value;
                            
            % map groundTruth attribute types to OpenLABEL types
            switch class(value)                
                case {'char', 'string'} % this covers string and list choices and results in "text" for OpenLABEL
                    olType = "text";
                case 'double' % "num" entry in OpenLABEL attributes
                    olType = "num";                
                case 'logical' % "boolean" entry
                    olType = "boolean";
                otherwise
                    assert(false, "This line should be unreachable"); % should never reach this line
            end

            if isfield(structOut{roiIdx}, "attributes") && isfield(structOut{roiIdx}.attributes, olType)
                % append to an existing field
                structOut{roiIdx}.attributes.(olType){end+1} = tmpStructSub;
            else
                % first time assignment to this olType
                structOut{roiIdx}.attributes.(olType) = {tmpStructSub};
            end

            % update data pointer
            objDataPtr.attributes.(field) = olType;

        else % it's a sub-label
            if isempty(value)
                continue % skip empty sub-label rows
            end

            rowIdx = labelDefinitions.Name == className;
            hierarchy = labelDefinitions.Hierarchy{rowIdx};

            sublabel = hierarchy.(field);
            isPoly = false;

            % Handle multiple sub-ROIs of the same type
            numRois = numel(value);

            for roiIdx=1:numRois

                position = value(roiIdx).Position;

                % write out the sub-label
                switch (sublabel.Type)
                    case "Rectangle"
                        shape = "bbox";
                        position = bbox2openlabel(position);
                    case "RotatedRectangle"
                        shape = "rbbox";
                        position = rbbox2openlabel(position);
                    case "Point"
                        shape = "point2d";
                        position = points2openlabel(position);
                    case "Line"
                        shape = "poly2d";
                        mode = "MODE_POLY2D_ABSOLUTE";
                        isPoly = true;
                        isPolyClosed = false;
                        position = points2openlabel(position);
                    case "Polygon"
                        shape = "poly2d";
                        mode = "MODE_POLY2D_ABSOLUTE";
                        isPoly = true;
                        isPolyClosed = true;
                        if iscell(position)
                            assert(false, "Malformed groundTruth object that was not produced by the Image Labeler");
                        end
                        position = points2openlabel(position);
                    case "ProjectedCuboid"
                        shape = "poly2d";
                        mode = "MODE_POLY2D_ABSOLUTE";
                        isPoly = true;
                        isPolyClosed = false;
                        points = projcuboid2openlabel(position);
                        position = points;
                    otherwise
                        % Note: pixel labels are not permitted to be sub-labels
                        assert(false, "This line should be unreachable");
                end

                % Form an addition as a struct.  This temp struct lets us
                % create array of structures when necessary, e.g. multiple point
                % sub-labels are present.
                tmpStructSub = []; % reset the struct if it was used previously
                tmpStructSub.name = field;
                tmpStructSub.val  = position;

                % add additional fields required by poly-lines and polygons
                if isPoly
                    tmpStructSub.mode = mode;
                    tmpStructSub.closed = isPolyClosed;
                end

                if sublabel.Type == "ProjectedCuboid"
                    tmpStructSub.matlab_projected_cuboid = true;
                end

                structOut{1} = appendToStructArray(structOut{1}, shape, tmpStructSub);

                % process sub-attributes (recursive call)
                [structOut{1}.(shape), objDataPtrSecondLevel] = ...
                    appendAttributesAndSublabels(structOut{1}.(shape), value(roiIdx), ...
                    [], labelDefinitions, roiIdx);

            end

            % construct data pointer (like our label definition)
            objDataPtr.shape.type = shape;
            if ~isempty(objDataPtrSecondLevel)
                objDataPtr.shape.attributes = objDataPtrSecondLevel.attributes; % append sub-attributes
            end

        end
    end
end

    %----------------------------------------------------------------------
    function out = appendToStructArray(rootStruct, field, newStructBranch)

        out = rootStruct;

        if isfield(out, field)
            % If the field already exists, append another set of values
            % to it by creating an array. Note: for example, lines, polygons 
            % and projected cuboids all fall under poly2d, so the line
            % below will just keep appending under the already existing
            % OpenLABEL ROI type. A few MathWorks ROIs map into a single
            % type in OpenLABEL but with distinctive flags.
            out.(field){end+1} = newStructBranch; 
        else
            % Create the field from scratch
            out.(field) = {newStructBranch}; % wrap in cell array to get an array in JSON
        end

    end

end

%--------------------------------------------------------------------------
% Grab image filenames from the source. Handle the special case of custom
% reader being used in the ImageDatastore
%--------------------------------------------------------------------------
function fnames = getImageFilenames(gTruth)

if isa(gTruth.DataSource.Source,"matlab.io.datastore.ImageDatastore")
    fnames = gTruth.DataSource.Source.Files;
else
    fnames = gTruth.DataSource.Source;
end

end

%--------------------------------------------------------------------------
% Every OpenLabel structure contains prescribed elements defined by this
% routine.
%--------------------------------------------------------------------------
function root = getRootStruct

% When the data is written out, it will start with the "openlabel" tag at
% the top level
root.openlabel.metadata.schema_version = "1.0.0";

end

%--------------------------------------------------------------------------
% This routine already expects that the data is in JSON format as is
% produced by the jsonencode() function
%--------------------------------------------------------------------------
function writeJsonFile(jsonText, filename)

    % Write out a file
    fname = filename + ".json";
    [fid, errMsg] = fopen(fname,'w');

    if fid == -1 % check if the open call was successful
        error(message("vision:gtruthOpenLabel:cantOpenFile", fname, errMsg));
    end

    fwrite(fid, jsonText);
    fclose(fid);

end

%--------------------------------------------------------------------------
function bboxout = bbox2openlabel(bboxin)

bboxout = bboxin;

ulX = bboxin(1) - 1; % 0-based indexing that affects the coordinate system
ulY = bboxin(2) - 1;

% convert from UL corner to center of the bounding box
bboxout(1) = ulX + bboxin(3)/2;
bboxout(2) = ulY + bboxin(4)/2;
end

%--------------------------------------------------------------------------
function rbboxout = rbbox2openlabel(rbboxin)

rbboxout = rbboxin;

% Adjust bbox center to be in 0-based coordinate system
rbboxout(1) = rbboxin(1) - 1;
rbboxout(2) = rbboxin(2) - 1;

% width and height remain unchanged

% rotation must be in radians
rbboxout(5) = deg2rad(rbboxin(5));

end

%--------------------------------------------------------------------------
function points = rect2points(rect)

    ul = rect(1:2); % upper left corner [x,y]
    w = rect(3);    % width
    h = rect(4);    % height

    points = [ul; ul + [w 0]; ul + [w h]; ul + [0 h]];
end

%--------------------------------------------------------------------------
function pointsout = projcuboid2openlabel(cuboid)

points = [rect2points(cuboid(1:4)); rect2points(cuboid(5:8))];
            pointsout = points2openlabel(points);
end

%--------------------------------------------------------------------------
function pointsout = points2openlabel(pointsin)

% rearrange the points to x1,y1,x2,y2,...., xn,yn
points = pointsin';
points = points(:)';

% 0-based indexing
pointsout = points - 1;

end

%--------------------------------------------------------------------------
function out = mergeFrames(inBase, inAppendage)

out = inBase;

if ~isempty(inAppendage) % is there anything to append?
    for field = fieldnames(inAppendage)'
        fieldStr = field{:}; % strip the cell wrapper
        for subfield = fieldnames(inAppendage.(fieldStr))'
            subfieldStr = subfield{:};
            for subsubfield = fieldnames(inAppendage.(fieldStr).(subfieldStr))'
            subsubfieldStr = subsubfield{:};
            out.(fieldStr).(subfieldStr).(subsubfieldStr) = ...
                inAppendage.(fieldStr).(subfieldStr).(subsubfieldStr);
            end
        end
    end
end

end

%--------------------------------------------------------------------------
function out = appendStruct(inBase, inAppendage)

out = inBase;

if ~isempty(inAppendage) % is there anything to append?
    for field = fieldnames(inAppendage)'
        fieldStr = field{1};
        out.(fieldStr) = inAppendage.(fieldStr);
    end
end

end

%--------------------------------------------------------------------------
function pathOut = fixPathDelimiter(pathIn)

pathOut = strtrim(strrep(pathIn,'\','/'));

end