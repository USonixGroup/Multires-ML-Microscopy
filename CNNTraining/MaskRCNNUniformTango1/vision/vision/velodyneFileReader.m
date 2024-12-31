classdef velodyneFileReader < handle & matlab.mixin.SetGet & matlab.mixin.Copyable & ...
        vision.internal.EnforceScalarHandle

    % Copyright 2017-2023 The MathWorks, Inc.

    properties (GetAccess = 'public', SetAccess = 'private')
        FileName;
        DeviceModel;
        CalibrationFile;
        OrganizePoints logical
        NumberOfFrames;
        Duration;
        StartTime;
        EndTime;
        Timestamps;
        HasPositionData;
    end

    properties (GetAccess = 'public', SetAccess = 'public', Dependent)
        CurrentTime;
    end

    properties (Access = 'private', Hidden)

        %LaserCalibrationData Laser calibration data loaded from xml file.
        LaserCalibrationData;

        %UserCurrentTimeFlag Flag to determine if CurrentTime property
        %   is set by user.
        UserCurrentTimeFlag;

        %CurrentTimeInternal Same as CurrentTime but used for internal
        %   purpose.
        CurrentTimeInternal;

        %Version Version Number used for backward compatibility.
        Version = 1.0;
    end

    properties (Access = 'private', Transient)

        %VelodyneFileReaderObj Internal object for reading file.
        VelodyneFileReaderObj;
    end

    %==================================================================
    % Custom Getters/Setters
    %==================================================================
    methods

        %==================================================================
        % Get CurrentTime property value
        %==================================================================
        function value = get.CurrentTime(this)
            value = this.CurrentTimeInternal;
        end

        %==================================================================
        % Set CurrentTime property value
        %==================================================================
        function set.CurrentTime(this, value)

        % Check for datatype validity.
            validateattributes(value, {'duration'}, ...
                               {'nonempty', 'scalar', 'finite'}, 'set', 'CurrentTime');

            % Check for value finiteness and acceptable limits.
            if (value < this.StartTime || value > this.EndTime)
                error(message('vision:velodyneFileReader:invalidCurrentTime', ...
                    mat2str(seconds(this.StartTime)), mat2str(seconds(this.EndTime))));
            end
            this.UserCurrentTimeFlag = true;
            this.CurrentTimeInternal = value;
        end
    end

    methods (Access = 'public')

        %==================================================================
        % Constructor
        %==================================================================
        function this = velodyneFileReader(fileName, deviceModel, varargin)

            paramsStruct = iParseAndValidateInputs(fileName, deviceModel, varargin{:});

            this.FileName               = paramsStruct.FileName;
            this.DeviceModel            = paramsStruct.DeviceModel;
            this.CalibrationFile        = paramsStruct.CalibrationFile;
            this.OrganizePoints         = paramsStruct.OrganizePoints;

            % Setup calibration data
            [laserCalibrationData, distanceResolution] = iRetrieveCalibrationData( ...
                this.CalibrationFile, this.DeviceModel);

            this.LaserCalibrationData = laserCalibrationData;

            % Create the file reader object and open the file
            this.VelodyneFileReaderObj = vision.internal.VelodyneFileReader();

            openFcnArgs.LaserCalibrations               = this.LaserCalibrationData;
            openFcnArgs.DistanceResolution              = distanceResolution;
            openFcnArgs.SkipPartialFrames               = paramsStruct.SkipPartialFrames; % skipPartialFrames is for internal usage only.
            openFcnArgs.OrganizePoints                  = this.OrganizePoints;
            
            timeStruct = open(this.VelodyneFileReaderObj, ...
                              this.FileName, openFcnArgs, this.DeviceModel);

            % Fill class properties returned from mex call.
            if(~isempty(timeStruct))
                this.NumberOfFrames = timeStruct.NumberOfFrames;
                this.StartTime      = seconds(hRoundValue(timeStruct.StartTime));
                this.EndTime        = seconds(hRoundValue(timeStruct.EndTime));
                this.Duration       = seconds(hRoundValue(timeStruct.Duration));
                this.Timestamps     = seconds(hRoundValue(timeStruct.TimestampsVector));
                this.HasPositionData = timeStruct.NumPositionPackets ~= 0;
            end

            if isempty(this.NumberOfFrames) || this.NumberOfFrames <=0
                error(message('vision:velodyneFileReader:NoFramesWarning'));
            end

            this.CurrentTimeInternal = this.StartTime;
            this.UserCurrentTimeFlag = false;
        end

        %==================================================================
        % Read point cloud frame from velodyneFileReader object
        %==================================================================
        function [ptCloudObj, pointTimestamps, positionData] = readFrame(this, varargin)
            
            narginchk(1,2);
            
            % Set flag to see if position data is needed
            getPositionData = nargout >= 3;

            if nargin>1

                if isduration(varargin{1})

                    durationToSeek = varargin{1};

                    % Validate frameTime
                    validateattributes(durationToSeek, {'duration'}, ...
                                       {'nonempty', 'scalar', 'finite'}, 'readFrame', ...
                                       'frameTime');

                    % Check for valid duration values.
                    if (durationToSeek < this.StartTime || durationToSeek > this.EndTime)
                        error(message( 'vision:velodyneFileReader:invalidTimeDuration', ...
                                       char(this.StartTime), char(this.EndTime)));
                    end

                    % Convert to double, and remove start time
                    durationToSeekSeconds = round(double(seconds(durationToSeek - this.StartTime)), 9);

                    % Call builtin function with duration in seconds.
                    [xyziPoints, intensity, currentTimestamp,...
                        rangeData, notNaNIndices, numValidPoints, ptTimestamps,...
                        gpsData, orientationData, positionTimestamps] = readPointCloud(...
                        this.VelodyneFileReaderObj, durationToSeekSeconds, getPositionData);

                elseif isnumeric(varargin{1})

                    frameNum = varargin{1};

                    % Validate frameNumber
                    validateattributes(frameNum, {'numeric'}, ...
                                       {'nonempty', 'scalar', 'finite', 'integer', 'real', 'positive', '<=', this.NumberOfFrames}, ...
                                       'readFrame', 'frameNumber');

                    % Convert to int32 and zero-based indexing
                    frameNumber = int32(frameNum - 1);

                    % Call builtin function with frame number.
                    [xyziPoints, intensity, currentTimestamp,...
                        rangeData, notNaNIndices, numValidPoints, ptTimestamps,...
                        gpsData, orientationData, positionTimestamps] = readPointCloud(...
                        this.VelodyneFileReaderObj, frameNumber, getPositionData);
                  
                else
                    % The type of varargin{1} is invalid. Throw an error
                    validateattributes(varargin{1}, {'numeric', 'duration'}, ...
                                       {}, 'readFrame', 'frameNumber or frameTime');
                end

            else

                % Default builtin call if optional arguments not provided.
                if this.UserCurrentTimeFlag
                    % If CurrentTime is set by user, use it to retrieve
                    % point cloud.
                    durationToSeekSeconds = round(double(seconds(this.CurrentTimeInternal - this.StartTime)), 9);

                    [xyziPoints, intensity, currentTimestamp,...
                        rangeData, notNaNIndices, numValidPoints, ptTimestamps,... 
                        gpsData, orientationData, positionTimestamps] = readPointCloud(...
                        this.VelodyneFileReaderObj, durationToSeekSeconds, getPositionData);
                else
                    % If user does not provide CurrentTime, read next point
                    % cloud in sequence.
                    if hasFrame(this)
                        [xyziPoints, intensity, currentTimestamp,...
                            rangeData, notNaNIndices, numValidPoints, ptTimestamps,...
                            gpsData, orientationData, positionTimestamps] = readPointCloud(...
                            this.VelodyneFileReaderObj, int32(-1), getPositionData);
                    else
                        error(message('vision:pcapFileReader:endOfFile'));
                    end
                end
            end

            numRows = size(xyziPoints,1);
            numCols = size(xyziPoints,2);

            
            if this.OrganizePoints
                % For organized point cloud output

                % In the builtin implementation, xyziPoints and other
                % matrices is not initialized with NaN, for better
                % performance. Hence once the points are obtained, the
                % indices that are not assigned any value, is assigned with
                % the NaN values.

                % Calculate the indices to be assigned with NaN value
                nanIndices = find(~notNaNIndices);
                nanIndicesAllDims = [nanIndices; nanIndices+(numRows*numCols);...
                    nanIndices+(numRows*numCols*2)];


                xyziPoints(nanIndicesAllDims) = nan('like', xyziPoints);
                rangeData(nanIndicesAllDims) = nan('like', rangeData);
                intensity(nanIndices) = zeros('like', intensity);
                ptTimestamps(nanIndices) = nan('like', ptTimestamps);
            else

                % For unorganized point cloud output

                % In the builtin implementation, xyziPoints and other
                % matrices are of size equal to the number of points. This
                % is done to maintain a common implementation b/w organized
                % and unorganized output, at the same time pre allocate
                % memory for these matrices. Once we get the output here,
                % only the valid points are obtained.
                
                % Get indices for valid points
                validIndices = 1:numValidPoints;

                % Get only valid points
                xyziPoints = squeeze(cat(3, xyziPoints(validIndices),...
                    xyziPoints(validIndices+(numRows*numCols)),...
                    xyziPoints(validIndices+(numRows*numCols*2))));
                intensity = intensity(validIndices);
                if size(intensity,2) ~= 1
                    intensity = intensity';
                end
                rangeData = squeeze(cat(3, rangeData(validIndices),...
                    rangeData(validIndices+(numRows*numCols)),...
                    rangeData(validIndices+(numRows*numCols*2))));

                ptTimestamps = ptTimestamps(validIndices)';
            end

            % Create pointCloud object from xyziPoints, intensity and
            % rangData
            [ptCloudObj, ptTimestamps]  = makePointCloud(this, xyziPoints, intensity, rangeData, ptTimestamps);

            % Create positionData structure from GPS, Orientation, and Timestamp Data
            if(this.HasPositionData && getPositionData && ~isempty(gpsData) && ...
                ~isempty(orientationData) && ~isempty(positionTimestamps))
                % gpsData Format: [lat lon heading altitude gpsTime]
                gpsReadings = gpsData(:, 1:4);
                
                % orientationData Format: [gyro1, gyro2, gyro3, accel1x,
                % accel2x, accel3x, accel1y, accel2y, accel3y]
                gyroReadings = deg2rad(orientationData(:, 1:3));

                % Reorder to align with gyro data
                accelReadings = orientationData(:, [8 7 4 9 6 5]);

                positionTimestamps = seconds(positionTimestamps);

                % Update GPSTimes to duration
                gpsHours = floor(gpsData(:, 5)/10000);
                gpsMinutes = floor((gpsData(:, 5) - gpsHours*10000)/100);
                gpsSeconds  = gpsData(:, 5) - gpsHours*10000 - gpsMinutes*100;
                gpsTimes = duration(gpsHours, gpsMinutes, gpsSeconds);

                if (this.DeviceModel ~= "HDL32E")
                    gyroReadings = [];
                    accelReadings = [];
                end

                % If all GPS readings are NaN, return empty array
                if (~any(any(gpsReadings)))
                    gpsReadings = [];
                    gpsTimes = [];
                end
                
                positionData = struct("GPSReadings", gpsReadings, ...
                                      "GyroscopeReadings", gyroReadings, ...
                                      "AccelerometerReadings", accelReadings, ...
                                      "PositionTimestamps", positionTimestamps, ...
                                      "GPSTimes", gpsTimes);
            else
                positionData = struct([]);
            end

            
            pointTimestamps = seconds(hRoundValue(ptTimestamps));
            % Update current time
            this.CurrentTimeInternal = this.StartTime + seconds(currentTimestamp);

            this.UserCurrentTimeFlag = false;
        end

        %==================================================================
        % Check if another point cloud is available to read
        %==================================================================
        function flag = hasFrame(this)

            % Check if timestamp of last frame requested is less than the
            % EndTime of the file.
            % Timestamps in the Velodyne packet are reported in
            % microseconds. If the difference between EndTime and
            % CurrentTimeInterval is less than 1 microsecond (i.e. 1e-6),
            % consider them to be close enough to report reaching last
            % frame, i.e, end of the file reached and next frame
            % unavailable.

            flag = abs(seconds(this.EndTime) - seconds(this.CurrentTimeInternal)) >= 1e-6;
        end

        %==================================================================
        % Reset velodyneFileReader object to beginning of the file
        %==================================================================
        function reset(this)

            % Set CurrentTime property to StartTime.
            this.CurrentTime = this.StartTime;
        end
    end
    methods (Hidden)
        %==================================================================
        % clear resources
        %==================================================================
        function delete(this)
        % Call builtin and release resources.
            close(this.VelodyneFileReaderObj);
            % Invalidate class properties.
            this.FileName             = [];
            this.NumberOfFrames       = [];
            this.StartTime            = [];
            this.EndTime              = [];
            this.Duration             = [];
            this.CurrentTimeInternal  = seconds(0);
            this.LaserCalibrationData = [];
            this.DeviceModel          = [];
            this.CalibrationFile      = [];
            this.Timestamps           = [];
            this.HasPositionData      = [];
        end

    end

    methods(Access = 'protected')
        %==================================================================
        % copy object
        %==================================================================
        function copyObj = copyElement(this)

            % Override copyElement method
            if(~isempty(this.CalibrationFile))
                copyObj = velodyneFileReader(this.FileName,...
                    this.DeviceModel, 'CalibrationFile', this.CalibrationFile,...
                    'OrganizePoints',this.OrganizePoints);
            else
                copyObj = velodyneFileReader(this.FileName, this.DeviceModel,...
                    'OrganizePoints',this.OrganizePoints);
            end
            copyObj.CurrentTime = this.CurrentTime;
        end
    end

    methods(Hidden)
        %==================================================================
        % save object
        %==================================================================
        function s = saveobj(this)
        % save properties into struct
            s.FileName        = this.FileName;
            s.DeviceModel     = this.DeviceModel;
            s.CalibrationFile = this.CalibrationFile;
            s.OrganizePoints  = this.OrganizePoints;
            s.Version         = this.Version;
            s.CurrentTime     = this.CurrentTime;
        end
    end

    methods (Static, Hidden)
        %==================================================================
        % load object
        %==================================================================
        function this = loadobj(s)
        % Load Object
            currentTime = s.CurrentTime;

            if ~isfield(s, 'OrganizePoints')
                organizePoints = true;
            else
                organizePoints = s.OrganizePoints;
            end

            if(~isempty(s.CalibrationFile))
                this = velodyneFileReader(s.FileName, s.DeviceModel,...
                    'CalibrationFile', s.CalibrationFile,...
                    'OrganizePoints', organizePoints);
            else
                this = velodyneFileReader(s.FileName, s.DeviceModel,...
                    'OrganizePoints', organizePoints);
            end
            this.CurrentTime = currentTime;
        end
    end
    
    methods (Static)
        function validDeviceModels = validDeviceModels()
            validDeviceModels = {'VLP16', 'PuckLITE', 'PuckHiRes',...
                'VLP32C', 'HDL32E', 'HDL64E', 'VLS128', 'VelarrayH800'};
        end
    end
    
    methods(Access = 'private')
        %==================================================================
        % Create pointCloud object
        %==================================================================
        function [ptCloudObj, ptTimestamps]  = makePointCloud(this, xyziPoints, intensity, rangeData, ptTimestamps)
            % The laser firing sequence reported in the Velodyne packet is
            % in an order different from the lasers vertical angles (i.e.
            % the order in which lasers are placed/mounted vertically). So,
            % the vertical angles are used to arrange points according to
            % their respective laser's position. Refer to Velodyne Device
            % Manual(s) for more info on this.
            
            persistent sortedVerticalAngleIndices;
            

            if this.OrganizePoints
                if(isempty(sortedVerticalAngleIndices) || length(sortedVerticalAngleIndices)~= size(this.LaserCalibrationData,1))
                    % Laser vertical angles are stored in 2nd column of
                    % LaserCalibrationData.
                    laserVerticalAngles = this.LaserCalibrationData(:, 2);

                    % Sort the vertical angles and obtain indices to sorted
                    % vertical angles.
                    [~, sortedVerticalAngleIndices] = sort(laserVerticalAngles, 'descend');
                end

                xyziPoints = xyziPoints(sortedVerticalAngleIndices, :, :);
                intensity = intensity(sortedVerticalAngleIndices, :);
                rangeData = rangeData(sortedVerticalAngleIndices, :, :);
                ptTimestamps = ptTimestamps(sortedVerticalAngleIndices, :);
            end

            % Create pointCloud object from xyziPoints, with points
            % sorted according to the laser vertical angles.
            ptCloudObj  = pointCloud(xyziPoints, ...
                'Intensity', intensity);
            
            % range, pitch, yaw
            ptCloudObj.RangeData = rangeData;
        end
    end

    methods (Access = ?matlab.unittest.TestCase)
        
        function laserVerticalAngles = getVerticalAngles(this)
        % Return laser vertical angles from the calibration file

            % Laser vertical angles are stored in 2nd column of
            % LaserCalibrationData.        
            laserVerticalAngles = this.LaserCalibrationData(:, 2);

            % Sort the vertical angles 
            laserVerticalAngles = deg2rad(sort(laserVerticalAngles, 'descend'));
        end

        function verticalOffsetCorr = getVerticalOffsetCorr(this)
        % Return vertical offset correction for lasers from the calibration
        % file. Note that the values are in centimeters, in order to use
        % this you might have to convert it to meters as the points are in
        % those units.

            laserVerticalAngles = this.LaserCalibrationData(:, 2);

            % Sort the vertical angles and obtain indices to sorted
            % vertical angles.
            [~, sortedVerticalAngleIndices] = sort(laserVerticalAngles, 'descend');


            % Laser vertical offset corrections are stored in 6th column of
            % LaserCalibrationData.            
            verticalOffsetCorr = this.LaserCalibrationData(sortedVerticalAngleIndices, 6);
        end
    end
end

%==========================================================================
% input parsing and validation
%==========================================================================
function paramsStruct = iParseAndValidateInputs(fileName, deviceModel, varargin)

% Validate fileName
    fileName = iValidatePCAPFile(fileName);

    % Validate device model
    deviceModel = iValidateDeviceModel(deviceModel);

    % Parse name-value inputs
    p = inputParser;
    p.FunctionName = 'velodyneFileReader';

    defaultCalibrationFile = iGetDefaultCalibrationFile(deviceModel);
    addParameter(p, 'CalibrationFile', defaultCalibrationFile);
    addParameter(p, 'SkipPartialFrames', true);    % Note that this parameter is undocumented
    addParameter(p, 'OrganizePoints', true, @(x)iValidateOrganizePoints(x));

    parse(p, varargin{:});

    paramsStruct = p.Results;
    if(isempty(paramsStruct.CalibrationFile))
        error(message('vision:velodyneFileReader:noCalibrationFile', deviceModel));
    end

    % Check that calibration file exists
    paramsStruct.CalibrationFile   = iValidateCalibrationFile(paramsStruct.CalibrationFile);
    paramsStruct.SkipPartialFrames = iValidateSkipPartialFrames(paramsStruct.SkipPartialFrames);

    paramsStruct.FileName       = fileName;
    paramsStruct.DeviceModel    = deviceModel;
end

%==========================================================================
function skipPartialFrames = iValidateSkipPartialFrames(skipPartialFrames)

    validateattributes(skipPartialFrames, {'logical'}, {'binary'}, ...
                       'velodyneFileReader', 'SkipPartialFrames');
end

%==========================================================================
function iValidateOrganizePoints(organizePoints)

    validateattributes(organizePoints, {'logical'}, {'scalar'}, ...
                       'velodyneFileReader', 'OrganizePoints');
end

%==========================================================================
function deviceModel = iValidateDeviceModel(deviceModel)

    validateattributes(deviceModel, {'char', 'string'}, {'nonempty', 'scalartext'}, ...
                       'velodyneFileReader', 'deviceModel');

    validDeviceModels = velodyneFileReader.validDeviceModels;
    deviceModel = validatestring(deviceModel, validDeviceModels, ...
                                 'velodyneFileReader', 'deviceModel');
end

%==========================================================================
function fileName = iValidatePCAPFile(fileName)

    validateattributes(fileName, {'char', 'string'}, {'nonempty', 'scalartext'}, ...
                       'velodyneFileReader', 'fileName');

    fileName = iCheckFile(fileName, '.pcap', 'PCAP');
end

%==========================================================================
function calibrationFile = iValidateCalibrationFile(calibrationFile)

    validateattributes(calibrationFile, {'char', 'string'}, ...
                       {'nonempty', 'scalartext'}, 'velodyneFileReader', 'CalibrationFile');

    calibrationFile = iCheckFile(calibrationFile, '.xml', 'Calibration XML');
end

%==========================================================================
function defaultCalibrationFile = iGetDefaultCalibrationFile(deviceModel)

	defaultCalibrationFile = fullfile(matlabroot, 'toolbox', 'shared', ...
        'pointclouds', 'utilities', 'velodyneFileReaderConfiguration', ...
        [char(deviceModel) '.xml']);
    if(string(deviceModel) == "VelarrayH800")
        defaultCalibrationFile = [];
    end
end

%==========================================================================
function [laserCalibrationData, distanceResolution] = iRetrieveCalibrationData(calibrationFile, deviceModel)

% Read Calibration data from xml file.
    [laserCalibrationData, distanceResolution]  = iGetVelodyneCorrectionsFromXML(calibrationFile);

    % Number of lasers based on Velodyne device model.
    if strcmpi('HDL32E', deviceModel) || strcmpi('VLP32C', deviceModel)
        numLasersOfDeviceModel = 32;
    elseif strcmpi('HDL64E', deviceModel)
        numLasersOfDeviceModel = 64;
    elseif strcmpi('VLS128', deviceModel)
        numLasersOfDeviceModel = 128;
    elseif strcmpi('VelarrayH800', deviceModel)
        numLasersOfDeviceModel = 8;
    else
        numLasersOfDeviceModel = 16;
    end

    % Validate laser count in calibration data with number of
    % lasers for given device model.
    if size(laserCalibrationData, 1) ~= numLasersOfDeviceModel
        error(message(...
            'vision:velodyneFileReader:invalidCalibrationFileIncorrectEnabledLaserCountForDeviceModel', ...
            deviceModel, numLasersOfDeviceModel));
    end
end

function fileName = iCheckFile(fileName, expectedExtension, fileType)

% Check the file extension
    [~,~,fileExtension] = fileparts(fileName);

    if ~strcmpi(fileExtension, expectedExtension)
        error(message('vision:velodyneFileReader:invalidFileType', fileType, ...
                      expectedExtension, fileExtension));
    end

    % Check if the file exists
    [fid, msg] = fopen(fileName, 'r');

    if fid == -1
        error(message('vision:pcapFileReader:fileOpenFailed', fileType, ...
                      fileName, msg));
    else
        % Get full path name of file
        fileName = fopen(fid);
        fclose(fid);
    end
end

%==========================================================================
% helper function to load laser calibration data from XML file
%==========================================================================
function [laserCorrections, distLSB] = iGetVelodyneCorrectionsFromXML(xmlFile)

% Parse the provided Velodyne laser calibration xml file.
    try
        domNode = readstruct(xmlFile);
    catch ME
        error(message('vision:velodyneFileReader:calibrationFileReadError', ME.message));
    end
    % Get distLSB_ value
    distLSB = single(0.2);
    isDistLSBTagFound = isfield(domNode.DB, 'distLSB_');
    if(isDistLSBTagFound)
        distLSB = single(domNode.DB.distLSB_);
    end
    
    isEnabledTagMissing = ~isfield(domNode.DB,'enabled_');
    if(isEnabledTagMissing)
        error(message('vision:velodyneFileReader:invalidCalibrationFileTagNotFound', ...
                      'enabled_'));
    end
    
    % Count number of lasers enabled
    enabled = domNode.DB.enabled_;
    
    enabledCount = 0;

        for i= 1 : numel(enabled.item)
            if( enabled.item(i)> 0)
                enabledCount = enabledCount+1;
            end
        end
    
    isValidLaserCount = (enabledCount == 8 || enabledCount == 16 || enabledCount == 32 || enabledCount == 64 || enabledCount == 128);
    if(enabledCount <= 0 || ~isValidLaserCount)
        error(message('vision:velodyneFileReader:invalidCalibrationFileIncorrectEnabledLaserCount'));
    end

    isPxTagMissing = ~(isfield(domNode.DB.points_.item, 'px'));
    if(isPxTagMissing)
        error(message('vision:velodyneFileReader:invalidCalibrationFileTagNotFound', 'px'));
    end
    isPxTagCountLessThanEnabledCount = (numel(domNode.DB.points_.item) < enabledCount);
    if(isPxTagCountLessThanEnabledCount)
        error(message('vision:velodyneFileReader:invalidCalibrationFileMissingCalibrationValues'));
    end
    laserCorrections = zeros(enabledCount, 9, 'single');
    % Read the following tag values into laserCorrections array.
    %   <rotCorrection_>-5.3328056</rotCorrection_>
    %   <vertCorrection_>-7.2988362</vertCorrection_>
    %   <distCorrection_>111</distCorrection_>
    %   <distCorrectionX_>118</distCorrectionX_>
    %   <distCorrectionY_>118</distCorrectionY_>
    %   <vertOffsetCorrection_>19.736338</vertOffsetCorrection_>
    %   <horizOffsetCorrection_>2.5999999</horizOffsetCorrection_>
    %   <focalDistance_>0</focalDistance_>
    %   <focalSlope_>0</focalSlope_>

    for k = 1 : enabledCount
        pxItem = domNode.DB.points_.item(k);
        if ~isstruct(pxItem)
            error(message('vision:velodyneFileReader:invalidCalibrationFileTagNotFound', 'px'));
        end
       
        laserCorrections(k, 1) = iGetTagValue(pxItem, 'rotCorrection_');
        laserCorrections(k, 2) = iGetTagValue(pxItem, 'vertCorrection_');
        laserCorrections(k, 3) = iGetTagValue(pxItem, 'distCorrection_');
        laserCorrections(k, 4) = iGetTagValue(pxItem, 'distCorrectionX_');
        laserCorrections(k, 5) = iGetTagValue(pxItem, 'distCorrectionY_');
        laserCorrections(k, 6) = iGetTagValue(pxItem, 'vertOffsetCorrection_');
        laserCorrections(k, 7) = iGetTagValue(pxItem, 'horizOffsetCorrection_');
        laserCorrections(k, 8) = iGetTagValue(pxItem, 'focalDistance_');
        laserCorrections(k, 9) = iGetTagValue(pxItem, 'focalSlope_');
    end
end

%==========================================================================
% helper function to parse a numeric tag value for a given XML tag
%==========================================================================
function tagValue = iGetTagValue(pxItem, tagName)
% Utility function to parse, validate and return tag
% value for a given tag in xml file.
  
if isfield(pxItem.px,tagName)
    tagValue = pxItem.px.(tagName);
    if isstring(tagValue)
        tagValue = str2double(tagValue);
    end
    if(isnan(tagValue))
        error(message('vision:velodyneFileReader:invalidCalibrationFileNonnumericTagValue', tagName));
    end
else
    error(message('vision:velodyneFileReader:invalidCalibrationFileTagNotFound', tagName));
end

end

function val = hRoundValue(val, precisionInDecimals)
% Helper function to round values
if(~exist('precisionInDecimals', 'var'))
    precisionInDecimals = 9;
end
val = round(val, precisionInDecimals);
end