% PatternSet Stores all information about calibration pattern images
%
%    This is a parent class that stores the file names from which the
%    patterns were extracted as well as pattern world points and detected
%    key points in the pattern. This should be inherited by child classes
%    which specialize this class based on the calibration pattern type (eg.
%    checkerboard, custom, etc.)

% Copyright 2021 The MathWorks, Inc.

classdef PatternSet < handle
       
    %----------------------------------------------------------------------
    properties(Access=private, Hidden)
        Version = ver('vision');
    end
    
    %----------------------------------------------------------------------
    properties(GetAccess=public, SetAccess=protected)
        PatternDetector
        
        PatternDetectorFile
        
        FullPathNames = {};
        
        PatternPoints 
        
        WorldPoints
        
        PatternLabels % labels displayed in the data browser
        
        LastNonDetectedPathNames = {};
        
        NumPatterns = 0;
        
        ImageSize = [];
    end
    
    %----------------------------------------------------------------------
    properties(GetAccess=public, SetAccess=public)
       Units; % made it writable in 17a to support switch from short names to longer names in->inches
       
       % SetAccess is public to support backward compatibility
       PartialPatterns = [];
    end
    
    %----------------------------------------------------------------------
    properties(Dependent)
        IsStereo;
    end
    
    %----------------------------------------------------------------------
    % Define these abstarct methods in the sub-classes for each pattern
    %----------------------------------------------------------------------
    methods (Abstract)
        %------------------------------------------------------------------
        % Method to detect keypoints in image and add to a clean session
        %------------------------------------------------------------------
        delIdx = detectPointsForNewSession(this, varargin);
        
        %------------------------------------------------------------------
        % Method to add additional patterns to existing session
        %------------------------------------------------------------------
        numDuplicates = addPatterns(this, varargin);

        %------------------------------------------------------------------
        % Method to get calibration pattern properties 
        %------------------------------------------------------------------
        pattern = getPattern(this, patternIdx)
    end
    
    methods
        %------------------------------------------------------------------
        function isStereo = get.IsStereo(this)
            isStereo = size(this.FullPathNames, 1) == 2;
        end        
    end
    
    %----------------------------------------------------------------------
    methods (Access=public)
        
        function this = PatternSet(varargin)
            % Allow barebones construction for backward compatibiliy during
            % loading
            if ~isempty(varargin{1})
                if nargin > 1
                    fileNames = varargin{1};
                    detector = varargin{2};
                    detectorFile = varargin{3};
                else
                    fileNames = varargin{1}{1};
                    detector = varargin{1}{2};
                    detectorFile = varargin{1}{3};
                end
                
                % Pattern detector
                this.PatternDetector = detector;
                
                % Pattern detector file name
                this.PatternDetectorFile = detectorFile;
                
                % begin by inspecting image size consistency
                this.ImageSize     = this.checkImageSizes(fileNames);
                
                % proceed with adding the boards
                this.FullPathNames = fileNames;
                this.Units         = detector.WorldUnits;
                
                % set board labels by picking the file names without the path
                this.NumPatterns = size(fileNames, 2);
                this.PatternLabels = cell(1, this.NumPatterns);
                for i = 1:this.NumPatterns
                    this.PatternLabels{i} = makeLabel(this, fileNames(:, i));
                end
            end
        end

        %------------------------------------------------------------------
        function updateImageSize(this)
            % Assumes that all sizes are valid and have been checked.
            imInfoBase = imageinfo(this.FullPathNames{1});
            
            % Use abs value since imfinfo could return negative
            % width/height (eg. BMP: https://en.wikipedia.org/wiki/BMP_file_format)
            this.ImageSize = abs([imInfoBase(1).Height, imInfoBase(1).Width]);
        end
            
        %------------------------------------------------------------------
        function removePattern(this, patternIdx)

            this.PatternLabels(patternIdx)   = [];
            this.FullPathNames(:, patternIdx) = [];
            this.PatternPoints(:,:,patternIdx, :) = [];
            this.PartialPatterns(patternIdx) = [];

            this.NumPatterns = this.NumPatterns - length(patternIdx);
        end
            
        %------------------------------------------------------------------
        function reset(this)
            this.PatternDetector = [];
            
            this.FullPathNames = [];
            this.LastNonDetectedPathNames = {};
            
            this.PatternPoints = [];
            this.WorldPoints = [];
            
            this.PatternLabels = {};
            this.PartialPatterns = [];
            
            this.NumPatterns = 0;
        end

        %------------------------------------------------------------------
        % This method should be called after the PatternSet is loaded from a
        % MAT file to check that all the images can be found at their
        % specified locations
        %------------------------------------------------------------------
        function checkImagePaths(this, currentSessionFilePath,...
                atSavingTimeFullSessionFileName)
            
            % verify that all the images are present; adjust path if
            % necessary
            for i=1:numel(this.FullPathNames)
                if ~exist(this.FullPathNames{i},'file')
                    
                    this.FullPathNames{i} = vision.internal.uitools.tryToAdjustPath(...
                        this.FullPathNames{i}, currentSessionFilePath, ...
                        atSavingTimeFullSessionFileName);
                    
                end
            end            
        end
        
    end % public methods
    
    %----------------------------------------------------------------------
    methods (Access=protected)

        function label = makeLabel(this, fileNames)
            [~,fname, ext] = fileparts(fileNames{1, 1});
            fileName1 = [fname, ext];
                                      
            if this.IsStereo
                [~,fname, ext] = fileparts(fileNames{2, 1});
                fileName2 = [fname, ext];
                label = [fileName1, ' & ', fileName2];
            else
                label = fileName1;
            end
        end
                     
        %------------------------------------------------------------------
        % checks image size consistency
        function imageSize = checkImageSizes(this, fileNames)
            
            isBrandNewSession = (this.NumPatterns == 0);
            
            % get base image size
            if isBrandNewSession
                % use the very first image
                imInfoBase = imageinfo(fileNames{1});
            else
                % use an already loaded image
                imInfoBase = imageinfo(this.FullPathNames{1});
            end
            
            for i=1:numel(fileNames)
                imInfo = imageinfo(fileNames{i});
                if (imInfoBase(1).Width ~= imInfo(1).Width) || ...
                        (imInfoBase(1).Height ~= imInfo(1).Height)
                    % issue an error message
                    error(message('vision:caltool:imageSizeInconsistent'));
                end                
            end
            imageSize = abs([imInfoBase(1).Height, imInfoBase(1).Width]);
        end                
        
        %------------------------------------------------------------------
        function addOnePattern(this, fileName, points)
            % for a single camera fileName is a cell array of 1 element
            % for a stereo camera fileName is a cell array of 2 elements
            this.PatternPoints(:, :, end+1, :) = points;
            this.FullPathNames(:, end+1) = fileName;

            this.PatternLabels{end+1} = makeLabel(this, fileName);
            this.NumPatterns = this.NumPatterns + 1;
            this.PartialPatterns(end+1) = any(isnan(points(:,1)));
        end
        
        %------------------------------------------------------------------
        function rollback(this, NumPatternsToKeep)
            this.FullPathNames = this.FullPathNames(1:NumPatternsToKeep);
            this.PatternPoints = this.PatternPoints(:, :, 1:NumPatternsToKeep);
            this.PatternLabels = this.PatternLabels(1:NumPatternsToKeep);
            this.PartialPatterns = this.PartialPatterns(1:NumPatternsToKeep);
            this.NumPatterns = NumPatternsToKeep;
        end
        
        %------------------------------------------------------------------
        function tf = isDuplicateFileName(this, fileName)
            tf = false;
            if any(strcmp(fileName{1}, this.FullPathNames(1, :)))
                tf = true;
                return;
            end
            
            if this.IsStereo && any(strcmp(fileName{2}, this.FullPathNames(2, :)))
                tf = true;
                return;
            end
        end
        
        %------------------------------------------------------------------
        function deleteRejectedPatterns(this, delIdx)
            % store file names of the images where patterns were not
            % detected; this is used for reporting
            this.LastNonDetectedPathNames = this.FullPathNames(:, delIdx);
            
            this.NumPatterns = size(this.PatternPoints, 3);
            this.PatternLabels(delIdx)   = [];
            this.FullPathNames(:, delIdx) = [];
        end
    end
    
    %----------------------------------------------------------------------
    % saveobj and loadobj are implemented to ensure compatibility across
    % releases even if architecture of PatternSet class changes
    methods (Hidden)
        %------------------------------------------------------------------
        function thisOut = saveobj(this)
            
            thisOut = this;            
        end
        
        %------------------------------------------------------------------
        function setImageSize(this, imSize)
           this.ImageSize = imSize; 
        end
        
    end
    
    %----------------------------------------------------------------------
    methods (Static, Hidden)
       
        function this = loadobj(that)
            % disable the mustReturnObject warning  to avoid warning popup
            % The mismatch occurred due to removal of java objects from
            % 'vision.internal.calibration.tool.PatternSet' with older version
            warning('off','MATLAB:class:mustReturnObject');
            this = that;
        end
        
    end % methods(static, hidden)
    
end

function info = imageinfo(filenames)
warnstruct = warning('off', 'imageio:tifftagsread:badTagValueDivisionByZero');
onCleanup(@()warning(warnstruct)); %#ok<UNONC>
info = imfinfo(filenames);
end
