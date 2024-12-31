%PixelLabelDatastore An object for storing a collection of pixel label data.
%
%   PixelLabelDatastore is created using the pixelLabelDatastore
%   function. Use a PixelLabelDatastore to represent and read data from a
%   collection of semantic segmentation results or pixel label data stored
%   in a groundTruth object.
%
%   PixelLabelDatastore Properties:
%
%      Files                    - A cell array of file names
%      ClassNames               - A cell array of class names
%      ReadSize                 - Upper limit on the number of images 
%                                 returned by the read method
%      ReadFcn                  - Function handle used to read files
%      AlternateFileSystemRoots - Alternate file system root paths for the Files.
%
%   PixelLabelDatastore Methods:
%
%      hasdata         - Returns true if there is more data to read.
%      read            - Reads the next consecutive file.
%      reset           - Resets the datastore to the start of the data.
%      preview         - Reads the first image from the datastore.
%      readimage       - Reads a specified image from the datastore.
%      readall         - Reads all pixel label data from the datastore.
%      partition       - Returns a new datastore that represents a single.
%                        partitioned portion of the original datastore.
%      numpartitions   - Returns an estimate for a reasonable number of
%                        partitions to use with the partition function,
%                        according to the total data size.
%      countEachLabel  - Counts the number of pixel labels for each class.
%      transform       - Create an altered form of the current datastore by
%                        specifying a function handle that will execute
%                        after read on the current datastore.
%      combine         - Create a new datastore that horizontally
%                        concatenates the result of read from two or more
%                        input datastores.
%      subset          - Return a new datastore that contains the
%                        observations corresponding to the input indices.
%      shuffle         - Return a new datastore that shuffles all the
%                        observations in the input datastore.
%
% Example: Read and display pixel label data.
% --------------------------------------------
% % Location of image and pixel label data
% dataDir = fullfile(toolboxdir('vision'), 'visiondata');
% imDir = fullfile(dataDir, 'building');
% pxDir = fullfile(dataDir, 'buildingPixelLabels');
% 
% % Create image datastore
% imds = imageDatastore(imDir);
% 
% % Create pixel label datastore. 
% classNames = ["sky" "grass" "building" "sidewalk"];
% pixelLabelID = [1 2 3 4];
% pxds = pixelLabelDatastore(pxDir, classNames, pixelLabelID);
% 
% % Read image and pixel label data. 
% I = read(imds);
% C = read(pxds);
% 
% % Display the label categories in C
% categories(C{1})
% 
% % Overlay pixel label data on the image and display.
% B = labeloverlay(I, C{1});
% figure
% imshow(B)
%
% See also pixelLabelDatastore, imageDatastore, groundTruth, semanticseg,
%          evaluateSemanticSegmentation, imageLabeler.

% Copyright 2017-2021 The MathWorks, Inc.

classdef PixelLabelDatastore <  ...
        matlab.io.Datastore & ...
        matlab.io.datastore.mixin.Subsettable & ... 
        matlab.io.datastore.HadoopFileBased & ...
        matlab.mixin.CustomDisplay
    
    properties(Dependent)
        %Files -
        % Cell array of file names.
        Files 
    end
    
    properties(Dependent, SetAccess = private)
        %ClassNames -
        % A cell array of the names of the classes.
        ClassNames
    end
    
    properties(Dependent)
        %ReadSize
        % Number of image files for one read.
        ReadSize        
        
        %ReadFcn
        % A custom reader function handle used by read method. The custom
        % read function must take an image file name as input, and then
        % output the corresponding pixel labeled data as a categorical
        % matrix.
        ReadFcn
        
        %AlternateFileSystemRoots
        % Alternate file system roots for the files.
        %   Alternate file system root paths for the files provided in the
        %   Files property. AlternateFileSystemRoots contains one or more
        %   rows, where each row specifies a set of equivalent root paths
        %   used to look up file paths when a saved PixelLabelDatastore is
        %   loaded from a MAT-file. Values for AlternateFileSystemRoots can
        %   be one of these:
        %
        %      - A string row vector of root paths, such as
        %                 ["Z:\datasets", "/mynetwork/datasets"]
        %
        %      - A cell array of root paths, where each row of the cell array can be
        %        specified as string row vector or a cell array of character vectors,
        %        such as
        %                 {["Z:\datasets", "/mynetwork/datasets"];...
        %                  ["Y:\datasets", "/mynetwork2/datasets","S:\datasets"]}
        %        or
        %                 {{'Z:\datasets','/mynetwork/datasets'};...
        %                  {'Y:\datasets', '/mynetwork2/datasets','S:\datasets'}}
        AlternateFileSystemRoots        
    end
    
    properties(Hidden, SetAccess = private)
        %LabelDefinitions A table with two columns: Name, PixelLabelID.
        %                 Holds label name to pixel label ID mappings.
        LabelDefinitions
        
        %CategoricalCache A cell array of cached categoricals for each
        %                 label definition table. This is used to create
        %                 the categorical during each read and acts as a
        %                 look-up table.
        CategoricalCache  
        
        %isValidCategoricalCache False if custom ReadFcn returns
        %                        categorical data, true else.
        isValidCategoricalCache logical
    end
    
    properties(Hidden, Access = private)
        %ImageDatastore Datastore used to read data.
        ImageDatastore
      
        %UsingDefaultReadFcn True if the default read function is used.
        UsingDefaultReadFcn logical
        
        %UndefinedPixelLabel A vector the same length as CategoricalCache.
        %                    This is used to hold a pixel ID to represent
        %                    <undefined> pixels in numeric representations
        %                    of categorical matrices.
        UndefinedPixelLabel
    end
    
    properties(Hidden, Access = private, Transient)
        % IsRGBPixelLabelID  Logical vector indicating whether
        %                    LabelDefinitions{i} uses RGB pixel label IDs.
        IsRGBPixelLabelID
    end
    
    methods(Hidden, Access = private)
        %------------------------------------------------------------------
        function this = PixelLabelDatastore(location, labelDefinitions, varargin)
            
            if isa(location, 'matlab.io.datastore.ImageDatastore')
                % used by partition method. assignments will already be set
                % in input datastore.Labels. Tack on label definitions.
                this.ImageDatastore = location;
                this.LabelDefinitions = labelDefinitions;
                this.CategoricalCache = varargin{1};
                this.isValidCategoricalCache = true;
                initializeIsRGBPixelLabelID(this);
                initializeUndefinedPixelLabel(this);
                updateReadFcnIfRequired(this, this.ImageDatastore.ReadFcn);
            else
                assignments = varargin{1};
                params      = varargin{2};
                try
                    % Restrict formats to imformats
                    
                    if ~isfield(params,'AlternateFileSystemRoots')
                        params.AlternateFileSystemRoots = {};
                    end                                    
                    
                    this.ImageDatastore = imageDatastore(location, ...
                        'IncludeSubfolders', params.IncludeSubfolders, ...
                        'ReadSize', params.ReadSize, ...
                        'FileExtensions', params.FileExtensions,...
                        'AlternateFileSystemRoots', params.AlternateFileSystemRoots);
                    
                catch ME
                    throwAsCaller(ME)
                end
                
                initializeLabelDefinitions(this, labelDefinitions, assignments);
                
                initializeIsRGBPixelLabelID(this);

                initializeCategoricalCache(this);
                
                updateReadFcnIfRequired(this, params.ReadFcn);                
                this.isValidCategoricalCache = true;
            end
            
            
        end
        
        %------------------------------------------------------------------
        function initializeLabelDefinitions(this, labelDefinitions, assignments)
            
            if ~iscell(labelDefinitions)
                labelDefinitions = {labelDefinitions};
            end

            this.LabelDefinitions = labelDefinitions;
            
            % set file assignments to label definitions: assignments(k)
            % maps file(k) to labelDefinitions( assignments(k) ).
            if isscalar(assignments)
                assert(numel(this.LabelDefinitions) == 1);
                assignments = ones(numel(this.ImageDatastore.Files),1);
            end
            this.ImageDatastore.Labels = assignments;
        end
        
        %------------------------------------------------------------------
        function initializeIsRGBPixelLabelID(this)
            % Cache whether ground truth labels are RGB pixel IDs or not.
            numLabelDef = numel(this.LabelDefinitions);
            this.IsRGBPixelLabelID = false(1,numLabelDef);
            for i = 1:numLabelDef
                this.IsRGBPixelLabelID(i) = iIsRGBPixelLabelFromLabelDef(this.LabelDefinitions{i});
            end
        end
        
        %------------------------------------------------------------------
        function initializeCategoricalCache(this)
            this.CategoricalCache = cell(1,numel(this.LabelDefinitions));
            
            % use first label definition to define categorical order
            catord = this.LabelDefinitions{1}.Name;
            
            for i = 1:numel(this.CategoricalCache)
                        
                this.CategoricalCache{i} = ...
                    matlab.io.datastore.PixelLabelDatastore.labeldef2cat(...
                    this.LabelDefinitions{i}, this.IsRGBPixelLabelID(i));

                % Maintain the same categories order across label
                % definitions.
                this.CategoricalCache{i} = reordercats(...
                    this.CategoricalCache{i}, catord);              
            end
            
            initializeUndefinedPixelLabel(this);
            
        end
        
        %------------------------------------------------------------------
        function initializeUndefinedPixelLabel(this)
            % Determine a pixel value to represent undefined pixels for
            % each image. Use the length of the cache minus 1 (cache starts
            % at 0 ID).
            %
            % The undefined pixel ID is used as a fill value during
            % data augmentation to ensure areas outside the pixel label
            % image are marked as <undefined> when the augmented label
            % matrices are converted back to categorical.
            N = numel(this.CategoricalCache);
            this.UndefinedPixelLabel = zeros(N,1);
            for i = 1:N
                this.UndefinedPixelLabel(i) = numel(this.CategoricalCache{i})-1;
            end
        end
        
        %------------------------------------------------------------------
        function updateReadFcnIfRequired(this, readFcn)

            this.UsingDefaultReadFcn = vision.internal.isDefaultImdsReadFcn(readFcn);

           % update ImageDatastore's read function
            if ~this.UsingDefaultReadFcn             
                this.ImageDatastore.ReadFcn = readFcn;
            end
        end
             
    end
    
    methods(Hidden, Access = protected)
        
        %------------------------------------------------------------------
        function C = validateCategorical(this,C)
            if iscell(C)
                for i = 1:numel(C)
                    C{i} = validateCategoricalMatrix(this,C{i});
                end
            else
                C = validateCategoricalMatrix(this,C);
            end
        end
        
        %------------------------------------------------------------------
        function C = validateCategoricalMatrix(this,C)
            % If the user specified a custom reader, check that the
            % categorical created by it is valid.
            if ~isa(C, 'categorical')
                error(message('vision:semanticseg:invalidCustomReadFcnOutput'));
            end
            
            cats = categories(C);
            % Should have the same categories.
            if ~isempty(setxor(cats, this.ClassNames))
                names = sprintf('%s, ', this.ClassNames{:});
                error(message('vision:semanticseg:unexpectedClassNames', names(1:end-2)));
            end

            % If all the categories are present, but the ordering is different
            % reorder categories.
            if ~isequal(cats, this.ClassNames)
                C = reordercats(C, this.ClassNames);
            end
        end
        
        %------------------------------------------------------------------
        function aCopy = copyElement(this)
            % Return a deep copy of the datastore. The copy is not reset
            % and state is preserved.
            imdsCopy = copy(this.ImageDatastore);
            
            aCopy = matlab.io.datastore.PixelLabelDatastore(...
                imdsCopy, this.LabelDefinitions, this.CategoricalCache);
        end
        
        %------------------------------------------------------------------
        %MAXPARTITIONS Return the maximum number of partitions possible for
        % the datastore.
        %
        %   N = MAXPARTITIONS(DS) returns the maximum number of partitions
        %   for a given Datastore, DS.
        %
        %   See also matlab.io.datastore.Partitionable, numpartitions,
        %   partition.
        function n = maxpartitions(this)            
            n = numpartitions(this.ImageDatastore);
        end
    end
    
    methods
        
        %------------------------------------------------------------------
        function C = readall(this)
            %readall Read all of the files from the datastore.
            %   C = readall(pxds) reads all of the files from pxds. C is a
            %   cell array containing all the pixel label data in pxds.
            
            L = readall(this.ImageDatastore);
                        
            if this.UsingDefaultReadFcn
                iErrorIfNotUint8OrLogicalImage(L);
                info.Label = this.ImageDatastore.Labels;
                C = this.label2categorical(L, info);
            else
                if isInpNumeric(L)
                    % When the custom reader reads 3d data, read function
                    % attempts to convert the input to categorical only if
                    % inputs is uint8 or logical.
                    iErrorIfNotUint8OrLogicalImage(L);
                    info.Label = this.ImageDatastore.Labels;
                    C = this.label2categorical(L, info);
                else
                    % The custom reader is responsible for converting image
                    % data to a categorical. Check that the result is a
                    % categorical and has the expected categories.
                    L = this.validateCategorical(L);
                    C = L;
                end
            end

        end
        
        %------------------------------------------------------------------
        function C = preview(this)
            %preview Read the first image from the datastore.
            %   C = preview(pxds) always reads the first pixel label data
            %   file from the datastore. C is a categorical matrix.
            %
            %   preview does not affect the state of the datastore.
            %
            % Example
            % -------
            % % Location of image and pixel label data
            % dataDir = fullfile(toolboxdir('vision'), 'visiondata');
            % pxDir = fullfile(dataDir, 'buildingPixelLabels');
            %
            % % Create pixel label datastore.
            % classNames = ["sky" "grass" "building" "sidewalk"];
            % pixelLabelID = [1 2 3 4];
            % pxds = pixelLabelDatastore(pxDir, classNames, pixelLabelID);
            %            
            % C = read(pxds);             
            % rgb = label2rgb(uint8(C{1}));
            % figure
            % imshow(rgb);                
           
            C = readimage(this,1);                       
        end
        
        %------------------------------------------------------------------
        function TF = hasdata(this)
            %hasdata Returns true if there is unread data in the PixelLabelDatastore.
            %   TF = hasdata(pxds) returns true if the datastore has one or
            %   more images available to read with the read method.
            %   read(pxds) returns an error when hasdata(pxds) returns
            %   false.
            %
            % Example
            % -------
            % % Location of image and pixel label data
            % dataDir = fullfile(toolboxdir('vision'), 'visiondata');
            % pxDir = fullfile(dataDir, 'buildingPixelLabels');
            %
            % % Create pixel label datastore.
            % classNames = ["sky" "grass" "building" "sidewalk"];
            % pixelLabelID = [1 2 3 4];
            % pxds = pixelLabelDatastore(pxDir, classNames, pixelLabelID);
            %
            % while hasdata(pxds)
            %     C = read(pxds);             % Read data
            % end
            % reset(pxds);                    % Reset to beginning.
            % C = read(pxds);                 % Read from the beginning.
            
            TF = this.ImageDatastore.hasdata();
        end
                        
        %------------------------------------------------------------------
        function subds = partition(this, varargin)
            %partition Returns a partitioned portion of the PixelLabelDatastore.
            %   subds = partition(pxds, N, index) partitions pxds into N
            %   parts and returns the partitioned PixelLabelDatastore,
            %   subds, corresponding to index. An estimate for a reasonable
            %   value for N can be obtained by using the NUMPARTITIONS
            %   function.
            %
            %   subds = partition(pxds,'Files',index) partitions pxds by
            %   files in the Files property and returns the partition
            %   corresponding to index.
            %
            %   subds = partition(pxds,'Files',filename) partitions pxds by
            %   files and returns the partition corresponding to filename.
            
            try
                subimds = partition(this.ImageDatastore, varargin{:});
                subds = matlab.io.datastore.PixelLabelDatastore(subimds, this.LabelDefinitions, this.CategoricalCache);
            catch ME
                throwAsCaller(ME)
            end
        end                
        
        %------------------------------------------------------------------
        function [C, info] = read(this)
            %read Read the next pixel label data file from the datastore.
            %   C = read(pxds) reads the next consecutive image from pxds.
            %   C is a cell array of categorical matrices, where C{k}(i,j)
            %   defines a categorical label.
            %
            %   read(pxds) errors if there is not data in pxds and should
            %   be used with hasdata(pxds).
            %
            %   [C, info] = read(pxds) also returns a structure with
            %   additional information about C. The fields of info are:
            %      Filename - Name of the file from which the data was read
            %      FileSize - Size of the file in bytes
            %      
            %   When the ReadSize property of PixelLabelDatastore is
            %   greater than 1, the fields of info are:            
            %      Filename - A cell array of filenames
            %      FileSize - A vector of file sizes           
            %
            % Example
            % -------
            %   % Location of image and pixel label data
            %   dataDir = fullfile(toolboxdir('vision'), 'visiondata');
            %   imDir = fullfile(dataDir, 'building');
            %   pxDir = fullfile(dataDir, 'buildingPixelLabels');
            %
            %   % Create image datastore
            %   imds = imageDatastore(imDir);
            %
            %   % Create pixel label datastore.
            %   classNames = ["sky" "grass" "building" "sidewalk"];
            %   pixelLabelID = [1 2 3 4];
            %   pxds = pixelLabelDatastore(pxDir, classNames, pixelLabelID);
            %
            %   % Read image and pixel label data.
            %   I = read(imds);
            %   C = read(pxds);
            %
            %   % Display the label categories in C
            %   categories(C{1})
            %
            %   % Overlay pixel label data on the image and display.
            %   B = labeloverlay(I, C{1});
            %   figure
            %   imshow(B)
            
            % Call read on imds. This calls either the default ReadFcn or
            % a custom one.
            [C, info] = read(this.ImageDatastore);            
            C = iWrapInCellIfRequired(C);
            
            if this.UsingDefaultReadFcn    
                iErrorIfNotUint8OrLogicalImage(C);            
                C = this.label2categorical(C, info);
            else 
                if isInpNumeric(C)
                    % When the custom reader reads 3d grayscale data, read function
                    % attempts to convert the input to categorical only if
                    % inputs is uint8 or logical.
                    iErrorIfNotUint8OrLogicalImage(C);
                    C = this.label2categorical(C, info);                  
                else
                    % The custom reader is responsible for converting image
                    % data to a categorical. Check that the result is a
                    % categorical and has the expected categories.
                    C = this.validateCategorical(C);
                end
            end
            
            if nargout == 2
                info = rmfield(info, 'Label');
            end
        end
        
        %------------------------------------------------------------------
        function [C, info] = readimage(this,i)
            %readimage Read a specified pixel label data file from the datastore.
            %   C = readimage(pxds,k) reads the k-th file from pxds.
            %   By default, C is a categorical matrix where each element
            %   C(i,j) defines a categorical label.               
            %
            %   [C, info] = readimage(pxds, k) also returns a structure with
            %   additional information about C. The fields of info are:
            %      Filename - Name of the file from which the data was read
            %      FileSize - Size of the file in bytes
            %                  
            % Example
            % -------
            %   % Location of image and pixel label data
            %   dataDir = fullfile(toolboxdir('vision'), 'visiondata');          
            %   pxDir = fullfile(dataDir, 'buildingPixelLabels');                       
            %
            %   % Create pixel label datastore.
            %   classNames = ["sky" "grass" "building" "sidewalk"];
            %   pixelLabelID = [1 2 3 4];
            %   pxds = pixelLabelDatastore(pxDir, classNames, pixelLabelID);
            %          
            %   C = readimage(pxds,2);
            %             
            %   L = uint8(C);
            %   rgb = label2rgb(L);
            %   figure
            %   imshow(rgb)
            
            validateattributes(i, {'numeric'},...
                {'vector', 'positive', '<=', numel(this.ImageDatastore.Files)}, ...
                mfilename, 'i')
            
            if isscalar(i)
                % Call readimage on imds. This calls either the default ReadFcn
                % or a custom one.
                [C, info] = readimage(this.ImageDatastore,i);
                
                if this.UsingDefaultReadFcn
                    iErrorIfNotUint8OrLogicalImage(C);
                    C = this.label2categorical(C, info);
                else
                    if isInpNumeric(C)
                        % When the custom reader reads 3d grayscale data, the read
                        % function attempts to convert the input to categorical
                        % only if inputs is uint8 or logical.
                        iErrorIfNotUint8OrLogicalImage(C);
                        C = this.label2categorical(C, info);
                    else
                        % The custom reader is responsible for converting image
                        % data to a categorical. Check that the result is a
                        % categorical and has the expected categories.
                        C = this.validateCategorical(C);
                    end
                end
                
                if nargout == 2
                    info = rmfield(info, 'Label');
                end
            else
                % Create datastore partition via a copy and index. This is
                % faster than constructing a new datastore with the new
                % files.
                subds = copy(this.ImageDatastore);
                subds.Files = this.ImageDatastore.Files(i);
                subds.Labels = this.ImageDatastore.Labels(i);
                C = readall(subds);       
                                               
                info.Label = subds.Labels;
                
                if this.UsingDefaultReadFcn  
                    iErrorIfNotUint8OrLogicalImage(C);                  
                    C = this.label2categorical(C, info);
                else
                    if isInpNumeric(C)
                        % When the custom reader reads 3d data, read function
                        % attempts to convert the input to categorical only if
                        % inputs is uint8 or logical.
                        iErrorIfNotUint8OrLogicalImage(C);
                        info.Label = this.ImageDatastore.Labels;
                        C = this.label2categorical(C, info);                        
                    else
                        % The custom reader is responsible for converting image
                        % data to a categorical. Check that the result is a
                        % categorical and has the expected categories.
                        C = this.validateCategorical(C);
                    end
                end
               
            end
        end
        
        %------------------------------------------------------------------
        function reset(this)
            %reset Reset the datastore to the start of the data.
            %   reset(pxds) resets pxds to the beginning of the datastore.
            %
            % Example
            % -------
            % % Location of image and pixel label data
            % dataDir = fullfile(toolboxdir('vision'), 'visiondata');
            % pxDir = fullfile(dataDir, 'buildingPixelLabels');
            %
            % % Create pixel label datastore.
            % classNames = ["sky" "grass" "building" "sidewalk"];
            % pixelLabelID = [1 2 3 4];
            % pxds = pixelLabelDatastore(pxDir, classNames, pixelLabelID);
            %
            % while hasdata(pxds)
            %     C = read(pxds);               % Read data
            %     rgb = label2rgb(uint8(C{1})); % Convert pixel labels to RGB image.
            % end
            % reset(pxds);                    % Reset to beginning.
            % C = read(pxds);                 % Read from the beginning.
            
            this.ImageDatastore.reset();
        end

        %------------------------------------------------------------------
        function files = get.Files(this)
            files = this.ImageDatastore.Files;
        end
        
        %------------------------------------------------------------------
        function set.Files(~, ~)
            % Setting the Files will invalidate LabelDefinitions.
            error(message('vision:semanticseg:pxdsFilesNotSettable'));
        end
        
        %------------------------------------------------------------------
        function sz = get.ReadSize(this)
            sz = this.ImageDatastore.ReadSize;
        end
        
        %------------------------------------------------------------------
        function set.ReadSize(this, sz)
            this.ImageDatastore.ReadSize = sz;
        end
        
        %------------------------------------------------------------------
        function fcn = get.ReadFcn(this)            
            fcn = this.ImageDatastore.ReadFcn;
        end
        
        %------------------------------------------------------------------
        function set.ReadFcn(this,fcn)   
            try
                this.updateReadFcnIfRequired(fcn);
            catch ME
                throwAsCaller(ME);
            end
                
        end
        
        %------------------------------------------------------------------
        function names = get.ClassNames(this)
            names = this.LabelDefinitions{1}.Name;
        end
        
        %------------------------------------------------------------------
        function altpath = get.AlternateFileSystemRoots(this)
            altpath = this.ImageDatastore.AlternateFileSystemRoots;
        end
         
        %------------------------------------------------------------------
        function set.AlternateFileSystemRoots(this, altpath)   
            try
                % Note: setting altroot resets the datastore.
                this.ImageDatastore.AlternateFileSystemRoots = altpath;
            catch ME
                throwAsCaller(ME)
            end
        end

        %------------------------------------------------------------------
        function s = saveobj(this)
            s.ImageDatastore      = saveobj(this.ImageDatastore);
            s.LabelDefinitions    = this.LabelDefinitions;
            s.CategoricalCache    = this.CategoricalCache;
            s.UndefinedPixelLabel = this.UndefinedPixelLabel;
            s.Version             = 2;
        end

        %------------------------------------------------------------------
        function tbl = countEachLabel(this)
            %countEachLabel Counts the number of pixel labels for each class.
            %
            % tbl = countEachLabel(pxds) counts the occurrence of each
            % pixel label for all images represented by pxds. The output
            % tbl is a table with the following variables names:
            %
            %   Name            - The pixel label class name.
            %
            %   PixelCount      - The number of pixels of a given class.
            %
            %   ImagePixelCount - The total number of pixels in images that
            %                     had an instance of the given class.
            %
            % Class Balancing
            % ---------------
            % The output of countEachLabel can be used to calculate class
            % weights for class balancing, for example:
            %
            %   * Uniform class balancing weights each class such that each
            %     has a uniform prior probability:
            %
            %        numClasses = height(tbl)
            %        prior = 1/numClasses;
            %        classWeights = prior./tbl.PixelCount
            %
            %   * Inverse frequency balancing weights each class such that
            %     underrepresented classes are given higher weight:
            %
            %        totalNumberOfPixels = sum(tbl.PixelCount)
            %        frequency = tbl.PixelCount / totalNumberOfPixels;
            %        classWeights = 1./frequency
            %
            %   * Median frequency balancing weights each class using the
            %     median frequency. The weight for each class c is defined
            %     as median(imageFreq)/imageFreq(c) where imageFreq(c) is
            %     the number of pixels of a given class divided by the
            %     total number of pixels in images that had a instance of
            %     the given class c.
            %
            %        imageFreq = tbl.PixelCount ./ tbl.ImagePixelCount
            %        classWeights = median(imageFreq) ./ imageFreq
            %
            % The calculated class weights can be passed to the
            % pixelClassificationLayer. See example below.
            %
            % Example
            % --------
            % % Setup of data location.
            % dataDir = fullfile(toolboxdir('vision'), 'visiondata');
            % imDir = fullfile(dataDir, 'building');
            % pxDir = fullfile(dataDir, 'buildingPixelLabels');
            %
            % % Create pixel label datastore.
            % classNames = ["sky" "grass" "building" "sidewalk"];
            % pixelLabelID = [1 2 3 4];
            % pxds = pixelLabelDatastore(pxDir, classNames, pixelLabelID);
            %
            % % Tabulate pixel label counts in dataset.
            % tbl = countEachLabel(pxds)
            %
            % % Class balancing using uniform prior weighting.
            % prior = 1/numel(classNames);
            % uniformClassWeights = prior./tbl.PixelCount
            %
            % % Class balancing using inverse frequency weighting.
            % totalNumberOfPixels = sum(tbl.PixelCount);
            % frequency = tbl.PixelCount / totalNumberOfPixels;
            % invFreqClassWeights = 1./frequency
            %
            % % Class balancing using median frequency weighting.
            % freq = tbl.PixelCount ./ tbl.ImagePixelCount
            % medFreqClassWeights = median(freq) ./ freq
            %
            % % Pass the class weights to the pixel classification layer.
            % layer = pixelClassificationLayer('ClassNames', tbl.Name, ...
            %     'ClassWeights', medFreqClassWeights)
            %
            % See also pixelClassificationLayer, pixelLabelDatastore,
            %          imageDatastore.
        
            % Make a copy so we do not dirty the state.
            pxdsCopy = copy(this);
            pxdsCopy.reset();
            
            % Read in batches to improve performance.
            pxdsCopy.ReadSize = 64; 
            
            counts = zeros(1,numel(this.ClassNames));
            N      = zeros(1,numel(this.ClassNames));
            while hasdata(pxdsCopy)
                C = read(pxdsCopy);
                for i = 1:numel(C)
                    % Vectorize C to handle multi-dimensional categorical
                    % arrays
                    countMatrix = countcats(C{i}(:));
                    
                    tmp = sum(countMatrix,2)';
                    counts = counts + tmp;
                    
                    Q = repelem(numel(C{i}), 1, numel(this.ClassNames));
                    idx = find(tmp > 0);
                    
                    % accumulate number of total pixels in images that have
                    % class idx
                    N(idx) = N(idx) + Q(idx);
                    
                end
            end                     
            
            tbl = table();
            tbl.Name            = this.ClassNames;
            tbl.PixelCount      = counts';
            tbl.ImagePixelCount = N';            
        end
    end

    methods
        function newds = subset(this,ord)
            %subset Returns a subset of the PixelLabelDatastore.
            %   subds = subset(pxds, indices) partitions pxds into the number
            %   of files in the datastore and returns a subset containing
            %   the files corresponding to indices.
            %
            %   Example
            %   -------
            %      % Location of image and pixel label data
            %      dataDir = fullfile(toolboxdir('vision'), 'visiondata');
            %      pxDir = fullfile(dataDir, 'buildingPixelLabels');
            %
            %      % Create pixel label datastore.
            %      classNames = ["sky" "grass" "building" "sidewalk"];
            %      pixelLabelID = [1 2 3 4];
            %      pxds = pixelLabelDatastore(pxDir, classNames, pixelLabelID);
            %
            %      % subds contains the first 5 files of the training data.
            %      subds = subset(pxds,1:5);
            %
            %      % If not empty, read the data represented by subds
            %      while hasdata(subds)
            %         % Read one row of pixel labels at a time
            %         pixelLabels = read(subds);
            %      end
            %
            %   See also boxLabelDatastore, matlab.io.datastore.mixin.Subsettable

            try
                newds = copy(this);
                newds.reset();
                newds.ImageDatastore = subset(newds.ImageDatastore,ord);
            catch ME
                throwAsCaller(ME)
            end
        end
    end

    methods(Hidden)
        
        %------------------------------------------------------------------
        function n = numobservations(this)
            %NUMOBSERVATIONS   The number of observations in pixelLabelDatastore.
            %
            %   N = NUMOBSERVATIONS(DS) returns the number of observations in
            %   this pixelLabelDatastore. This is equal to the number of files
            %   in the datastore i.e., numel(DS.Files).
            %
            %   Example
            %   -------
            %   % Location of image and pixel label data
            %   dataDir = fullfile(toolboxdir('vision'), 'visiondata');
            %   pxDir = fullfile(dataDir, 'buildingPixelLabels');
            %
            %   % Create pixel label datastore.
            %   classNames = ["sky" "grass" "building" "sidewalk"];
            %   pixelLabelID = [1 2 3 4];
            %   pxds = pixelLabelDatastore(pxDir, classNames, pixelLabelID);
            %
            %   % Find the total number of observations in the datastore.
            %   N = numobservations(pxds)
            %
            %   See also boxLabelDatastore, matlab.io.datastore.mixin.Subsettable
            
            % Each file in an ImageDatastore is a single individual observation.
            % Therefore the total number of observations is equal to the
            % total number of files in the ImageDatastore.
            n = numpartitions(this.ImageDatastore);
        end
        
        %------------------------------------------------------------------
        function initializeDatastore(this, info)
            %INITIALIZEDATASTORE Initialize the datastore with necessary
            %   split information sent from Hadoop.
            %   initializeDatastore(DS, INFO) initializes the datastore with
            %   necessary information sent from Hadoop.
            %   The info struct consists of the following fields -
            %     1) FileName,
            %     2) Offset, and
            %     3) Size.
            %   The FileName field is of type char, and the fields Offset and
            %   Size are of type double.                       
            initFromFileSplit(this.ImageDatastore, info.FileName, info.Offset, info.Size);
        end
        
        %------------------------------------------------------------------
        function location = getLocation(this)
            %GETLOCATION Return the location of the files in Hadoop.
            %   LOCATION = getLocation(DS) returns the location of the files
            %   in Hadoop to which this datastore points.
            location = this.Files;
        end
        
        %------------------------------------------------------------------
        function tf = isfullfile(~)
            %ISFULLFILE Return whether datastore supports full file or not.
            %   TF = isfullfile(DS) returns a logical indicating whether or
            %   not initializeDatastore method must get information for one
            %   complete file.            
            tf = true;
        end                
        
        %------------------------------------------------------------------
        % Hidden method used for reading just the data. 
        function [L,info] = readNumeric(this, indices)
            if nargin == 1
                [L,info] = read(this.ImageDatastore);
            else
                % Create datastore partition via a copy and index. This is
                % faster than constructing a new datastore with the new
                % files.
                subds = copy(this.ImageDatastore);
                subds.Files = this.ImageDatastore.Files(indices);
                subds.Labels = this.ImageDatastore.Labels(indices);
                L = readall(subds);
                info.Filename = subds.Files;
                info.Label = subds.Labels;
            end                        
            
            % Determine a pixel value to represent undefined pixels for
            % each image. Use the length of the cache minus 1 (cache starts
            % at 0 ID).
            %
            % The undefined pixel ID is used as a fill value during
            % data augmentation to ensure areas outside the pixel label
            % image are marked as <undefined> when the augmented label
            % matrices are converted back to categorical.
            undefined = this.UndefinedPixelLabel(info.Label);
            
            % Return undefined pixel value in info struct. This is read in
            % pixelLabelImageDatastore.
            info.FillValue = undefined;
            
            if this.UsingDefaultReadFcn                
                iErrorIfNotUint8OrLogicalImage(L);
            else
                % Allowing customReadFcn to return numeric values, lets
                % readNumeric return underlying numeric data without
                % another conversion to single.
                if isInpNumeric(L)
                    iErrorIfNotUint8OrLogicalImage(L);
                else
                    % The custom reader is responsible for converting image
                    % data to a categorical. Check that the result is a
                    % categorical and has the expected categories.
                    L = this.validateCategorical(L);
                    
                    % Converting categorical to single invalidates the
                    % CategoricalCache
                    this.isValidCategoricalCache = false;
                    % Convert categorical to single and fill missing (NaN)
                    % values with the undefined pixel ID found above.
                    if iscell(L)
                        for i = 1:numel(L)
                            L{i} = single(L{i}); % <undefined> categorical become NaN
                            L{i}(isnan(L{i})) = undefined(i);
                        end
                    else
                        L = single(L); % <undefined> categorical become NaN
                        L(isnan(L)) = undefined;
                    end
                end
            end
            
        end
        
        %------------------------------------------------------------------
        % Hidden method used to remove files. Used in datastore 
        % distribute.
        function removeFiles(this, toRemove)
            this.ImageDatastore.Files(toRemove) = [];
        end
        
        %------------------------------------------------------------------
        function L = label2categorical(this, L, info)
            % Input L is a cell array of label matrices or a single label
            % matrix. L may be a categorical, which case this function does
            % nothing. 
            % 
            % The output is a cell array of categoricals or a single
            % categorical matrix.
            if this.isValidCategoricalCache && iscell(L)
                for i = 1:numel(L)
                    if ~isa(L{i},'categorical')
                        isRGBPixelID = this.IsRGBPixelLabelID(info.Label(i));
                        L{i} = matlab.io.datastore.PixelLabelDatastore.label2cat(...
                            L{i}, this.CategoricalCache{info.Label(i)}, isRGBPixelID);
                    end
                end
            elseif this.isValidCategoricalCache && ~iscell(L)
                if ~isa(L,'categorical')
                    isRGBPixelID = this.IsRGBPixelLabelID(1); 
                    L = matlab.io.datastore.PixelLabelDatastore.label2cat(...
                        L, this.CategoricalCache{info.Label(1)}, isRGBPixelID);
                end
            else

                % When a user defined ReadFcn returning categoricalData is
                % used, the categorical cache is not valid because the
                % original label matrix's value set is replaced by the
                % categorical IDs (i.e. 1:numel(this.ClassNames)).
                %
                % Convert label matrix into categorical using defined
                % ClassNames.              
                if iscell(L)
                    for i = 1:numel(L)
                        if ~isa(L{i},'categorical')
                            L{i} = categorical(L{i},1:numel(this.ClassNames),this.ClassNames);
                        end
                    end
                else
                    if ~isa(L,'categorical')
                        L = categorical(L,1:numel(this.ClassNames),this.ClassNames);
                    end
                end
            end
        end
    
        %------------------------------------------------------------------
        function frac = progress(this)
            frac = progress(this.ImageDatastore);
        end
                
    end
    
    methods(Hidden, Static)
        %------------------------------------------------------------------
        function this = loadobj(s)
            imds = matlab.io.datastore.ImageDatastore.loadobj(s.ImageDatastore);
            this = matlab.io.datastore.PixelLabelDatastore(imds, s.LabelDefinitions, s.CategoricalCache);
            if s.Version < 2
                % reinitialize categorical cache to account for new
                % undefined pixel ID.
                initializeCategoricalCache(this);
            end
        end
        
        %------------------------------------------------------------------
        function c = labeldef2cat(labeldef, isRGBPixelLabelID)
            % convert label definition table into categorical.
            classset = labeldef.Name;
            valueset = labeldef.PixelLabelID;
            classes = cell(numel(classset),1);
           
            if isRGBPixelLabelID
                % RGB label matrix
                valueset = matlab.io.datastore.PixelLabelDatastore.rgb2labelID(valueset);
                
                % create a 24-bit look-up table (24 bits are all that's
                % required to store 3 uint8 pixel values). This consumes 64
                % MB, but allows for fast label to categorical conversion.
                %
                % Allow for an extra slot at the end to represent undefined
                % values. This value (2^24 is used as a fill value in
                % pre-processing and ensures conversion from label matrix
                % to categorcial is consistent via the cache.
                L = single(0):single(2^24); 
            else                
                L = single(0):single(255+1);
            end
            
            % replicate class names for each value
            for j = 1:numel(valueset)
                values = valueset{j};
                classes(j) = {repelem(classset(j), numel(values), 1)};
            end
            
            valueset = vertcat(valueset{:});
            classes  = vertcat(classes{:});
            
            c = categorical(L, single(valueset), classes);
            
        end
        
        %------------------------------------------------------------------
        function C = label2cat(L, categoricalCache, isRGBPixelLabelID)
            % Convert a label matrix into a categorical.
                           
            if isRGBPixelLabelID
                % Label matrix must have 3 channels when RGB pixel label
                % IDs are used.
                if size(L,3) ~= 3
                    error(message('vision:semanticseg:expectedRGBLabels'));
                end
                L = matlab.io.datastore.PixelLabelDatastore.rgb2labelmatrix(L);                                               
            end                        
            
            % Index into cache.
            C = categoricalCache(single(L)+1);
            
            if isvector(L)
                % The cache is stored in a row vector. When L is a column
                % vector, indexed expansion makes C a row vector. Reshape C
                % to ensure size(C) == size(L);
                C = reshape(C, size(L));
            end
        end
        
        %------------------------------------------------------------------
        function L = rgb2labelmatrix(rgb)
            % convert input rgb label matrix to single channel label matrix
            rgb = uint32(rgb);
            rgb(:,:,1) = bitshift(rgb(:,:,1), 16);
            rgb(:,:,2) = bitshift(rgb(:,:,2), 8);
            
            L = bitor( bitor(rgb(:,:,1), rgb(:,:,2)), rgb(:,:,3) );
        end
        
        %------------------------------------------------------------------
        function v = rgb2labelID(rgb)
            % rgb is cell array of M-by-3 matrices. output is cell array of
            % N-by-1 vectors.
            v = cell(numel(rgb), 1);
            for i = 1:numel(rgb)
                
                m = uint32(rgb{i});
                
                % pack uint8 RGB values into uint32 by bit shifting R and B
                % pixel vals.
                m(:,1) = bitshift(m(:,1), 16);
                m(:,2) = bitshift(m(:,2), 8);
                
                v{i} = bitor( bitor(m(:,1), m(:,2)), m(:,3) );
            end
        end
        
        %------------------------------------------------------------------
        function limds = create(location, classes, values, params)
            % limds = create(files, classes, values) returns a
            % PixelLabelDatastore provided a cellstr of filenames
            assert(iscellstr(classes));
            assert(iscell(values));
            labelDefinitions = table(...
                reshape(classes,[],1), ...
                reshape(values,[],1), ...
                'VariableNames', {'Name', 'PixelLabelID'});
            labelDefAssignments = 1;
            limds = matlab.io.datastore.PixelLabelDatastore(...
                location, labelDefinitions, labelDefAssignments, params);
        end
        
        %------------------------------------------------------------------
        function [limds, imds] = createFromGroundTruth(gTruthArray, params)
            % limds = createFromGroundTruth(gTruthArray) returns a
            % PixelLabelDatastore for reading pixel labeled images.
            %
            %  C = read(limds) returns categorical matrix representing the
            %  pixel label data.
            
            src = {};
            
            labelDefAssignments = [];
            numGroundTruth = numel(gTruthArray);
            labelDefinitions = cell(numGroundTruth,1);
            
            % extract all label definition tables.
            for i = 1:numGroundTruth
                % strip out Type column and only keep labelType.PixelLabel
                defs = gTruthArray(i).LabelDefinitions;
                defs(gTruthArray(i).LabelDefinitions.Type ~= labelType.PixelLabel, :) = [];
                defs(:,2) = [];
                labelDefinitions{i} = defs;
            end
            
            % Fill label def assignments. This maps each file to a label
            % definition table. Required if we shuffle all files to find a
            % files label definition table.
            for i = 1:numGroundTruth
                tmp = gTruthArray(i).LabelData.PixelLabelData;
                
                src = [src; tmp]; %#ok<AGROW>
                
                labelDefAssignments = [labelDefAssignments; repelem(i, numel(tmp),1)]; %#ok<AGROW>
                
            end
            
            % remove for missing labels
            missing = strcmp('', src);
            src(missing) = [];
            labelDefAssignments(missing) = [];
            
            % store which groundTruth object corresponds to each image as a Label in
            % the imageDatastore. This is a simple way to map a specific image to it's
            % corresponding groundTruth object.
            limds = matlab.io.datastore.PixelLabelDatastore(src, labelDefinitions, labelDefAssignments, params);
            
            if nargout == 2
                % Construct image datastore to hold groundTruth data source
                isrc = {};
                for i = 1:numGroundTruth
                    isrc = [isrc; gTruthArray(i).DataSource.Source]; %#ok<AGROW>
                end
                
                % remove images that don't have labels.
                isrc(missing) = [];
                
                imds = imageDatastore(isrc, 'ReadFcn', params.ReadFcn, "FileExtensions", params.FileExtensions);
            end
        end
        
        %------------------------------------------------------------------
        function pxds = createFromDirectory(labelMatrixList, labelDefinitions, labelDefAssignments, params)
            % pxds = createFromDirectory(labelMatrixList, labelDefinitions,
            % labelDefAssignments, params) creates a PixelLabelDataStore
            % with per-image definitions indexed by labelDegAssignments in
            % labelDefinitions for all labelMatrices in labelMatrixList.
            % params include options to look in SubFolders and specify
            % ReadSize.
            
            pxds = matlab.io.datastore.PixelLabelDatastore(labelMatrixList, labelDefinitions, labelDefAssignments, params);
        end
        
        %------------------------------------------------------------------
        function imOut = resizeLabelMatrix(L,outputSize)
           
            supportedWithCast = isa(L,'int8') || isa(L,'uint16') || isa(L,'int16');
            supportedType = isa(L,'uint8') || isa(L,'single');
            modeSupported = supportedWithCast || supportedType;
                            
            if supportedWithCast
                L = single(L);
            end
            
            if modeSupported
                imOut = images.internal.builtins.imageStackResizeNearest(L,outputSize);
            else
                imOut = imresize(L,'OutputSize',outputSize,'method','nearest','Antialias',false);
            end
            
        end
                
    end
end

%--------------------------------------------------------------------------
function iErrorIfNotUint8OrLogicalImage(L)
if iscell(L)
    if ~all(cellfun(@(x)isa(x,'uint8')||isa(x,'logical'),L))
        error(message('vision:semanticseg:pixelValueNotUint8'))
    end
else
    if ~(isa(L,'uint8')||isa(L,'logical'))
        error(message('vision:semanticseg:pixelValueNotUint8'));
    end
end
end

%--------------------------------------------------------------------------
function tf = isInpNumeric(C)
 tf = (iscell(C) && (isa(C{1},'numeric') || isa(C{1},'logical'))) || (isa(C,'numeric') || isa(C,'logical'));
end

%--------------------------------------------------------------------------
function C = iWrapInCellIfRequired(C)
if ~iscell(C)
    C = {C};
end
end

%--------------------------------------------------------------------------
function tf = iIsRGBPixelLabelFromLabelDef(labeldef)
valueset = labeldef.PixelLabelID;
tf = size(valueset{1},2) == 3;
end
