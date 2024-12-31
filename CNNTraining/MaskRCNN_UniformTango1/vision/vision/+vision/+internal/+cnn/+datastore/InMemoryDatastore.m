%InMemoryDatastore A datastore for holding in-memory data.
%   ids = InMemoryDatastore(DATA) creates an InMemoryDatastore object
%   using data provided by DATA. The read method returns READSIZE height
%   of data from the provided DATA. The data is sliced along the first
%   dimension and each individual slice is considered of height 1. This
%   datastore supports any data type.
%
%   [...] = InMemoryDatastore(..., READSIZE) uses READSIZE as the upper
%   limit to read the in-memory data using the read method.
%
%   [...] = InMemoryDatastore(..., FCNCNAME) includes specified function
%   name in the generated error messages.
%
%   InMemoryDatastore properties:
%     InMemoryData       -  All of the data provided to this datastore.
%     ReadSize           -  Upper limit on the height of data returned by
%                           the read method.
%     Height             -  Height of the in-memory data.
%     CurrentHeightIndex -  The index of InMemoryData from which read
%                           method will return data.
%
%   InMemoryDatastore methods:
%     preview       -  Read the first row of the data from the start of
%                      the datastore.
%     read          -  Read data of ReadSize height from the datastore.
%     readall       -  Read all of the in-memory data from the datastore.
%     hasdata       -  Returns true if there is more data in the datastore.
%     reset         -  Reset the datastore to the start of the data.
%     partition     -  Return a new datastore that represents a single
%                      partitioned part of the original datastore.
%     numpartitions -  Return an estimate for a reasonable number of
%                      partitions to use with the partition function for
%                      the given information.
%     combine       -  Form a single datastore from multiple input
%                      datastores.
%     transform     -  Define a function which alters the underlying data
%                      returned by the read() method.
%
%   Example: Use table as in-memory data
%   ------------------------------------
%      t = table({'a','b','c','d','e','f'}');
%      ds = vision.internal.cnn.datastore.InMemoryDatastore(t);
%
%      % Read and see how the data looks for training.
%      read(ds)
%
%   Example: Use cell array as in-memory data
%   -----------------------------------------
%      c = {'a',[1,2],'d',3,'f'}');
%      ds = vision.internal.cnn.datastore.InMemoryDatastore(c);
%
%      % Read and see how the data looks for training.
%      read(ds)
%
%   See also boxLabelDatastore, imageDatastore, objectDetectorTrainingData,
%            trainFasterRCNNObjectDetector, trainYOLOv2ObjectDetector,
%            groundTruthLabeler, videoLabeler, imageLabeler.

% Copyright 2019 The MathWorks, Inc.
classdef InMemoryDatastore < matlab.io.Datastore & ...
        matlab.io.datastore.mixin.Subsettable

    properties
        %INMEMORYDATA All of the data provided to this datastore.
        InMemoryData
        %READSIZE Upper limit height of the data returned by the
        %   read method.
        ReadSize
    end

    properties (SetAccess = private)
        %CURRENTHEIGHTINDEX The index of InMemoryData from which read
        %   method will return READSIZE amount of data.
        CurrentHeightIndex
        %HEIGHT Height of the in-memory data.
        Height
    end

    properties (Access = private)
        DataSubstruct
        InvokingFunctionName
    end

    methods
        function obj = InMemoryDatastore(inMemoryData, readSize, invokingFunctionName)
            narginchk(1,3);
            switch nargin
                case 1
                    readSize = 1;
                    invokingFunctionName = mfilename;
                case 2
                    invokingFunctionName = mfilename;
            end
            obj.InvokingFunctionName = invokingFunctionName;
            obj.ReadSize = readSize;
            obj.InMemoryData = inMemoryData;
            reset(obj);
        end

        function [data, info] = read(obj)
            if ~hasdata(obj)
                error(message('vision:inMemoryDatastore:noMoreData'));
            end
            data = getDataUsingSubstructInfo(obj);
            info.CurrentIndex = obj.CurrentHeightIndex;
            info.ReadSize = obj.ReadSize;
            obj.CurrentHeightIndex = obj.CurrentHeightIndex + obj.ReadSize;
        end

        function tf = hasdata(obj)
            %HASDATA Returns true if there is unread data in the InMemoryDatastore.
            %   TF = HASDATA(DS) returns true if the datastore has more data
            %   available to read with the read method. read(DS) returns an error
            %   when HASDATA(DS) returns false.
            tf = obj.CurrentHeightIndex <= obj.Height;
        end

        function reset(obj)
            obj.CurrentHeightIndex = 1;
            obj.Height = size(obj.InMemoryData, 1);
            setInMemorySubstruct(obj);
        end

        function data = readall(obj)
            data = obj.InMemoryData;
        end

        function data = preview(obj)
            if isempty(obj.InMemoryData)
                data = obj.InMemoryData;
            else
                data = obj.InMemoryData(1,:);
            end
        end

        function subds = partition(obj, N, ii)
            [N,ii] = iValidatePartitionNumericStrategy(N,ii);
            n = maxpartitions(obj);
            % pigeonhole: N(r-1) + 1 objects into N boxes
            import matlab.io.datastore.internal.util.pigeonHole;
            partitionIndices = pigeonHole(N, n);
            partitionIndices = find(partitionIndices == ii);
            inMemoryData = getDataUsingSubstructInfo(obj, partitionIndices);
            subds = vision.internal.cnn.datastore.InMemoryDatastore(inMemoryData, ...
                obj.ReadSize,...
                obj.InvokingFunctionName);
        end

        function subds = subset(obj, indices)
            % Validate the indices input.
            import matlab.io.datastore.internal.validators.validateSubsetIndices;
            indices = validateSubsetIndices(indices, obj.Height, obj.InvokingFunctionName);
            % Return a copy of the datastore with the sliced property set.
            subds = copy(obj);
            subds.InMemoryData = obj.InMemoryData(indices,:);
            reset(subds);
        end

        function N = numobservations(obj)
            N = size(obj.InMemoryData,1);
        end

        function set.InMemoryData(obj, inmemData)
            obj.InMemoryData = inmemData;
            reset(obj);
        end

        function set.ReadSize(obj, rSize)
            classes = {'numeric'};
            attrs = {'>=', 1, 'scalar', 'positive', 'integer', 'nonsparse'};
            validateattributes(rSize, classes, attrs, ...
                obj.InvokingFunctionName, 'ReadSize');
            obj.ReadSize = rSize;
        end

        function s = saveobj(obj)
            s.InMemoryData         = obj.InMemoryData;
            s.ReadSize             = obj.ReadSize;
            s.CurrentHeightIndex   = obj.CurrentHeightIndex;
            s.InvokingFunctionName = obj.InvokingFunctionName;
            s.Version              = 1;
        end

        function frac = progress(obj)
            if hasdata(obj)
                frac = (obj.CurrentHeightIndex - 1)/obj.Height;
            else
                frac = 1.0;
            end
        end

    end

    methods(Access = protected)
        function N = maxpartitions(obj)
            %MAXPARTITIONS Return the maximum number of partitions
            %   possible for the InMemoryDatastore.
            %
            %   N = MAXPARTITIONS(DS) returns the maximum number of
            %   partitions for a given InMemoryDatastore, DS. This
            %   number is always the height of the in-memory data.

            N = obj.Height;
        end
    end

    methods (Access = private)
        % Set the private substruct
        %
        % See also read, getDataUsingSubstructInfo
        function setInMemorySubstruct(obj)
            numDims = ndims(obj.InMemoryData);
            % colon : for all non-ReadSize dimensions
            col = repmat({':'}, 1, numDims - 1);
            obj.DataSubstruct = substruct('()', [{[]}, col]);
        end

        % Get data using the private substruct and buffered size values
        % This is called after setting BufferedSubstruct
        %
        % See also read, setInMemoryDataInfo
        function data = getDataUsingSubstructInfo(obj, indices)
            if nargin == 1
                % Set the subs field value to the readsize amount
                endIdx = min(obj.Height, obj.CurrentHeightIndex + obj.ReadSize - 1);
                indices = obj.CurrentHeightIndex:endIdx;
            end
            obj.DataSubstruct.subs{1} = indices;
            data = subsref(obj.InMemoryData, obj.DataSubstruct);
        end

    end

    methods (Static, Hidden)
        function obj = loadobj(s)
            import vision.internal.cnn.datastore.InMemoryDatastore;
            obj = InMemoryDatastore(s.InMemoryData, s.ReadSize,...
                s.InvokingFunctionName);
            % Do not set the CurrentHeightIndex from the saved struct,
            % which would set the state of the datastore from the saved state.
            % This is because currently all datastores reset on load.
        end
    end
end

%--------------------------------------------------------------------------
function numeric = iValidateNumericValue(numeric, maxValue, funcName, varargin)
    classes  = {'numeric'};
    attrs    = {'scalar', 'nonempty', 'nonsparse',...
        'nonzero','nonnan','finite', 'integer', ...
        'nonnegative',... % this covers non complex
        '<=', maxValue};

    validateattributes(numeric, classes, attrs, funcName, varargin{:});

    numeric = double(numeric);
end

%--------------------------------------------------------------------------
function [N,ii] = iValidatePartitionNumericStrategy(N,ii)
    varName  = 'NumPartitions';
    argIndex = 2;
    N = iValidateNumericValue(N,realmax('double'),mfilename,varName,argIndex);

    varName  = 'Index';
    argIndex = 3;
    ii = iValidateNumericValue(ii,N,mfilename,varName,argIndex);
end
