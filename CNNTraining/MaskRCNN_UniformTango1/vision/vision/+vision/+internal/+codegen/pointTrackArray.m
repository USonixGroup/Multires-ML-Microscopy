%   This pointTrack class contains code generation implementation of
%   array of vision.internal.pointTrack objects

%   This class is for internal use only.

%   Copyright 2021-2022 The MathWorks, Inc.

%#codegen

classdef pointTrackArray < matlab.mixin.internal.indexing.ParenAssign & ...
        matlab.mixin.internal.indexing.Paren

    properties(Access = protected, Abstract) % For array of pointTrack codegen
        Data
    end

    methods
        %------------------------------------------------------------------
        % Paren reference function for array of pointTrack objects
        %------------------------------------------------------------------
        function this1 = parenReference(this, idx, varargin)

            coder.internal.errorIf(numel(varargin)>0, ...
                                   "vision:pointTrack:oneDimIndexing");
            this1 = vision.internal.codegen.pointTrack.makeEmpty(class(this.Points));

            coder.internal.errorIf(ischar(idx), "vision:pointTrack:oneDimIndexing");
            if ischar(idx) && strcmpi(idx, ':')
                return;
            end

            % Maintain sizing
            if isrow(this)
                dataArray = coder.nullcopy( cell(1, numel(idx)) );
            else
                dataArray = coder.nullcopy( cell(numel(idx), 1) );
            end

            for n = 1 : numel(idx)
                dataArray{n} = this.Data{idx(n)};
            end

            this1.Data   = dataArray;
            this1.Points                = dataArray{1}.Points;
            this1.ViewIds               = dataArray{1}.ViewIds;
            this1.FeatureIndices        = dataArray{1}.FeatureIndices;
        end
        %------------------------------------------------------------------
        % Paren assign function for array of pointTrack objects
        %------------------------------------------------------------------
        function this = parenAssign(this, rhs, idx, varargin)

            coder.internal.errorIf(numel(varargin)>0, ...
                "vision:pointTrack:oneDimIndexing");
            this.checkTypes(rhs);
            if ischar(idx) && strcmp(idx, ':')
                idx = 1 : numel(this);
            elseif ischar(idx)
                coder.internal.error("vision:pointTrack:oneDimIndexing");
            end

            farthestElement = max(idx);

            if farthestElement > numel(this)
                % Copy over current elements
                if isrow(this)
                    dataArray = coder.nullcopy( cell(1, farthestElement) );
                else
                    dataArray = coder.nullcopy( cell(farthestElement, 1) );
                end
                for n = 1 : numel(this)
                    dataArray{n} = this.Data{n};
                end

                % Replace/add new elements
                for n = 1 : numel(idx)
                    dataArray{idx(n)} = rhs.Data{n};
                end
                this.Data = dataArray;
            else
                % No need to grow cell array, just replace data
                for n = 1 : numel(idx)
                    this.Data{idx(n)} = rhs.Data{n};
                end
            end
        end

        %------------------------------------------------------------------
        % Overloading the vertcat functionality
        %------------------------------------------------------------------
        function this = vertcat(this, varargin)

            coder.internal.assert(...
                isa(this, class(this)), ...
                "vision:pointTrack:invalidClass");
            coder.internal.errorIf(...
                (isrow(this) && numel(this)~=1), 'MATLAB:catenate:matrixDimensionMismatch');

            num = numel(varargin);
            for n = 1 : num
                this.checkTypes(varargin{n});
            end

            data = initializeArrayData(this);
            dataArray  = repmat({data}, coder.ignoreConst(0), 1);

            this = copyData(this, dataArray, num, varargin{:});
        end

        %------------------------------------------------------------------
        % Calculate the number of pointTrack in pointTrack array
        %------------------------------------------------------------------
        function n = numel(this)
            n = numel(this.Data);
        end

        %------------------------------------------------------------------
        % Overloading the isscalar functionality
        %------------------------------------------------------------------
        function n = isscalar(this)
            n = numel(this.Data)==1;
        end

        %------------------------------------------------------------------
        % Overloading the repmat functionality
        %------------------------------------------------------------------
        function this = repmat(this, varargin)
            coder.internal.assert( numel(varargin)<3, ...
                "vision:pointTrack:oneDimIndexing");

            % Validate repmat(obj,[a, b, ...]) syntax
            if numel(varargin)==1 && ~isscalar(varargin{1})
                in = varargin{1};

                % Only indexing up to two dimensions
                coder.internal.assert( numel(in)<=2 && ...
                    (in(1)<=1 || in(2)<=1), ...
                    "vision:pointTrack:oneDimIndexing");
            end

            two_scalar_arguments = (numel(varargin)==2 && isscalar(varargin{1}) && isscalar(varargin{2}));
            coder.internal.assert(~two_scalar_arguments || ...
                (varargin{1} <=1 && varargin{2}<=1),...
                'vision:pointTrack:oneDimIndexing');

            this.Data = repmat(this.Data, varargin{:});
        end

        %------------------------------------------------------------------
        % Overloading the horzcat functionality
        %------------------------------------------------------------------
        function this = horzcat(this, varargin)

            coder.internal.assert(...
                isrow(this), 'MATLAB:catenate:matrixDimensionMismatch');
            num = numel(varargin);
            for n = 1 : num
                this.checkTypes(varargin{n});
            end

            data = initializeArrayData(this);
            dataArray  = repmat({data}, 1, coder.ignoreConst(0));

            this = copyData(this, dataArray, num, varargin{:});
        end

        %------------------------------------------------------------------
        % Overloading the transpose functionality
        %------------------------------------------------------------------
        function this = transpose(this)

            % Transpose is not supported for cell arrays in code
            % generation. Use reshape instead for transpose because these
            % are 1-D arrays.
            if isrow(this)
                this.Data = reshape(this.Data, numel(this), 1);
            else
                this.Data = reshape(this.Data, 1, numel(this));
            end
        end

        %------------------------------------------------------------------
        % Overloading the ctranspose functionality
        %------------------------------------------------------------------
        function this = ctranspose(this)

            this = transpose(this);
        end

        %------------------------------------------------------------------
        % Overloading the reshape functionality
        %------------------------------------------------------------------
        function this = reshape(this, varargin)

            coder.internal.assert( numel(varargin)<3, ...
                "vision:pointTrack:oneDimIndexing");

            two_arguments = (numel(varargin) == 2);
            coder.internal.assert(~two_arguments || ...
                ~(isempty(varargin{1}) || isempty(varargin{2})), ...
                'images:geotrans:reshapeWithEmpties');

            this.Data = reshape(this.Data, varargin{:});
        end

        %------------------------------------------------------------------
        % Overloading the isempty functionality
        %------------------------------------------------------------------
        function ie = isempty(this)

            ie = numel(this)== 0;
        end

        %------------------------------------------------------------------
        function n = end(this,varargin)
            % Only 1-D indexing is supported, so end is always numel.
            n = numel(this);
        end

        %------------------------------------------------------------------
        function l = length(this)
            % For a 1-D array, length is numel
            l = numel(this);
        end

        %------------------------------------------------------------------
        function y = isrow(this)
            y = isrow(this.Data);
        end
        %------------------------------------------------------------------
        function this = copyData(this, dataArray, num, varargin)

            % copy over current elements
            for n=1:numel(this)
                dataArray{end+1} = this.Data{n};
            end

            % copy over new elements
            for n=1:num
                for nn = 1:numel(varargin{n})
                    dataArray{end+1} = varargin{n}.Data{nn};
                end
            end

            % Assign dataArray to corresponding data property
            this.Data = dataArray;
        end

        %------------------------------------------------------------------
        function checkTypes(this, tObj)
            coder.internal.errorIf(~isa(tObj, class(this)), ...
                    "vision:pointTrack:invalidClass");
            coder.internal.errorIf(~isa(this.Points, class(tObj.Points)),...
                "vision:pointTrack:differentTypes");
            coder.internal.errorIf(~isa(this.ViewIds, class(tObj.ViewIds)),...
                "vision:pointTrack:differentTypes");
            coder.internal.errorIf(~isa(this.FeatureIndices, class(tObj.FeatureIndices)),...
                "vision:pointTrack:differentTypes");
        end

        %------------------------------------------------------------------
        % initialize the array objects for pointTrack object
        %------------------------------------------------------------------
        function data = initializeArrayData(this)
            % initialize pointTrack array data
            data = vision.internal.pointTrackImpl(this.ViewIds, ...
                this.Points, this.FeatureIndices);
        end
    end
end
