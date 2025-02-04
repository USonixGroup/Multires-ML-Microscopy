%   This cameraIntrinsics class contains code generation implementation of
%   cameraIntrinsics and array of cameraIntrinsics object

%   This class is for internal use only.

%   Copyright 2021 The MathWorks, Inc.

%#codegen
classdef cameraIntrinsics < vision.internal.cameraIntrinsicsImpl & ...
        matlab.mixin.internal.indexing.ParenAssign & ...
        matlab.mixin.internal.indexing.Paren

    properties(Access = private, Hidden) % For array of cameraIntrinsics codegen
        cameraIntrinsicsArrayData
    end

    properties (Constant, Hidden)
        ClassName = 'cameraIntrinsics';
    end
    methods
        %------------------------------------------------------------------
        % Constructor
        %------------------------------------------------------------------
        function this = cameraIntrinsics(varargin)
            this = this@vision.internal.cameraIntrinsicsImpl(varargin{:});
            this = initializeCameraIntrinsicsArrayData(this);
        end
    end

    methods(Access = public, Hidden)
        %------------------------------------------------------------------
        % Paren assign function for array of camera Intrinsics objects
        %------------------------------------------------------------------
        function this = parenAssign(this, rhs, idx, varargin)

            coder.internal.errorIf(numel(varargin)>0, ...
                                   "vision:cameraIntrinsics:oneDimIndexing");
            this.checkTypes(rhs);
            if ischar(idx) && strcmp(idx, ':')
                idx = 1 : numel(this);
            elseif ischar(idx)
                coder.internal.errorIf(true, "vision:cameraIntrinsics:oneDimIndexing");
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
                    dataArray{n} = this.cameraIntrinsicsArrayData{n};
                end

                % Replace/add new elements
                for n = 1 : numel(idx)
                    dataArray{idx(n)} = rhs.cameraIntrinsicsArrayData{n};
                end
                this.cameraIntrinsicsArrayData = dataArray;
            else
                % No need to grow cell array, just replace data
                for n = 1 : numel(idx)
                    this.cameraIntrinsicsArrayData{idx(n)} = rhs.cameraIntrinsicsArrayData{n};
                end
            end
        end

        %------------------------------------------------------------------
        % Paren reference function for array of camera Intrinsics objects
        %------------------------------------------------------------------
        function this1 = parenReference(this, idx, varargin)

            coder.internal.errorIf(numel(varargin)>0, ...
                                   "vision:cameraIntrinsics:oneDimIndexing");
            this1 = this.makeEmpty(class(this.FocalLength), class(this.PrincipalPoint) ,...
                class(this.ImageSize), class(this.RadialDistortion), ...
                class(this.TangentialDistortion), class(this.Skew));

            if ischar(idx) && strcmpi(idx, ':')
                return;
            elseif ischar(idx)
                coder.internal.errorIf(true, "vision:cameraIntrinsics:oneDimIndexing");
            end

            % Maintain sizing
            if isrow(this)
                dataArray = coder.nullcopy( cell(1, numel(idx)) );
            else
                dataArray = coder.nullcopy( cell(numel(idx), 1) );
            end
            
            for n = 1 : numel(idx)
                dataArray{n} = this.cameraIntrinsicsArrayData{idx(n)};
            end

            this1.cameraIntrinsicsArrayData    = dataArray;
            this1.FocalLength                  = dataArray{1}.FocalLength;
            this1.PrincipalPoint               = dataArray{1}.PrincipalPoint;
            this1.ImageSize                    = dataArray{1}.ImageSize;
            this1.RadialDistortion             = dataArray{1}.RadialDistortion;
            this1.TangentialDistortion         = dataArray{1}.TangentialDistortion;
            this1.Skew                         = dataArray{1}.Skew;
        end

        %------------------------------------------------------------------
        % Calculate the number of pointTracks in camera Intrinsics array
        %------------------------------------------------------------------
        function n = numel(this)
            n = numel(this.cameraIntrinsicsArrayData);
        end

        %------------------------------------------------------------------
        % Overloading the isscalar functionality
        %------------------------------------------------------------------
        function n = isscalar(this)
            n = numel(this.cameraIntrinsicsArrayData)==1;
        end

        %------------------------------------------------------------------
        % Overloading the repmat functionality
        %------------------------------------------------------------------
        function this = repmat(this, varargin)
            coder.internal.assert( numel(varargin)<3, ...
                                   "vision:cameraIntrinsics:oneDimIndexing");

            % Validate repmat(obj,[a, b, ...]) syntax
            if numel(varargin)==1 && ~isscalar(varargin{1})
                in = varargin{1};

                % Only indexing up to two dimensions
                coder.internal.assert( numel(in)<=2 && ...
                                       (in(1)<=1 || in(2)<=1), ...
                                       "vision:cameraIntrinsics:oneDimIndexing");
            end

            if numel(varargin)==2 && isscalar(varargin{1}) && isscalar(varargin{2})
                coder.internal.assert( ((varargin{1}<=1) && (varargin{2}<=1)), ...
                                       "vision:cameraIntrinsics:oneDimIndexing");
            end
            this.cameraIntrinsicsArrayData = repmat(this.cameraIntrinsicsArrayData, varargin{:});
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

            % Initialize pointTrackArray data
            data       = vision.internal.codegen.cameraIntrinsicsArray(this);
            % Assign dataArray to corresponding Data property
            dataArray  = repmat({data}, 1, coder.ignoreConst(0));

            % Copy over current elements
            for n=1:numel(this)
                dataArray{end+1} = this.cameraIntrinsicsArrayData{n};
            end

            % Copy over new elements
            for n=1:num
                for nn = 1:numel(varargin{n})
                    dataArray{end+1} = varargin{n}.cameraIntrinsicsArrayData{nn};
                end
            end
            
            this.cameraIntrinsicsArrayData = dataArray;
        end

        %------------------------------------------------------------------
        % Overloading the vertcat functionality
        %------------------------------------------------------------------
        function this = vertcat(this, varargin)

            coder.internal.assert(...
                isa(this, class(this)), ...
                "vision:cameraIntrinsics:invalidClass");
            coder.internal.errorIf(...
                isrow(this), 'MATLAB:catenate:matrixDimensionMismatch');

            num = numel(varargin);
            for n = 1 : num
                this.checkTypes(varargin{n});
            end

            % Initialize  pointTrackArray data
            data       = vision.internal.codegen.cameraIntrinsicsArray(this);
            dataArray  = repmat({data}, coder.ignoreConst(0), 1);

            % Copy over current elements
            for n=1:numel(this)
                dataArray{end+1} = this.cameraIntrinsicsArrayData{n};
            end

            % Copy over new elements
            for n=1:num
                for nn = 1:numel(varargin{n})
                    dataArray{end+1} = varargin{n}.cameraIntrinsicsArrayData{nn};
                end
            end
            this.cameraIntrinsicsArrayData = dataArray;
        end

        %------------------------------------------------------------------
        % Overloading the transpose functionality
        %------------------------------------------------------------------
        function this = transpose(this)

        % Transpose is not supported for cell arrays in code
        % generation. Use reshape instead for transpose because these
        % are 1-D arrays.
            if isrow(this)
                this.cameraIntrinsicsArrayData = reshape(this.cameraIntrinsicsArrayData, numel(this), 1);
            else
                this.cameraIntrinsicsArrayData = reshape(this.cameraIntrinsicsArrayData, 1, numel(this));
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
                                   "vision:cameraIntrinsics:oneDimIndexing");

            if numel(varargin)==2
                coder.internal.assert(...
                    ~(isempty(varargin{1}) || isempty(varargin{2})), ...
                    'driving:oneDimArrayBehavior:reshapeWithEmpties');
            end

            this.cameraIntrinsicsArrayData = reshape(this.cameraIntrinsicsArrayData, varargin{:});
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
            y = isrow(this.cameraIntrinsicsArrayData);
        end
    end

    methods(Hidden, Static)
        %------------------------------------------------------------------
        % Creating an empty camera Intrinsics object
        %------------------------------------------------------------------
        function obj = makeEmpty(focalLenClass, principalPointClass, ...
                imageSizeClass, radDistortClass, tangDistortClass, skewClass)
            
            focalLength  = ones(coder.ignoreConst(1), coder.ignoreConst(2), focalLenClass);
            principalPoint = ones(coder.ignoreConst(1), coder.ignoreConst(2), principalPointClass);
            imageSize = ones(coder.ignoreConst(1), coder.ignoreConst(2), imageSizeClass);
            radialDistortion = zeros(coder.ignoreConst(1), coder.ignoreConst(2), radDistortClass);
            tangentialDistortion = zeros(coder.ignoreConst(1), coder.ignoreConst(2), tangDistortClass);
            skew = zeros(coder.ignoreConst(1), skewClass);

            camInt = cameraIntrinsics(focalLength, principalPoint, imageSize, ...
                'RadialDistortion', radialDistortion, 'TangentialDistortion', ...
                tangentialDistortion, 'Skew', skew);
            obj = repmat(camInt, 0, 0);
        end
    end

    methods(Access = public, Hidden)
        %------------------------------------------------------------------
        % Initialize the properties of camera Intrinsics array objects
        %------------------------------------------------------------------
        function this = initializeCameraIntrinsicsArrayData(this)
            
            data = {vision.internal.codegen.cameraIntrinsicsArray(this)};
            coder.varsize('dataArray');
            dataArray = data;
            this.cameraIntrinsicsArrayData = dataArray;
        end

        function checkTypes(this, tObj)
            coder.internal.assert(isa(tObj, class(this)), ...
                    "vision:cameraIntrinsics:invalidClass");
            coder.internal.errorIf(~isa(this.FocalLength, class(tObj.FocalLength)),...
                "vision:cameraIntrinsics:differentTypes");
            coder.internal.errorIf(~isa(this.PrincipalPoint, class(tObj.PrincipalPoint)),...
                "vision:cameraIntrinsics:differentTypes");
            coder.internal.errorIf(~isa(this.ImageSize, class(tObj.ImageSize)),...
                "vision:cameraIntrinsics:differentTypes");
            coder.internal.errorIf(~isa(this.RadialDistortion, class(tObj.RadialDistortion)),...
                "vision:cameraIntrinsics:differentTypes");
            coder.internal.errorIf(~isa(this.TangentialDistortion, class(tObj.TangentialDistortion)),...
                "vision:cameraIntrinsics:differentTypes");
            coder.internal.errorIf(~isa(this.Skew, class(tObj.Skew)),...
                "vision:cameraIntrinsics:differentTypes");

        end
    end

end