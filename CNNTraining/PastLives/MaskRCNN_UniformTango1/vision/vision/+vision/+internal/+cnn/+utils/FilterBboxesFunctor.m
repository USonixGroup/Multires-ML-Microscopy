classdef FilterBboxesFunctor < handle
    %FilterBboxesFunctor A functor like object to support filtering of bounding boxes.
    %
    % See also fastRCNNObjectDetector, fasterRCNNObjectDetector.

    % Copyright 2019-2023 The MathWorks, Inc.
    %#codegen
    properties (Access = protected)
        PreprocessFunction;
        PostprocessFunction;
    end

    methods (Access = protected)
        function varargout = preprocess(this, minSize, maxSize, varargin)
            varargout = varargin;
        end
        function varargout = postprocess(this, minSize, maxSize, varargin)
            varargout = varargin;
        end
    end

    methods
        %------------------------------------------------------------------
        % Get the detections in the range min to max.
        %------------------------------------------------------------------
        function varargout = filterBBoxes(this, minSize, maxSize, varargin)
            assert(nargin - 3 == nargout);

            if ~isempty(this.PreprocessFunction)
                [varargin{1:nargout}] = this.PreprocessFunction(minSize, maxSize, varargin{:});
            end

            [varargin{1:nargout}] = vision.internal.cnn.utils.FilterBboxesFunctor.filterSmallBBoxes(minSize, varargin{:});
            [varargin{1:nargout}] = vision.internal.cnn.utils.FilterBboxesFunctor.filterLargeBBoxes(maxSize, varargin{:});

            if ~isempty(this.PostprocessFunction)
                [varargin{1:nargout}] = this.PostprocessFunction(minSize, maxSize, varargin{:});
            end
            varargout = varargin;
        end
    end

    methods (Static, Hidden)
        %------------------------------------------------------------------
        % Filter Boxes based on size.
        %------------------------------------------------------------------
        function varargout = filterSmallBBoxes(minSize, varargin)
            assert(nargin - 1 == nargout);
            bboxes = varargin{1};
            tooSmall = any((bboxes(:,[4 3]) < minSize), 2);

            for ii = 1:numel(varargin)
                varargout{ii} = varargin{ii}(~tooSmall,:);
            end
        end

        %------------------------------------------------------------------
        function varargout = filterLargeBBoxes(maxSize, varargin)
            assert(nargin - 1 == nargout);
            bboxes = varargin{1};
            tooBig = any((bboxes(:,[4 3]) > maxSize), 2);

            for ii = 1:numel(varargin)
                varargout{ii} = varargin{ii}(~tooBig,:);
            end
        end
    end
end
