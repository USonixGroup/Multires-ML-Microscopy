%   This pointTrack class contains code generation implementation of
%   pointTrack

%   This class is for internal use only.

%   Copyright 2021-2022 The MathWorks, Inc.

%#codegen

classdef pointTrack < vision.internal.pointTrackImpl & ...
        vision.internal.codegen.pointTrackArray

    properties(Access = protected) % For array of pointTrack codegen
        Data
    end

    properties (Constant, Hidden)
        ClassName = 'pointTrack';
    end

    methods
        %------------------------------------------------------------------
        % Constructor
        %------------------------------------------------------------------
        function this = pointTrack(varargin)
            this = this@vision.internal.pointTrackImpl(varargin{:});
            coder.varsize('data');
            data = {vision.internal.pointTrackImpl(varargin{:})};
            this.Data = data;
        end
    end

    methods(Hidden, Static)
        %------------------------------------------------------------------
        % Creating an empty pointTrack object
        %------------------------------------------------------------------
        function obj = makeEmpty(pointsClass)
            % makeEmpty Make an empty object

            % The empty() method cannot be overridden in code generation.
            % To overcome this, we use the make-empty method as an interface
            % to create empty objects.
            points  = zeros(coder.ignoreConst(0), coder.ignoreConst(2), pointsClass);
            viewIds = zeros(1, coder.ignoreConst(0), 'uint32');
            featureIndices = zeros(1, coder.ignoreConst(0), 'uint32');

            % Create a dummy object
            pt = pointTrack(viewIds, points, featureIndices);
            % Use repmat to make an empty object
            obj = repmat(pt, 0, 0);
        end
    end
end
