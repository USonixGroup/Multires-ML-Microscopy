%imageSet Define collection of images.
%
%   -----------------------------------------------------------------------
%   imageSet has been removed. Use imageDatastore instead.
%   ----------------------------------------------------------------------- 
%

% Copyright 2014-2024 MathWorks, Inc.

classdef imageSet
    methods (Access = public)
        function this = imageSet(varargin)
            % g2971664: start erroring out
            error(message('vision:imageSet:imageSetDeprecation'));
        end
    end
end