function ax = showReprojectionErrors(cameraParams, varargin)

%   Copyright 2014-2023 The MathWorks, Inc.

if nargin > 1
    [varargin{:}] = convertStringsToChars(varargin{:});
end

[view, hAxes, highlightIndex] = parseInputs(cameraParams, varargin{:});


h = showReprojectionErrorsImpl(cameraParams, view, hAxes, highlightIndex);

if nargout > 0
    ax = h;
end
end

%--------------------------------------------------------------------------
function [view, hAxes, highlightIndex] = ...
    parseInputs(cameraParams, varargin)

validateattributes(cameraParams, {'cameraParameters', ...
    'stereoParameters', 'fisheyeParameters'}, {}, mfilename, 'cameraParams');

parser = inputParser;
parser.addOptional('View', 'BarGraph', @checkView);
parser.addParameter('HighlightIndex', [], @checkPatternIndex);
parser.addParameter('Parent', [], ...
    @vision.internal.inputValidation.validateAxesHandle);
parser.parse(varargin{:})

view = parser.Results.View;
hAxes = parser.Results.Parent;

% turn highlightIndex into a logical vector
highlightIndex = false(1,cameraParams.NumPatterns);
highlightIndex(unique(parser.Results.HighlightIndex)) = true;

    %----------------------------------------------------------------------
    function tf = checkView(view)
        validatestring(view, {'barGraph', 'scatterPlot'}, ...
            'showReprojectionErrors', 'View');
        tf = true;
        if isa(cameraParams, 'stereoParameters') && ...
                strcmpi(view, 'scatterPlot')
            error(message('vision:calibrate:noStereoScatterPlot'));
        end
    end

    %----------------------------------------------------------------------
    % share this function between showReprojectionErrors and
    % showExtrinsics
    function r = checkPatternIndex(in)
        r = true;
        if isempty(in) % empty is allowed
            return;
        end
        
        validateattributes(in, {'numeric'},...
            {'integer','vector', 'positive', '<=', cameraParams.NumPatterns}, ...
            'showReprojectionErrors', 'HighlightIndex');
    end
end
