%Submappable Base class for map representations with submaps
%   The Submappable class formalizes the required interface for map
%   representations with submaps for localization. This class is an
%   abstract base class.
%
%   Submappable properties:
%      SelectedSubmap - Selected submap, as [xmin, xmax, ymin, ymax, zmin, zmax]
%
%   Submappable methods
%      selectSubmap (ABSTRACT)   - Select a submap within the map
%      isInsideSubmap            - Check if a position is inside the selected submap
%
%   See also pcmapndt.

% Copyright 2020-2023 The MathWorks, Inc.

%#codegen
classdef (Abstract, Hidden) Submappable < vision.internal.EnforceScalarValue
    
    properties (SetAccess = private, Abstract)
        %SelectedSubmap
        %   Currently selected submap, specified as a region of interest
        %   [xmin, xmax, ymin, ymax, zmin, zmax].
        SelectedSubmap
    end
    
    methods (Abstract)
        varagout = selectSubmap(varagin);
    end
    
    methods
        function [isInside, distToEdge] = isInsideSubmap(this, pos)
            
            validateattributes(pos, {'single', 'double'}, ...
                {'numel', 3, 'real', 'nonsparse', 'finite'}, ...
                'isInsideSubmap', 'pos');
            
            xlims = this.SelectedSubmap(1:2);
            ylims = this.SelectedSubmap(3:4);
            zlims = this.SelectedSubmap(5:6);
            
            isInside = ...
                pos(1)>xlims(1) && pos(1)<xlims(2) && ...
                pos(2)>ylims(1) && pos(2)<ylims(2) && ...
                pos(3)>zlims(1) && pos(3)<zlims(2);
            
            distToEdge = iComputeDistanceToSubmapEdge(pos, xlims, ylims, zlims);
        end
    end
end

%--------------------------------------------------------------------------
% Private Function
%--------------------------------------------------------------------------
function distToEdge = iComputeDistanceToSubmapEdge(pos, xlims, ylims, zlims)
% Find distance to X, Y and Z planes of submap
distToX = min(abs(pos(1)-xlims));
distToY = min(abs(pos(2)-ylims));
distToZ = min(abs(pos(3)-zlims));

distToEdge = [distToX, distToY, distToZ];
end