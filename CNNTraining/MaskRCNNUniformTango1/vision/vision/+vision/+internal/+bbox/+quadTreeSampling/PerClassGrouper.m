classdef PerClassGrouper < handle
% PerClassGrouper Group Windows and NumBoxesRatio for each class.
%
%   Window        = [imageNumber,levelNumber,x,y,w,h]
%   NumBoxesRatio = Ratio of number of boxes of a class in a window and
%                   total number of boxes in that window.
% This classdef is for internal use only and is likely to change in future
% releases.

% Copyright 2019 The MathWorks, Inc.

    properties (SetAccess=private)
        UniqueLabels
        Windows
        NumBoxesRatio
    end

    methods
        function this = PerClassGrouper(uniqueLabels)
            this.UniqueLabels = uniqueLabels;
            this.Windows = cell(numel(uniqueLabels),1);
            this.NumBoxesRatio = cell(numel(uniqueLabels),1);
        end

        function group(this, window, label, numBoxesRatio)
            % For each of the labels, place windows that contain a specific
            % label. Each label can contain the same window.
            for jj = 1:numel(label)
                idx = this.UniqueLabels == label(jj);
                this.Windows{idx} = vertcat(this.Windows{idx}, window);
                this.NumBoxesRatio{idx} = vertcat(this.NumBoxesRatio{idx}, numBoxesRatio(jj));
            end
        end
    end
end
