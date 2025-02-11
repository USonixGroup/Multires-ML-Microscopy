classdef objectDetectionMetrics < vision.internal.objectMetrics

%   Copyright 2023 The MathWorks, Inc.

    methods
        function obj =  objectDetectionMetrics(predDS, gtDS, threshold, useParallel, verbose, additionalMetrics) 
        
                obj = obj@vision.internal.objectMetrics(predDS, gtDS,...
                                                        @bboxOverlapRatio,...
                                                        threshold, useParallel, verbose, ...
                                                        "Object Detection", additionalMetrics);
        end
    end

    methods(Access=protected)

        function sample = getObjFromList (~, objList, idx)
            % Object localization for object detection is expressed as 
            % boudning box stacks. This function indexes into the bounding box stacks to
            % return a subset of bounding boxes based on idx
            sample = objList(idx,:);
        end

        function area = objArea(~, objList)
            area = objList(:,3) .* objList(:,4);            
        end

    end
end