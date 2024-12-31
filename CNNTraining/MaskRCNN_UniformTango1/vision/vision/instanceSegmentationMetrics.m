classdef instanceSegmentationMetrics < vision.internal.objectMetrics

%   Copyright 2022-2023 The MathWorks, Inc.

    methods
        function obj =  instanceSegmentationMetrics(predDS, gtDS, threshold, useParallel, verbose) 
        
                obj = obj@vision.internal.objectMetrics(predDS, gtDS,...
                                                        @vision.internal.maskOverlapRatio,...
                                                        threshold, useParallel, verbose,...
                                                        "Instance Segmentation");
        end
    end

    methods(Access=protected)

        function sample = getObjFromList (~, objList, idx)
            % Object localization for instance segmentation is expressed as 
            % mask stacks. This function indexes into the mask stacks to
            % return a subset of masks based on idx
            sample = objList(:,:,idx);
        end

        function area = objArea(~, objList)
            % Raw number of pixels in each mask
            maskPixelArea = squeeze(sum(objList, [1 2]));  

            area = zeros(size(objList,3),1);
            horizontal = squeeze(sum(objList,1));
            % Ensure that horizontal is always a column vector. This fails
            % for cases with single objects.
            if(isrow(horizontal))
                horizontal = horizontal';
            end
            vertical = squeeze(sum(objList,2));
            
            for maskIdx = 1:numel(area)
                % Skip masks with zero area
                if maskPixelArea(maskIdx) == 0
                    continue
                end
                % Determine W and H from the extent of the mask
                nonzeroH = find(horizontal(:,maskIdx));
                width = nonzeroH(end) - nonzeroH(1) + 1;
                nonzeroV = find(vertical(:,maskIdx));
                height = nonzeroV(end) - nonzeroV(1) + 1;
                area(maskIdx) = width*height;
            end
        end
    end
end