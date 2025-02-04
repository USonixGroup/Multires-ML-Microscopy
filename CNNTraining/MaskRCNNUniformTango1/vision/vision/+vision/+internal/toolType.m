classdef toolType < uint8
    %toolType Enumeration of supported labeler tool types
    %   toolType creates an enumeration specifying the type of
    %   labeler tool.
    %
    %   toolType enumerations:
    %   None               - No tool specified
    %   ImageLabeler       - Image Labeler
    %   VideoLabeler       - Video Labeler
    %   GroundTruthLabeler - Ground Truth Labeler
    %   LidarLabeler       - Lidar Labeler
    %
    % See also labelType.
    
    % Copyright 2017-2021 The MathWorks, Inc.
    
    enumeration
        % No Labeler
        None               (0)
        
        % ImageLabeler
        ImageLabeler       (1)
        
        % VideoLabeler
        VideoLabeler       (2)
        
        % GroundTruthLabeler
        GroundTruthLabeler (3)
        
        % LidarLabeler
        LidarLabeler       (4)        
        
    end

    methods (Hidden)
        % TODO: Update getToolTitle() and getInstanceName() for other labelers
        function title = getToolTitle(this)
            if this == vision.internal.toolType.ImageLabeler
                title = vision.getMessage('vision:labeler:ToolTitleIL');
            elseif this == vision.internal.toolType.VideoLabeler
                title = vision.getMessage('vision:labeler:ToolTitleVL');
            elseif this == vision.internal.toolType.GroundTruthLabeler
            elseif this == vision.internal.toolType.LidarLabeler
            end
        end

        function instanceName = getInstanceName(this)
            if this == vision.internal.toolType.ImageLabeler
                instanceName = 'imageLabeler';
            elseif this == vision.internal.toolType.VideoLabeler
                instanceName = 'videoLabeler';
            elseif this == vision.internal.toolType.GroundTruthLabeler
            elseif this == vision.internal.toolType.LidarLabeler
            end
        end        
    end
end
