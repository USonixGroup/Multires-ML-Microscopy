% SignalType Enumeration of supported signal types 
%   SignalType creates an enumeration specifying the type of signal
%   that can be used in groundTruthMultisignal object and the signal
%   types that can be labeled in GroundTruth Labeler app.
%
%   SignalType enumerations:
%   
%   Image      - Signal of type Image is obtained from sources such as
%                video, image sequences etc
%   PointCloud - Signal of type PointCloud is obtained from data sources
%                such as Velodyne Lidar
%   Time       - Signal of type Time. This type is used for signals that
%                are of type duration or datetime
%
%   See also groundTruthMultisignal,
%   vision.labeler.loading.MultiSignalSource

%   Copyright 2019-2023 The MathWorks, Inc.
classdef SignalType < uint8
    enumeration
        Image (0)
        
        PointCloud(1)
        
        Time(2)
        
        Custom(3)
        
    end
    
    
    methods (Hidden)
        
        function writtenFileName = writeFrame(this, frame, filePath)
            validateFrameForWrite(this,frame);
            
            if this == vision.labeler.loading.SignalType.Image
                imwrite(frame, filePath);
            elseif this == vision.labeler.loading.SignalType.PointCloud
                [~,~,ext] = fileparts(filePath);
                
                if string(ext) == "pcd"
                    encodingFormat = 'compressed';
                else
                    encodingFormat = 'binary';
                end
                pcwrite(frame, filePath, 'Encoding', encodingFormat);
            else
                % Do nothing
            end
            
            writtenFileName = filePath;
        end
        
        function validateFrameForWrite(this, frame)
            % writeFrame only supports Image and PointCloud data.
            if this == vision.labeler.loading.SignalType.Image
                if ~isnumeric(frame)
                    error(message(...
                        'driving:groundTruthMultiSignal:SignalTypeInvalidImageFrameData'));
                end
            elseif this == vision.labeler.loading.SignalType.PointCloud
                if ~isa(frame, 'pointCloud')
                    error(message(...
                        'driving:groundTruthMultiSignal:SignalTypeInvalidPointCloudFrameData'));
                end
            else
                error(message(...
                     'driving:groundTruthMultiSignal:SignalTypeInvalidFrameData'));
            end
        end
        
        function supportedLabelTypes = getSupportedLabelTypes(this)
            if this == vision.labeler.loading.SignalType.Image
                supportedLabelTypes = [labelType.Rectangle, labelType.RotatedRectangle, labelType.Line, ...
                    labelType.ProjectedCuboid, labelType.Polygon, ...
                    labelType.PixelLabel, labelType.Point];
            elseif this == vision.labeler.loading.SignalType.PointCloud
                supportedLabelTypes = [labelType.Cuboid, labelType.Line];
            elseif this == vision.labeler.loading.SignalType.Time
                supportedLabelTypes = labelType.Scene;
            elseif this == vision.labeler.loading.SignalType.Custom
                supportedLabelTypes = labelType.Custom;
            else
                supportedLabelTypes = labelType.empty();
            end
        end
    end
end
