classdef vslamStatusType < uint8
%

% Copyright 2023 The MathWorks, Inc.
    
    enumeration
        % Tracking is lost
        TrackingLost   (0)        
        
        % Tracking is successful
        TrackingSuccessful   (1)

        % Tracking adds key frames too frequently
        FrequentKeyFrames   (2)
    end
end