classdef CalibrationSessionManager < vision.internal.uitools.SessionManager
%

%   Copyright 2014-2021 The MathWorks, Inc.

    properties
        IsStereo = false;
    end
    
    methods
        
        %------------------------------------------------------------------
        function session = loadSession(this, pathname, filename, varargin)
            % Call general load routine, and specialize.
            if nargin > 3 && isa(varargin{1}, 'matlab.ui.container.internal.AppContainer')
                session = loadSession@vision.internal.uitools.SessionManager(...
                    this, pathname, filename, varargin{1});
            else
                session = loadSession@vision.internal.uitools.SessionManager(...
                    this, pathname, filename);
            end
            
            % Setup imageSize in PatternSet if not previously set
            if ~isempty(session)
                if isprop(session, 'PatternSet') && ~isempty(session.PatternSet)
                    if isprop(session.PatternSet, 'ImageSize') && isempty(session.PatternSet.ImageSize)
                        imSize = size(imread(session.PatternSet.FullPathNames{1,1}));
                        session.PatternSet.setImageSize([imSize(1), imSize(2)]);
                    end
                end 
            end
        end
        
        %------------------------------------------------------------------
        function isValid = isValidSessionFile(this, sessionStruct)
            isValid = isValidSessionFile@vision.internal.uitools.SessionManager(this, sessionStruct);
            if isValid
                session = sessionStruct.(this.SessionField);
                if this.IsStereo
                    isValid = session.IsValidStereoCameraSession;
                    if session.IsValidSingleCameraSession
                        this.CustomErrorMsgId = ...
                            'vision:caltool:loadingSingleCameraSessionIntoStereoCalibrator';
                    end
                else
                    isValid = session.IsValidSingleCameraSession;
                    if session.IsValidStereoCameraSession
                        this.CustomErrorMsgId = ...
                            'vision:caltool:loadingStereoCameraSessionIntoCameraCalibrator';
                    end
                end
            end
        end
    end
end