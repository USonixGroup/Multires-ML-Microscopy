function OpenPatternDetectorTemplate(cameraType)
%OpenPatternDetectorTemplate opens a custom calibration pattern detector template.
%  vision.OpenPatternDetectorTemplate('monocular') opens a custom calibration
%  pattern detector template for single camera calibration in the MATLAB editor.
%
%  vision.OpenPatternDetectorTemplate('stereo') opens a custom calibration
%  pattern detector template for stereo camera calibration in the MATLAB editor.
%
%  Note: This is an undocumented function. Its behavior may change, or the
%  feature itself may be removed in a future release.

%   Copyright 2021 The MathWorks, Inc.

    arguments
        cameraType (1,1) string {mustBeMember(cameraType,["stereo","monocular"])}
    end
    
    if strcmp(cameraType, 'monocular')
        template = fullfile(matlabroot, 'toolbox','vision','vision','+vision','+calibration',...
            '+template','MonocularDetectorTemplate.m');
    else
        template = fullfile(matlabroot, 'toolbox','vision','vision','+vision','+calibration',...
            '+template','StereoDetectorTemplate.m');
    end
    fileString = fileread(template);
    
    % Open the template code in the editor
    matlab.desktop.editor.newDocument(fileString);
end