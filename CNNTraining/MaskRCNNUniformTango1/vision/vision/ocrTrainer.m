function varargout = ocrTrainer(varargin)
%

% Copyright 2015-2023 The MathWorks, Inc.

error(message('vision:ocrTrainer:notSupported'));
import matlab.internal.capability.Capability;
Capability.require([Capability.Swing, Capability.ComplexSwing]);

if strcmpi(computer,'maca64')
    error(message('vision:ocrTrainer:unsupportedMaca64'));
end

if nargin > 0
    [varargin{:}] = convertStringsToChars(varargin{:});
end

narginchk(0,1);

if nargin == 0
    % Create a new Training Data Labeler
    tool = vision.internal.ocr.webTool.OCRTrainer();
    
    % Render the tool on the screen
    tool.show();
    
    % Display deprecation dialog
    tool.displayDeprecationDialog();

elseif nargin == 1

    if strcmpi(varargin{1}, 'close')
    
    elseif exist(varargin{1}, 'file') || exist([varargin{1}, '.mat'], 'file')
        
        % Load a session
        sessionFileName = varargin{1};
        import vision.internal.calibration.tool.*;
        [sessionPath, sessionFileName] = parseSessionFileName(sessionFileName);

        tool = vision.internal.ocr.webTool.OCRTrainer();
        tool.show();

        % Display deprecation dialog
        tool.displayDeprecationDialog();

        processOpenSession(tool, sessionPath, sessionFileName,false);
      
    else
        error(message('vision:trainingtool:InvalidInput',varargin{1}));
    end
end

if nargout == 1
    varargout{1} = tool;
end

end
