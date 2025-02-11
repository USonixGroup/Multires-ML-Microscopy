classdef ProgressBar < handle
    % ProgressBar A progress bar for detecting checkerboard points in a set of images.
    %
    % waitBar = ProgressBar(numImages) returns an object that
    % encapsulates a wait bar for detecting checkerboard points in a set of
    % images.
    %
    % ProgressBar methods:
    %   update - advance the progress bar by one image.
    %   delete - close the progress bar window
    %
    
    % Copyright 2020 The MathWorks, Inc.
    
    properties(Access=private)
        HWaitBar = [];
        NumImages = 0;
        ImageIdx = 1;
        MessageId
    end
    
    properties(Access=private)
        IsHTMLCanvas = false;
    end
    
    properties(Dependent, GetAccess=public)
        Canceled;
    end
    
    methods
        function this = ProgressBar(numImages, messageId, titleId, tag, parent)
            this.NumImages = numImages;
            this.MessageId = messageId;
            
            if this.NumImages > 1
                this.ImageIdx = 1;
                waitBarMsg = getWaitBarMessage(this.MessageId, 1, numImages);
                if nargin > 4 && ~isempty(parent) && ...
                        isa((parent), 'matlab.ui.container.internal.AppContainer')
                    
                    this.HWaitBar = uiprogressdlg(parent, ...
                        'Message',waitBarMsg, ...
                        'Title', getString(message(titleId)));
                    this.IsHTMLCanvas = true;
                else
                    this.HWaitBar = waitbar(0, waitBarMsg, ...
                        'Tag', tag,...
                        'WindowStyle', 'modal',...
                        'Name', getString(message(titleId)));
                end
            end
        end
        
        %------------------------------------------------------------------
        function canceled = get.Canceled(this)
            if this.IsHTMLCanvas
                canceled = this.NumImages > 1 && (isempty(this.HWaitBar) || ~isvalid(this.HWaitBar));
            else
                canceled = this.NumImages > 1 && (isempty(this.HWaitBar) || ~ishandle(this.HWaitBar));
            end
        end
        
        %------------------------------------------------------------------
        function update(this)
            % update(this) advance the progress bar by one image
            if this.NumImages > 1 && ~this.Canceled
                waitBarMsg = getWaitBarMessage(this.MessageId, this.ImageIdx, this.NumImages);
                if this.IsHTMLCanvas
                    this.HWaitBar.Value = this.ImageIdx/this.NumImages;
                    this.HWaitBar.Message = waitBarMsg;
                else
                    waitbar(this.ImageIdx/this.NumImages, this.HWaitBar, waitBarMsg);
                end
                this.ImageIdx = this.ImageIdx + 1;
            end
        end
        
        %------------------------------------------------------------------
        function delete(this)
            % close the progress bar window
            if this.IsHTMLCanvas
                if ~isempty(this.HWaitBar) && isvalid(this.HWaitBar)
                    close(this.HWaitBar);
                end
            else
                if ishandle(this.HWaitBar)
                    delete(this.HWaitBar);
                end
            end
        end
    end
end

%--------------------------------------------------------------------------
function waitBarMsg = getWaitBarMessage(messageId, i, numImages)
    waitBarMsg = getString(message(messageId, i, numImages));
end