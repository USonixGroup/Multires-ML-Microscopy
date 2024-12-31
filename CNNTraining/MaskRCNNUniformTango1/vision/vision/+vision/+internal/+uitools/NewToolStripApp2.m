%%% After upgrade to AppContainer, rename it to Rename it to
%%% Toolstrip_helper.m or MultiUserTabGroup.m

% NewToolStripApp2 the base class for the toolstrip-based apps
%
%  This is a base class for toolstrip apps using the new Toolstrip MCOS
%  API. It contains the toolgroup object and the session information.
%
%  NewToolStripApp2 properties:
%    ToolGroup      - the matlab.ui.internal.desktop.ToolGroup object
%    TabGroup       - the matlab.ui.internal.Toolstrip.TabGroup object
%    Session        - the session object, containing the App's data
%    SessionManager - the object that should handle session loading/saving
%
%  NewToolStripApp2 methods:
%    removeViewTab - remove the View tab, which is enabled by default
%    addFigure     - add a figure to the app (protected)
%    getGroupName  - return group name
%    getToolGroup  - return toolgroup
%    getTabGroup   - return main TabGroup
%    hideTab       - make tab invisible
%    showTab       - make tab visible
%    removeViewTab - remove default view tab
%    addFigure     - add figure and unregister drag-drop
%    askForSavingOfSession  - popup save session dialog
%    configureQuickAccessBarHelpButton - specify callback for QAB help

%
%   Notes
%   -----
%   Note that this infrastructure is currently set up to only allow
%   Toolstrip apps that use a single tab group, i.e. contextual tab groups
%   aren't supported. Contextual tabs are supported, just not contextual
%   tab groups.

% Copyright 2016-2023 The MathWorks, Inc.

classdef NewToolStripApp2 < handle
    %----------------------------------------------------------------------
    properties(Access = protected)
        % ToolGroup the matlab.ui.internal.desktop.ToolGroup object.
        %  This object must be instantiated in the derived class 
        ToolGroup;
        
        % TabGroup the matlab.ui.internal.toolstrip.TabGroup object. 
        %  This object must be instantiated in the derived class
        TabGroup;
        
        % SessionManager object that handles saving/loading of the session
        %  This object must be instantiated in the derived class
        SessionManager;
    end
    
    %----------------------------------------------------------------------
    properties(Hidden,GetAccess=public,SetAccess = protected)
        % Session the object containing the App's data
        %  This object must be instantiated in the derived class
        Session         
        
    end
    
    %----------------------------------------------------------------------
    methods(Abstract, Access = protected)
        %------------------------------------------------------------------ 
        % closeAllFigures Abstract method for removing all app figures. App
        % authors must implement this function. Its role is to delete all
        % app managed figures.
        %------------------------------------------------------------------
        closeAllFigures(this);        
    end
    
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------  
        % Common delete method for tool strip based apps. This
        % implementation handles the work required for most apps. Namely,
        % deleting the main ToolGroup and all the figures associated with
        % the app.
        % -----------------------------------------------------------------
        function delete(this)
             % seems like a bug that we need to execute close twice
            this.deleteComponenDestroyListener();
            close(this.Container);             
             this.closeAllFigures(); % shut down all figures  
             drawnow();  % allow time for closing of all figures
             delete(this.Container);
        end
       
       
        function TF = hasTab(this, tab)
            TF = ~isempty(contains(this.TabGroup, tab.Tag));
        end        
        
        function tab = getSelectedTab(this)
            % Toolstrip tab
            tab = this.TabGroup.SelectedTab;            
        end        
        %------------------------------------------------------------------
        % Return the TabGroup object. This is a convenience function for
        % use in apps.
        %------------------------------------------------------------------
        function tabGroup = getTabGroup(this)
            tabGroup = this.TabGroup;
        end
        
        %------------------------------------------------------------------
        % For contextual tab support. Makes tab invisible.
        %------------------------------------------------------------------
        function hideTab(this, tab)
            this.TabGroup.remove(getToolTab(tab));
        end
        
        %------------------------------------------------------------------
        % For contextual tab support. Makes tab visible.
        %------------------------------------------------------------------
        function showTab(this, tab)
            
            this.TabGroup.add(getToolTab(tab));
        end
        
        %------------------------------------------------------------------
        % Remove the view tab. Removes the view tab from the app. If you do
        % not call this method, the view tab will be on by default. 
        %------------------------------------------------------------------
        function removeViewTab(this)
            
            this.ToolGroup.hideViewTab();
        end
      
       
        %------------------------------------------------------------------
        % Attach help call back to quick access bar help button. Note
        % you must call call this method before calling open(ToolGroup) in
        % your app.
        %------------------------------------------------------------------
        function configureQuickAccessBarHelpButton(this, helpCallback)
            
            this.ToolGroup.setContextualHelpCallback(helpCallback);
        end
        
        %------------------------------------------------------------------
        % Set Status bar text. Set the Status bar text (bottom bar).
        %------------------------------------------------------------------
        function setStatusText(this, text)
            setStatusText(this.Container, text);
        end

        %------------------------------------------------------------------
        % Add a figure to the app and disable drag-and-drop into it.
        %------------------------------------------------------------------
        function addFigure(this, fig)
            
            this.ToolGroup.addFigure(fig);
            
            this.ToolGroup.getFiguresDropTargetHandler().unregisterInterest(...
               fig);
        end
               
    end
    
    %----------------------------------------------------------------------
    methods (Access = protected)
        function protectOnDelete(~, fHandle, varargin)
            %protectOnDelete is a wrapper around any callback that protects
            % against deletion of the App window while the callback is 
            % being processed. Wrap your callback with this method to avoid
            % command line errors being thrown in such circumstances. 
            %
            % See the configureDisplays method in VideoLabelingTool for
            % example.
            try
                fHandle(varargin{:});
            catch ME
                if isDebugMode()
                    throw(ME);
                else
                    if strcmp(ME.identifier, 'MATLAB:class:InvalidHandle')
                        % Do NOT throw error messages on the command line if the
                        % App has been deleted while processing a callback.
                        return
                    end
                end
            end
        end
    end
    
    %----------------------------------------------------------------------
    methods(Static, Access = protected)
        %------------------------------------------------------------------
        % Pops up dialog asking if session should be saved. Returns the
        % dialog selection: yes, no, or cancel. Should be called during
        % when closing a session or creating a new session when a session
        % already open.
        %------------------------------------------------------------------
        function selection = askForSavingOfSession(hFig)
            if ~isvalid(hFig)
                selection = vision.getMessage('MATLAB:uistring:popupdialogs:No');
                % The app has been deleted when a callback is executed.
                return;
            else
                yes    = vision.getMessage('MATLAB:uistring:popupdialogs:Yes');
                no     = vision.getMessage('MATLAB:uistring:popupdialogs:No');
                cancel = vision.getMessage('MATLAB:uistring:popupdialogs:Cancel');
                question = vision.getMessage('vision:uitools:SaveSessionQuestion');
                title = vision.getMessage('vision:uitools:SaveSessionTitle');
                
                % uiconfirm(fig, Question, Title, 'Options (Btn)', {'a','b','c'}, 'DefaultOption', 'b')
                selection = uiconfirm(hFig, question, title, ...
                    'Options', {yes, no, cancel}, 'DefaultOption', yes);
                
            end

        end
    end
end

function tf = isDebugMode()
 tf  = strcmpi(vision.internal.videoLabeler.gtlfeature('debug'), 'on');
end
