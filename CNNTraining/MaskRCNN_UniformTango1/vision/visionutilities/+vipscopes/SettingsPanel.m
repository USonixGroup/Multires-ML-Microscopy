classdef SettingsPanel < dialogmgr.DPVerticalPanel
  %SettingsPanel Define measurement extension override for dialog manager
  
  %   Copyright 2022 The MathWorks, Inc.

  properties
    Application    
  end
  
  properties (Access = private)
    hFigPanelListener = [];
    ApplicationIsVisibleListener;
  end
  
  properties (Constant)
    % Measurement panel width for all measurement panels in all scopes
    SettingsPanelWidth = 180;
  end
  
  methods
    function dp = SettingsPanel(hApp)
      % Get the visualization parent from the framework application
      hParent = getVisualizationParent(hApp); 
      
      % save the original child container.
      hContainer = get(hParent,'Children');

      % make a figure to house the uipanel.        
      % Destroying this panel causes the DPVerticalPanel to delete
      hFigPanel = uipanel(hParent,'BorderType','none');

      % temporarily move container to the injected uipanel
      % this prevents flashing of the axes when creating dialog manager
      set(hContainer(end:-1:1), 'Parent', hFigPanel);

      % %copy background color from the Parent to the FigPanel
      set(hFigPanel, ...
        'BackgroundColor',get(hParent,'BackgroundColor'));

      %propagate any background color change from the Parent to the FigPanel
      l = uiservices.addlistener(hParent, ...
            'BackgroundColor','PostSet', ...
            @(src, evt) set(hFigPanel,'BackgroundColor', ...
                            get(hParent,'BackgroundColor')));
                            
      % call the constructor
      dp = dp@dialogmgr.DPVerticalPanel(hFigPanel);
      % measurements need to special case high-DPI machines
      dp.PixelFactor = matlabshared.scopes.measurements.getPixelFactor;
      
      % now perform some default initialization      
      dp.AutoHide   = false;
      dp.PanelLock  = false;
      dp.PanelWidth = dp.SettingsPanelWidth * dp.PixelFactor;
      dp.PanelLockWidth = true;
      
      % force dialog manager to accept new panel width
      resizeChildPanels(dp,true);
      
      % defer drawing the body splitter arrows until final draw
      dp.hBodySplitter.Visible = false;

      % Use a custom font size
      dp.FontSize = uiservices.getFontSize;
 
      % turn on the visibility of the body panel's parent
      % this prevents blanking the entire screen during a drawnow
      hBodyPanelParent = get(dp.hBodyPanel,'Parent');
      set(hBodyPanelParent,'Visible','on');
      
      % move container to the dialog manager's body panel
      set(hContainer(end:-1:1), 'Parent', dp.hBodyPanel);
      
      % reparent the screen message and turn on visibility
      setScreenMsgParent(hApp, dp.hBodyPanel);
      
      % resizing the hFigPanel should also resize the dialog panel.
      % We'll need to perform this step manually.
      set(hFigPanel,'ResizeFcn',@(src,evt) resizeParentPanel(dp));  
      
      % Set listener on application Is Visible to re-size
      dp.ApplicationIsVisibleListener = uiservices.addlistener(hApp, ...
          'IsVisible', @(~, ~) resizeParentPanel(dp));
      
      % resize the container to be the same shape as the parent
      % this should call the children's resize event (if any)                  
          for i=1:numel(hContainer)
              setpixelposition(hContainer(i), getpixelposition(dp.hBodyPanel));
          end
        
      dp.Application = hApp;
      dp.hFigPanelListener = l;
      
      % disable measurements when all dialogs are closed
      setAllDialogsClosedFcn(dp,@(src) autoDisable(dp));           
    end    
    
    function updatedDialogs = resizeChildPanels(dp,varargin)      
      % Resize child panels and update inset if needed
      updatedDialogs = resizeChildPanels@dialogmgr.DPVerticalPanel(dp,varargin{:});
      if ~isempty(dp.Application)
        updateDynamicWidgetsControl(dp.Application);
        hVisual = dp.Application.Visual;
        if ~isempty(hVisual) && isa(hVisual,'dsp.scopes.SpectrumVisual')
          hVisual.updateInset;
          hVisual.updateCorrectionModeMessage;
          hVisual.updateLegend;
        end
      end            
    end        
    
    function closeDialogPanel(dp)
      
      % close all undocked dialogs
      while ~isempty(dp.UndockedDialogs)
        closeUndockedDialog(dp, dp.UndockedDialogs(1));
      end
      
      % close all docked dialogs
      while ~isempty(dp.DockedDialogs)
        closeDialog(dp, dp.DockedDialogs(1));
      end
      
      % Turn off the panel
      setDialogPanelVisible(dp,false);
      
      % Delete trace selection listeners
      dlgTrace = findobj(dp.Dialogs, 'Name', getTraceSelectionDlgName(dp));
      if ~isempty(dlgTrace)
         deleteListeners(dlgTrace.DialogContent);
      end
      
      % delete the dialogs      
      dp.Dialogs = [];     
      
      % delete background color listener
      delete(dp.hFigPanelListener);
      dp.hFigPanelListener = [];
      
      % delete application visibility listener
      delete(dp.ApplicationIsVisibleListener);
      dp.ApplicationIsVisibleListener = [];

      % return the screen message back to the visualization parent
      setScreenMsgParent(dp.Application, []);
      
      % queued events may not reliably fire if parent changes
      drawnow
    end

    function setSettingsDialogVisibility(dp, dlgName, visible)
      % find the dialog panel
      dlg = findobj(dp.Dialogs, 'Name', dlgName);

      if visible
        if ~isDialogVisible(dp, dlgName)
          %show trace selection dialog if this is the first (docked) measurement        
          if ~any(strcmp(dlgName,getSettingsDialogNames(dp)))
            autoShowTraceSelectionDialog(dp);  
          end

          % show the dialog and its containing panel
          setDialogVisibility(dp, dlg, true);  
          setDialogPanelVisible(dp, true);
          dp.setVisible;

          % enable the body splitter
          % dp.hBodySplitter.Visible = true;

          % call the open dialog method on the dialog
          onOpenDialog(dlg.DialogContent);
        end
      elseif any(strcmpi(getUndockedDialogNames(dp), dlgName))
        % dialog is in the undocked state and must be closed explicitly.
        closeUndockedDialog(dp, dlg);
      elseif any(strcmpi(getDockedDialogNames(dp), dlgName))
        % dialog is docked, make it no longer visible
        setDialogVisibility(dp, dlg, false);
        closeDialog(dp, dlg);
      end
    end    
    
    function moveToTopOfMeasurements(dp, dlgTrace)
      % Get current display order of docked dialogs
      % Move this dialog to first entry, if found
      visDlgs = dp.DockedDialogs;
      findIdx = dlgTrace.DialogContent.ID == getID(visDlgs);
      if any(findIdx)
        % Remove dialog from where it was
        visDlgs(findIdx) = [];
        insertAtTopOfMeasurements(dp, visDlgs, dlgTrace);
        showDialogPanel(dp); % Rearrange dialogs in display
        resetDialogPanelShift(dp); % Shift panels to top of display
      end
    end    
    
    function propertyInfo = getPropertyInfo(dp)
      visDlgs = [dp.DockedDialogs dp.UndockedDialogs];
      if isempty(visDlgs)
        propertyInfo = [];
      else
        for i=1:numel(visDlgs)
          [tag, info] = getPropertyInfo(visDlgs(i).DialogContent);
          propertyInfo.(tag) = info;
        end
      end
    end
    
    function applyPropertyInfo(dp, tag, info)
      dlgs = dp.Dialogs;
      for i=1:numel(dlgs)
        applyPropertyInfo(dlgs(i).DialogContent, tag, info);
      end
    end
    
    function applyCachedPropertyInfo(dp)
      % users may close measurement panels before simulation starts
      % act only over those dialogs which are actually visible
      dlgs = [dp.DockedDialogs dp.UndockedDialogs];
      for i=1:numel(dlgs)
        applyCachedPropertyInfo(dlgs(i).DialogContent);
      end
    end

    % override "no docked dialog" on startup
    function showNoDockedDialogsMsg(dp)
      if ~isempty(dp.hFigPanelListener)
        showNoDockedDialogsMsg@dialogmgr.DPVerticalPanel(dp);
      end
    end
  end  
  
  methods (Access = private)
    function insertAtTopOfMeasurements(dp, visDlgs, dlgTrace)
      for i=1:numel(visDlgs)
        if isa(visDlgs(i).DialogContent.Measurer, ...
            'matlabshared.scopes.measurements.AbstractMeasurement')
         dp.DockedDialogs = [visDlgs(1:i-1) dlgTrace visDlgs(i:end)];
         return
        end
      end
      dp.DockedDialogs = [visDlgs dlgTrace];
    end  
  end
  
  methods (Hidden)
 
            
    function autoDisable(dp)
      % for performance reasons, destroy all measurements when inactive
      if isempty(getDockedDialogNames(dp)) && isempty(getUndockedDialogNames(dp))
        vipscopes.disableSettings(dp);
      end
    end

    
    function dockUndockedDialog(dp, dlg)
      dlgContent = dlg.DialogContent;
      dockUndockedDialog@dialogmgr.DPVerticalPanel(dp, dlg)
      onPostDockUndockedDialog(dlgContent);
    end
    
    function undockDialog(dp, dlg)
      dlgContent = dlg.DialogContent;
      undockDialog@dialogmgr.DPVerticalPanel(dp, dlg)
      onPostUndockDialog(dlgContent);
      % Ensure undocked dialog appears in front
      drawnow
      figure(dlgContent.Dialog.DialogBorder.DialogPresenter.hFig);
    end    
    
  end
  
  methods (Access = private)
    
    
    function dlgName = getTraceSelectionDlgName(~)
      dlgName = getString(message('Spcuilib:measurements:TraceSelection'));
    end
    
    function dlgNames = getSettingsDialogNames(~)
      dlgNames = {'Settings'};
    end
  end
end
