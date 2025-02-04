classdef SettingsDialog < matlabshared.scopes.measurements.AbstractMeasurementDialog
  %SettingsDialog Create Axes dialog

  %   Copyright 2022-2023 The MathWorks, Inc.
  
  properties (Access = private)
    xLimitsLbl
    yLimitsLbl
    zLimitsLbl
    xLimitsField
    yLimitsField
    zLimitsField
    markerSizeLbl
    markerSizeField
    TogglePanelGroup
    hSourceListener
  end
    
  methods        
    function dlg = SettingsDialog(measObject,dlgName)
      dlg@matlabshared.scopes.measurements.AbstractMeasurementDialog(measObject, dlgName);
      dlg.PropertyTag = 'Settings';
      % Source listener gets initialized on openDialog.
      dlg.hSourceListener = event.listener.empty;
      % add listener to when the visual is updated with new displays/lines
    end

    function delete(dlg)
      delete@matlabshared.scopes.measurements.AbstractMeasurementDialog(dlg);
    end


    function buildDialogContextMenu(dlg,dp)
      % Do Nothing
    end
    
  end
  
  methods (Access=protected)
    function createContent(dlg)
     % create the content container
      createContent@matlabshared.scopes.measurements.AbstractMeasurementDialog(dlg);

      % create a toggle panel group contained in dlg.ContentPanel
      % with panel names and initially closed(0) and open(1) panel states.
      % The callback is set to re-render the (first) panel
      panelTagPrefixes = {'AxesLimits','Appearance'};
      dlg.TogglePanelGroup = matlabshared.scopes.measurements.TogglePanelGroup( ...
        dlg.ContentPanel, ...
        {'Axes Limits', 'Appearance'}, ...
        panelTagPrefixes, ...
        [1 1], ...
        @(idx, state) renderContent(dlg)); 

      % Set the content of our dialog
      dlg.Content = dlg.TogglePanelGroup.ContentPanel;
      set(dlg.Content,'Tag','Settings_panel');
      dlg.TogglePanelGroup.LabelToFieldWidthRatio = 0.5;
      makeAxesPanel(dlg);
      makeMarkerPanel(dlg);
      refreshSettingsDialog(dlg);
    end
    
  end
  
  methods (Access=private)

    function makeAxesPanel(dlg)
      hParent = dlg.Content;
      bg = get(hParent,'BackgroundColor');
      hPanel = dlg.TogglePanelGroup.Panel(1);

      dlg.xLimitsLbl = createLbl(dlg, hPanel, bg, 'XLimits', 'right');
      dlg.yLimitsLbl = createLbl(dlg, hPanel, bg, 'YLimits', 'right');
      dlg.zLimitsLbl = createLbl(dlg, hPanel, bg, 'ZLimits', 'right');
      dlg.xLimitsField = createEditNum(dlg, hPanel, bg, 'XLimits', 'left');
      dlg.yLimitsField = createEditNum(dlg, hPanel, bg, 'YLimits', 'left');
      dlg.zLimitsField = createEditNum(dlg, hPanel, bg, 'ZLimits', 'left');


      updateSettingsField(dlg, dlg.xLimitsField, 'X');
      updateSettingsField(dlg, dlg.yLimitsField, 'Y');
      updateSettingsField(dlg, dlg.zLimitsField, 'Z');
      set(dlg.xLimitsField, 'Callback', @(src, evt) xLimitChanged(dlg));
      set(dlg.yLimitsField, 'Callback', @(src, evt) yLimitChanged(dlg));
      set(dlg.zLimitsField, 'Callback', @(src, evt) zLimitChanged(dlg));


      hLabels = [dlg.xLimitsLbl; dlg.yLimitsLbl; dlg.zLimitsLbl];
      hFields = [dlg.xLimitsField; dlg.yLimitsField; dlg.zLimitsField];

      alignPanelContents(dlg.TogglePanelGroup, 1, hLabels, hFields, 0, 0);
      for h=hLabels'
        set(h, 'Position', get(h, 'Position') - [0 3 0 0]);
      end
    end
    
    function makeMarkerPanel(dlg)
        hParent = dlg.Content;
        bg = get(hParent,'BackgroundColor');
        hPanel = dlg.TogglePanelGroup.Panel(2);

        hLabels = zeros(1,1);
        hLabels(1) = createLbl(dlg, hPanel, bg, 'MarkerSize', 'right');
        dlg.markerSizeField = createEditNum(dlg, hPanel, bg, 'MarkerSize', 'left');
        hFields = [dlg.markerSizeField];

        visual = dlg.Measurer;
        pcviewerHandle = findall(visual.PcplayerObj.Axes, 'Tag', 'pcviewer');
        markerSize = pcviewerHandle.SizeData;
        set(dlg.markerSizeField, 'String', num2str(markerSize));
        set(dlg.markerSizeField, 'Callback', @(src, evt) markerSizeChanged(dlg));

        alignPanelContents(dlg.TogglePanelGroup, 2, hLabels, hFields, 0, 0);
        for h=hLabels'
            set(h, 'Position', get(h, 'Position') - [0 3 0 0]);
        end
    end
    function markerSizeChanged(dlg)
        value = get(dlg.markerSizeField,'String');
        val = str2double(value);

         tf = checkMarkerSize(dlg, val);

        if tf
            setRange(dlg,val, 'Marker');
        else
            err(dlg,'InvalidMarkerSize');
        end
        updateSettingsField(dlg, dlg.markerSizeField, 'MarkerSize');
    end

    function xLimitChanged(dlg)
        value = get(dlg.xLimitsField,'String');
        vals = str2double(strsplit(value, ','));

        tf = checkRange(dlg, vals, 'XLimits');
        if tf
        setRange(dlg,vals, 'X');
        else
        updateSettingsField(dlg, dlg.xLimitsField, 'X');
        end
    end
    function yLimitChanged(dlg)
        value = get(dlg.yLimitsField,'String');
        vals = str2double(strsplit(value, ','));

        tf = checkRange(dlg, vals, 'YLimits');
        if tf
        setRange(dlg, vals, 'Y');
        else
        updateSettingsField(dlg, dlg.yLimitsField, 'Y');
        end
    end
    function zLimitChanged(dlg)
        value = get(dlg.zLimitsField,'String');

        vals = str2double(strsplit(value, ','));

        tf = checkRange(dlg, vals, 'ZLimits');

        if tf
            setRange(dlg,vals, 'Z');
        else
            updateSettingsField(dlg, dlg.zLimitsField, 'Z');
        end
    end

    function tf = checkRange(~, range, limits)
        tf = true;
      try
          validateattributes(range, {'numeric'}, ...
              {'vector', 'numel', 2, 'finite', 'real', 'nonsparse', 'increasing'}, ...
              mfilename, limits);
      catch me
          errordlg(me.message);
          tf = false;
      end


    end

    function tf = checkMarkerSize(~, range)
        tf = isscalar(range) && isnumeric(range) && numel(range) ==1 && ...
                isreal(range) && ~isnan(range) && isfinite(range);
    end
    function setRange(dlg,range, axesLabel)
        visual = dlg.Measurer;
        switch axesLabel
            case 'X'
                visual.PcplayerObj.Axes.XLim(1) = range(1);
                visual.PcplayerObj.Axes.XLim(2) = range(2);
            case 'Y'
                visual.PcplayerObj.Axes.YLim(1) = range(1);
                visual.PcplayerObj.Axes.YLim(2) = range(2);
            case 'Z'
                visual.PcplayerObj.Axes.ZLim(1) = range(1);
                visual.PcplayerObj.Axes.ZLim(2) = range(2);

            case 'Marker'

                pcviewerHandle = findall(visual.PcplayerObj.Axes, 'Tag', 'pcviewer');
                pcviewerHandle.SizeData = range(1);
        end


    end
    function updateSettingsField(dlg, hEdit, tPropName)

        visual = dlg.Measurer;
        switch tPropName
            case 'X'
                limMin = num2str( visual.PcplayerObj.Axes.XLim(1));
                limMax = num2str( visual.PcplayerObj.Axes.XLim(2));
                set(hEdit, 'String', [limMin ',' limMax]);
            case 'Y'
                limMin = num2str(visual.PcplayerObj.Axes.YLim(1));
                limMax = num2str(visual.PcplayerObj.Axes.YLim(2));
                set(hEdit, 'String', [limMin ',' limMax]);
            case 'Z'
                limMin = num2str(visual.PcplayerObj.Axes.ZLim(1));
                limMax = num2str(visual.PcplayerObj.Axes.ZLim(2));
                set(hEdit, 'String', [limMin ',' limMax]);
            case 'MarkerSize'
                pcviewerHandle = findall(visual.PcplayerObj.Axes, 'Tag', 'pcviewer');
                markerSize = pcviewerHandle.SizeData;
                set(hEdit, 'String', num2str(markerSize));
        end
    end
    
    function updateSettingsType(dlg)
      dlg.TogglePanelGroup.paintMe;      
      renderContent(dlg);
    end    
    
    function err(dlg, tag)
      string = getMsgString(dlg, tag);
      h = errordlg(string);
      set(h, 'Tag', ['ErrorDlg.Settings.' tag]);
    end

    function refreshSettingsDialog(dlg)
      updateSettingsType(dlg);
    end
  end
  
  methods (Access = protected)
    function string = getMsgString(dlg, tag)
      % translate tag (or cell array of tags) 
      % into a string (or cell array of strings)
      msgPrefix = 'vision:block:';
      if iscell(tag)
        string = cell(size(tag));
        for i = 1:numel(tag)
          string{i} = getMsgString(dlg, tag{i});
        end
      else
        msgId = [msgPrefix tag];
        msg = message(msgId);
        string = getString(msg);
      end
    end    
    
    function updateContent(~)
      % This method overrides the the Enable check performed in
      % AbstractMeasurementDialog
    end
    
    function h = createLbl(dlg, hParent, bg, msgID, align)
      h = uicontrol('Parent', hParent, ...
                    'String', getMsgString(dlg, msgID), ...
                    'Tooltip',getMsgString(dlg, ['TT' msgID]), ...
                    'Tag', [dlg.TagPrefix lower(msgID) '_lbl'], ...
                    'HorizontalAlignment',align, ...
                    'BackgroundColor', bg, ...
                    'Units','Pixels', ...
                    'FontSize',dlg.FontSize,...
                    'Style','Text');
      p = get(h,'Position');
      set(h,'Position',[p(1:3) 20]);
      set(h,'String',[get(h,'String') ':']);
    end

  end
  
  methods (Hidden)
   
    function onOpenDialog(dlg) 

      % Create source listeners to enable/disable pan properly.
      hScope = dlg.Measurer.Application;
      dlg.hSourceListener = [...
        event.listener(hScope, 'SourceRun', @(hScope, ~) onSourceRun(hScope)); ...
        event.listener(hScope, 'SourceContinue', @(hScope, ~) onSourceRun(hScope)); ...
        event.listener(hScope, 'SourcePause', @(hScope, ~) onSourceStop(hScope)); ...
        event.listener(hScope, 'SourceStop', @(hScope, ~) onSourceStop(hScope));...
        event.listener(hScope, 'RunningInExternalMode', @(hScope, ~) onSourceRun(hScope));];

      %If the source is already running, force a call to onSourceRun.
      if ~isempty(hScope.DataSource) && hScope.DataSource.isRunning
        onSourceRun(hScope);
      end

      notifySettingsChange(dlg);
    end

    function onCloseDialog(dlg)
      % this method is an override of the onCloseDialog method of
      % DialogContent class.  This method is invoked when the user
      % closes/hides the measurement dialog or when the measurement
      % extension is disabled.
      %
      % we synchronize the toolbar/menu items via this method.     
      toggleSettingsDialog(dlg.Measurer, 'forcequit');
      % deleteDisplayListeners(dlg);
      % disable(dlg.CursorsHandler);
      % deleteCursors(dlg.CursorsHandler);
      delete(dlg.hSourceListener(isvalid(dlg.hSourceListener)));
      onSourceStop(dlg.Measurer.Application);
      notifySettingsChange(dlg);
    end
    

    function enable(dlg)
      set(dlg.hSource,'Enable','on');
    end

    function disable(dlg)
      set(dlg.hSource,'Enable','off');
    end   
  end
end

function onSourceRun(hScope)
ext = getExtInst(hScope, 'Tools', 'Plot Navigation');
if ~isempty(ext)
    hVisual = hScope.Visual;
    if strcmp(ext.ZoomMode, 'Pan') && strcmp(hVisual.Configuration.Name, 'Time Domain')
        updateTimeSpan(hVisual);
    end
    enablePan(ext, false);
end
end

function onSourceStop(hScope)
% The panner should not be enabled when Axes are active.
ext = getExtInst(hScope, 'Tools', 'Plot Navigation');
if ~isempty(ext)
    enablePan(ext, true);
end
end

