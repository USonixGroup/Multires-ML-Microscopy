% IntrinsicsOptionsSection Encapsulates intrinsics options tool strip section
%
%   IntrinsicsOptionsSection() creates a tool
%   strip section containing the fixed/compute intrinsics options controls.
%
% IntrinsicsOptionsSection properties:
%
%  UseFixedIntrinsicsRadioBtn - radio button to use fixed intrinsics
%  ComputeIntrinsicsRadioBtn  - radio button to compute intrinsics
%  LoadIntrinsicsButton       - button to open dialog to load
%                               intrinsics
%
% IntrinsicsOptionsPanel methods:
%
%   setAllButtonsEnabled      - enable or disable the panel controls

% Copyright 2017-2023 The MathWorks, Inc.

classdef IntrinsicsOptionsSection < vision.internal.uitools.NewToolStripSection
    properties
        UseFixedIntrinsicsRadioBtn
        ComputeIntrinsicsRadioBtn
        IntrinsicsOptionBtnGroup
        
        LoadIntrinsicsButton
    end
    
    properties (Dependent)
        IsFixedIntrinsicsSelected
    end
    
    methods
        function this = IntrinsicsOptionsSection()
            this.createSection();
            this.layoutSection();
        end
        
        %------------------------------------------------------------------
        % Gets the option to use Fixed or Computed Intrinsics
        %------------------------------------------------------------------
        function TF = get.IsFixedIntrinsicsSelected(this)
            TF = this.UseFixedIntrinsicsRadioBtn.Value;
        end
        
        %------------------------------------------------------------------
        function set.IsFixedIntrinsicsSelected(this, useFixedIntrinsics)
            this.UseFixedIntrinsicsRadioBtn.Value = useFixedIntrinsics;
            this.LoadIntrinsicsButton.Enabled = useFixedIntrinsics;
            this.ComputeIntrinsicsRadioBtn.Value = ~useFixedIntrinsics;
        end
        
        %------------------------------------------------------------------
        function enableLoad(this, state)
            this.LoadIntrinsicsButton.Enabled = state;
        end
        
        %------------------------------------------------------------------
        function enableRadioButtons(this, state)
            this.UseFixedIntrinsicsRadioBtn.Enabled = state;
            this.ComputeIntrinsicsRadioBtn.Enabled = state;
        end
        
        %------------------------------------------------------------------
        function setAllButtonsEnabled(this, state)
            % setAllButtonsEnabled Enable or disable all panel controls
            %   setAllButtonsEnabled(panel, state) enables or disables all
            %   controls of the panel. panel is a ToolStripPanel object. state
            %   is a logical scalar.
            this.UseFixedIntrinsicsRadioBtn.Enabled = state;
            this.ComputeIntrinsicsRadioBtn.Enabled = state;
            this.LoadIntrinsicsButton.Enabled = state;
        end
        
        %------------------------------------------------------------------
        function selectFixedMode(this)
            this.IsFixedIntrinsicsSelected = true;
            this.enableLoad(true);
        end
        
        %------------------------------------------------------------------
        function selectComputeMode(this)
            this.IsFixedIntrinsicsSelected = false;
            this.enableLoad(false);
        end
    end
    
    methods
        %------------------------------------------------------------------
        function createSection(this)
            title = getString(message('vision:caltool:IntrinsicsSection'));
            tag = 'secIntrinsics';
            this.Section = matlab.ui.internal.toolstrip.Section(title);
            this.Section.Tag = tag;
        end
        %------------------------------------------------------------------
        function layoutSection(this)
            this.addButtons();
            
            colIntrinsics = this.addColumn();
            colIntrinsics.add(this.ComputeIntrinsicsRadioBtn);
            colIntrinsics.add(this.UseFixedIntrinsicsRadioBtn);
            colIntrinsics.add(this.LoadIntrinsicsButton);
            
        end
        %------------------------------------------------------------------
        function addButtons(this)
            this.IntrinsicsOptionBtnGroup = matlab.ui.internal.toolstrip.ButtonGroup;
            
            
            titleId = 'vision:caltool:FixedIntrinsicsRadioButton';
            toolTipId = 'vision:caltool:FixedIntrinsicsToolTip';
            tag  =  'btnFixedIntrinsics';
            this.UseFixedIntrinsicsRadioBtn = this.createRadioButton(titleId,...
                tag, toolTipId,this.IntrinsicsOptionBtnGroup);
            
            titleId = 'vision:caltool:ComputeIntrinsicsRadioButton';
            toolTipId = 'vision:caltool:ComputeIntrinsicsToolTip';
            tag = 'btnComputeIntrinsics';
            this.ComputeIntrinsicsRadioBtn = this.createRadioButton(titleId,...
                tag, toolTipId,this.IntrinsicsOptionBtnGroup);
            
            titleId = 'vision:caltool:LoadIntrinsicsButton';
            toolTipId = 'vision:caltool:LoadIntrinsicsToolTip';
            loadIntrinsicsBtnIcon = matlab.ui.internal.toolstrip.Icon('import_camera');
            tag = 'loadIntrinsicsButton';
            this.LoadIntrinsicsButton = this.createButton(loadIntrinsicsBtnIcon, titleId, tag);
            this.setToolTipText(this.LoadIntrinsicsButton,toolTipId);
            
        end
    end
end