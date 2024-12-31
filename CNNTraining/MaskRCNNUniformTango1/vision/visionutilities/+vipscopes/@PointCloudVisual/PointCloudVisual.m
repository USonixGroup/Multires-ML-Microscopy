classdef PointCloudVisual < handle & matlabshared.scopes.visual.Visual
    %POINTCLOUDVISUAL Class definition for PointcloudVisual class

    %   Copyright 2022 The MathWorks, Inc.

    properties(SetAccess=protected)
        XData
        YData
        ZData
        CData
        XLimit
        YLimit
        ZLimit
        PcplayerObj
        Primitive
        MarkerSize  = 1;
        PointCloudInfo
        Axes = -1;
        MaxDimensions
        IsOrganized
        ColorProvided
    end

    properties(SetAccess=protected,Dependent,AbortSet)
        DataType
    end
  
    properties(Access=protected)
        ScalingChangedListener
        DataSourceChangedListener
        DataLoadedListener
        ToolsMenuListener
        Extension
        OldDimensions
        DimsStatus
        KeyPlayback
        PointCloudInfoMenu
    end

    properties        
        DialogMgr
    end

    properties(SetAccess=protected,Hidden=true)
        AxesLimitsButton
        AxesPanel
    end

    methods (Hidden)
        
        toggleSettingsDialog(this, src, ~)
        
    end

    methods
        %Constructor
        function this = PointCloudVisual(varargin)

            this@matlabshared.scopes.visual.Visual(varargin{:});

            % Create the Point Cloud Information dialog.
            this.PointCloudInfo = vipscopes.PointCloudInformation(this.Application);
            update(this.PointCloudInfo);

            this.DataLoadedListener = event.listener(this.Application, ...
                'DataReleased', @this.dataReleased);
            this.DataSourceChangedListener = event.listener(this.Application, ...
                'DataSourceChanged', @this.dataSourceChanged);
        end
    end

    methods
        function dataType = get.DataType(this)
            dataType = class(this.XData);
        end

        function set.DataType(this,dataType)

            if ~isempty(this.XData)
                displayType = class(this.XData);
            else
                displayType = dataType;
            end

            if ~isempty(this.PointCloudInfo)
                this.PointCloudInfo.DataType        = dataType;
                this.PointCloudInfo.DisplayDataType = displayType;
            end

        end
    end

    methods (Access = protected)
        cleanup(this, hVisParent)
        hInstall = createGUI(this)
    end
    methods(Static)
        propSet = getPropertySet
    end
end
