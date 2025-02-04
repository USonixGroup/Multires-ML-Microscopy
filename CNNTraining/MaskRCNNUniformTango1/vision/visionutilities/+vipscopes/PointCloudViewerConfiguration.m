classdef PointCloudViewerConfiguration < Simulink.scopes.BlockScopeConfiguration
    % PointCloudViewerConfiguration object for the Scope
    %   This object is used to configure the Scope programmatically.
    %   Click on a Scope block and use
    %   get_param(gcb,'ScopeConfiguration') to get the object.
    
    %   Copyright 2022 The MathWorks, Inc.
    properties
        XData
        YData
        ZData
        MapExpression
    end
    
    methods(Hidden)
        function props = getDisplayProperties(this)            
            props = {'Name','Position','Visible','XData','YData','ZData', 'MapExpression'};
        end
    end
    
    methods
        
        function this = PointCloudViewerConfiguration(varargin)
                    this@Simulink.scopes.BlockScopeConfiguration(varargin{:});
        end
        
        function value = get.XData(this)
           value = [];
           if isLaunched(this.Specification)
               fw = getUnifiedScope(this.Specification);
               value = get(fw.Visual.Primitive,'XData');
           end 
        end

        function value = get.YData(this)
           value = [];
           if isLaunched(this.Specification)
               fw = getUnifiedScope(this.Specification);
               value = get(fw.Visual.Primitive,'YData');
           end 
        end

        function value = get.ZData(this)
           value = [];
           if isLaunched(this.Specification)
               fw = getUnifiedScope(this.Specification);
               value = get(fw.Visual.Primitive,'ZData');
           end 
        end
        
        % get for MapExpression
        % The colormap should be set using the block parameters useColormap and
        % colorMapValue
        function value = get.MapExpression(this)
            value = getScopeParam(this.Specification,'Visuals','PointCloud',...
                'ColorMapExpression');
        end       
        
    end
    
end

