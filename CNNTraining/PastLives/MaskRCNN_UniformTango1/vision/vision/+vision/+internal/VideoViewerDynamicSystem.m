classdef VideoViewerDynamicSystem < extmgr.DynamicSystem
%

%   Copyright 2017-2023 The MathWorks, Inc.

    methods
        function this = VideoViewerDynamicSystem(hApplication, extFile, regName, varargin)
            this@extmgr.DynamicSystem(hApplication, extFile, regName, varargin{:});
        end
        
        function renderToolbars(this)
            % Exercise deeper control over how the toolbar buttons are
            % ordered: the export button must be first, followed by the
            % pixel navigation tools, and then finally the image navigation
            % tools.
            hExtensions = this.ExtensionSet.Children;
            imageViewerExporter = [];
            imageNavigationTool = [];
            pixelNavigationTool = [];
            for hExtension = hExtensions
                if isa(hExtension, 'iptscopes.ImageViewerExporter')
                    imageViewerExporter = hExtension;
                elseif isa(hExtension, 'iptscopes.ImageNavigationTool')
                    imageNavigationTool = hExtension;
                elseif isa(hExtension, 'iptscopes.PixelRegionTool')
                    pixelNavigationTool = hExtension;
                else
                    renderToolbars(hExtension);
                end
            end
            renderToolbars(imageViewerExporter);
            renderToolbars(pixelNavigationTool);
            renderToolbars(imageNavigationTool);
        end
    end
end
