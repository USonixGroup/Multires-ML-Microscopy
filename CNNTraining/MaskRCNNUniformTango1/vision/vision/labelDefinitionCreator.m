%labelDefinitionCreator Define labels, sublabels, and attributes.
%   labelDefinitionCreator defines labels, sublabels, and attributes
%   for setting up the LabelDefinitions table to use with the labeling
%   apps.
%  
%   ldc = labelDefinitionCreator() creates a labelDefinitionCreator object.
%   Use addLabel, addSublabel, and addAttribute to add label definitions.
%
%   ldc = labelDefinitionCreator(labelDefs) creates a
%   labelDefinitionCreator object with the labels specified in the table of
%   label definitions labelDefs. labelDefs can be obtained from the
%   labeling apps by exporting label definitions. To add, remove, or modify
%   labels, sublabels, or attributes, use the methods listed below.
%
%   labelDefinitionCreator methods:
%   Create new label definitions
%   addLabel        - Add a new label.
%   addSublabel     - Add a new sublabel.
%   addAttribute    - Add a new attribute.
%
%   Remove existing label definitions
%   removeLabel     - Remove an existing label.
%   removeSublabel  - Remove an existing sublabel.
%   removeAttribute - Remove an existing attribute.
%
%   Edit existing label definitions
%   editLabelGroup           - Assign label to new group.
%   editLabelDescription     - Modify description of a label or sublabel.
%   editAttributeDescription - Modify description of an attribute.
%   editGroupName            - Rename an existing group.
%
%   Create label definitions table
%   create - Create the label definition table.
%
%   Get more information
%   info - Get more information about the attribute or label.
%
%   Notes
%   -----
%   - labelDefinitionCreator supports the Image Labeler and the Video 
%     Labeler apps. For the Ground Truth Labeler app, use
%     labelDefinitionCreatorMultiSignal. This app and function both require
%     Automated Driving Toolbox(TM).
%   - The labels defined can be of type Rectangle, RotatedRectangle, Line, ProjectedCuboid,
%     Polygon, Point, Scene, Pixel, or Custom. The sublabels defined can be of type
%     Rectangle, RotatedRectangle, Line, Polygon or ProjectedCuboid. The attributes defined 
%     can be of type List, Logical, String, or Numeric.
%
%   Example: Create a Vehicle Label Hierarchy
%   -----------------------------------------
%   % Create a labelDefinitionCreator object.
%   ldc = labelDefinitionCreator();
%
%   % Add a label for Vehicle.
%   addLabel(ldc, 'Vehicle', 'Rectangle');
%
%   % Add an attribute called Color to Vehicle.
%   addAttribute(ldc, 'Vehicle', 'Color', attributeType.List, {'Red', 'White', 'Green'});
%
%   % Add sublabels called Headlamp and Wheel to Vehicle.
%   addSublabel(ldc, 'Vehicle', 'HeadLamp', labelType.Rectangle);
%   addSublabel(ldc, 'Vehicle', 'Wheel', 'Rectangle');
%
%   % Add an attribute called Diameter to the sublabel Wheel.
%   addAttribute(ldc, 'Vehicle/Wheel', 'Diameter', 'Numeric', 14);
%
%   % Inspect the labels created.
%   ldc
%   
%   Example: Modify an existing Label Definition Table
%   --------------------------------------------------
%   % Load an existing label definition table.
%   labelDefFile = fullfile(toolboxdir('vision'), 'visiondata', 'labelDefsWithAttributes.mat');
%   ld = load(labelDefFile);
%
%   % Create a labelDefinitionCreator object from the table loaded.
%   ldc = labelDefinitionCreator(ld.labelDefs);
%
%   % Add a new attribute to the TrafficLight label.
%   addAttribute(ldc, 'TrafficLight', 'Color', 'List', {'Red', 'Amber', 'Green'});
%   
%   % Inspect the labels created.
%   ldc
%
%   See also groundTruth, videoLabeler, imageLabeler, labelType, attributeType.

% Copyright 2018-2023 The MathWorks, Inc.

%--------------------------------------------------------------------------
classdef labelDefinitionCreator < matlab.mixin.CustomDisplay & vision.internal.EnforceScalarHandle
   % This class is based on the Builder Creational Design Pattern, where a
   % complex object (Label Definitions table) is created with the help of
   % another object (this labelDefinitionCreator). The inheritance from 
   % matlab.mixin.CustomDisplay is purely to customize the default display.
   
    %----------------------------------------------------------------------
    properties (Access = private)
        %labelDefTable Holds the raw label definition table
        labelDefTable
    end
    
    %----------------------------------------------------------------------
    % Public: Constructor
    methods (Access = public)
        %------------------------------------------------------------------
        function this = labelDefinitionCreator(varargin)
            %labelDefinitionCreator constructs labelDefinitionCreator.
            %   This is the constructor, and the documentation for this is
            %   in the beginning of this file.
            
            [inputargs, status] = labelDefinitionCreator.parseInputs(varargin);
            if status.islabelDef
                this.labelDefTable = inputargs;
            elseif status.isempty
                this.labelDefTable = [];
            end
        end
    end
    
    %----------------------------------------------------------------------
    % Create and show details.
    methods (Access = public)
        %------------------------------------------------------------------
        function labelDefs = create(this)
            %create Create the label definition table.
            %   labelDefs = create(ldc) creates a table, labelDefs, that
            %   can be imported as label definitions in the labeling apps.
            %
            %   Example: Export a Bird Label Definition table
            %   ---------------------------------------------
            %   % Create a labelDefinitionCreator object.
            %   ldc = labelDefinitionCreator();
            %
            %   % Add a label called Bird.
            %   addLabel(ldc, 'Bird', labelType.Rectangle);
            %
            %   % Add sublabels called Beak and Wing to label Bird.
            %   addSublabel(ldc, 'Bird', 'Beak', 'Rectangle');
            %   addSublabel(ldc, 'Bird', 'Wing', 'Rectangle');
            %
            %   % Add an attribute called Color to label Bird.
            %   addAttribute(ldc, 'Bird', 'Color', 'List', {'White', 'Blue', 'Pink'});
            % 
            %   % Generate the table to be imported into the labeling apps.
            %   labelDefs = create(ldc);
            %   
            %   See also labelDefinitionCreator/addLabel, labelDefinitionCreator/addSublabel, labelDefinitionCreator/addAttribute.
            
            labelDefs = this.labelDefTable;
            % Don't show Hierarchy or PixelLabelID if not relevant.
            labelDefs = labelDefinitionCreator.removePixelColumnIfEmpty(labelDefs);
            labelDefs = labelDefinitionCreator.removeHierarchyColumnIfEmpty(labelDefs);            
        end
        
        %------------------------------------------------------------------
        function varargout = info(this, name)
            %info Get more information about the attribute or label.
            %   info(ldc, name) displays more information about the
            %   attribute or label referred to by name, which is a string
            %   or character vector. 'name' can also refer to a sublabel or
            %   attribute, when specified as 'labelname/sublabelname' or
            %   'labelname/attributename', or
            %   'labelname/sublabelname/attributename'. This information
            %   includes the Name, Type, and Description. It also includes
            %   attributes and sublabels when pertinent, and ListItems and
            %   DefaultValue for attributes.
            %
            %   infoStruct = info(ldc, name) returns more information about
            %   the label, sublabel, or attribute referred to by 'name' in
            %   the struct infoStruct. The structure contains Name, Type,
            %   Description, Attributes (when pertinent), Sublabels (when
            %   pertinent), DefaultValue (for attributes) and ListItems
            %   (for List attributes).
            %
            %   Example: Display ListItems of an existing Table
            %   -----------------------------------------------
            %   % Load an existing label definition table.
            %   labelDefFile = fullfile(toolboxdir('vision'), 'visiondata', 'labelDefsWithAttributes.mat');
            %   ld = load(labelDefFile);
            %
            %   % Create a labelDefinitionCreator object from the table loaded.
            %   ldc = labelDefinitionCreator(ld.labelDefs);
            %
            %   % Add a new attribute to the TrafficLight label.
            %   addAttribute(ldc, 'TrafficLight', 'Color', 'List', {'Red', 'Amber', 'Green'});
            %   
            %   % Inspect the labels created.
            %   ldc
            %
            %   % Find out the ListItems in the Color attribute of
            %   % TrafficLight.
            %   colorStruct = info(ldc, 'TrafficLight/Color')
            %   colorStruct.ListItems
            %
            %   See also labelDefinitionCreator/create and labelDefinitionCreator/addLabel.
            
            narginchk(2,2);
            validateattributes(name, {'char', 'string'}, {'scalartext'}, mfilename, 'labelName');

            name = strrep(name, '\', '/');
            labelsOnly = listLabels(this, false); % do not include sublabels
            labelsAndSublabels = listLabels(this, true); % include sublabels
            attributes = listAttributes(this);
            
            if ~isempty(labelsOnly) && ~isempty(find(contains(labelsOnly, name), 1))
                % show information about labels if there's a match.
                value = showLabelDetail(this, name);
            elseif ~isempty(labelsAndSublabels) && ~isempty(find(contains(labelsAndSublabels, name), 1))
                % if no labels match, but sublabels do, show info for
                % sublabels.
                value = showSublabelDetail(this, name);
            elseif ~isempty(attributes) && ~isempty(find(contains(attributes, name), 1))
                % if neither label or sublabel matches but attributes does,
                % show info for attributes.
                value = showAttributeDetail(this, name);
            end
            
            if ~exist('value', 'var')
                error(message('vision:labelDefinitionCreator:LabelDoesNotExist',...
                    vision.getMessage('vision:labeler:NewLabelButtonTitle'),...
                    name));                
            end
            
            if nargout == 0
                disp(value)
            else
                varargout{1} = value;
            end
        end
    end
    
    %----------------------------------------------------------------------
    % Custom Display methods
    methods (Access = protected)
        %------------------------------------------------------------------
        function header = getHeader(this)
            classNameHyperLink = matlab.mixin.CustomDisplay.getClassNameForHeader(this);
            % Return table elements as a string array, first column is
            % label name, second is label type, and third is Description.
            numTopLabels = size(this.labelDefTable, 1);
            header = '';
            if numTopLabels == 0
                addLabelLink = "<a href=""matlab:help('labelDefinitionCreator/addLabel')"">addLabel</a>";
                noLabelsText = vision.getMessage('vision:labelDefinitionCreator:HeaderNoLabels', classNameHyperLink, addLabelLink);
                header = strcat(header, noLabelsText);
            else
                % Header sentence.
                labelsText = vision.getMessage('vision:labelDefinitionCreator:HeaderWithLabels', classNameHyperLink);
                header = strcat(header, labelsText);
                header = [header, newline];
                
                % Text description of contents:
                objname = inputname(1);
                for idx = 1:numTopLabels
                    nSublabels = 0;
                    nAttributes = 0;
                    if ~isempty(this.labelDefTable.Hierarchy{idx})
                       hierarchyElems = fieldnames(this.labelDefTable.Hierarchy{idx});
                       for hidx = 1:size(hierarchyElems, 1)
                           if this.isAttribute(this.labelDefTable.Hierarchy{idx}.(hierarchyElems{hidx}))
                               nAttributes = nAttributes + 1;
                           elseif this.isSublabel(this.labelDefTable.Hierarchy{idx}.(hierarchyElems{hidx}))
                               nSublabels = nSublabels + 1;
                           end
                       end
                    end
                    
                    infoLink = sprintf("\t<a href=""matlab:disp(info(%s, '%s'))"">(info)</a>", objname, this.labelDefTable.Name{idx});
                    thisElem = vision.getMessage('vision:labelDefinitionCreator:LabelDetails', this.labelDefTable.Name{idx}, nSublabels, nAttributes, this.labelDefTable.Group{idx});
                    header = [header, thisElem, char(infoLink)]; %#ok<AGROW>
                end
                listLabelsLink = "<a href=""matlab:help('labelDefinitionCreator/info')"">info</a>";
                footer = vision.getMessage('vision:labelDefinitionCreator:HeaderHelpText', char(listLabelsLink));
                header = [header, footer];
            end
        end    
    end   
    
    %----------------------------------------------------------------------
    % Add elements
    methods (Access = public)
        %------------------------------------------------------------------
        function addLabel(this, varargin)
            %addLabel Add a new label.
            %   addLabel(ldc, labelName, typeOfLabel) adds a new label with
            %   name labelName and type typeOfLabel to the label definition
            %   table created using the labelDefinitionCreator ldc.
            %   labelName is a character vector or string scalar that
            %   uniquely identifies the label created. typeOfLabel is a
            %   labelType enumeration or a character vector declaring the
            %   type of label to be added. It can take on values
            %   'Rectangle', 'RotatedRectangle','Line', 'ProjectedCuboid', 'PixelLabel',
            %   'Polygon', 'Scene' or 'Custom'.
            %
            %   addLabel(..., Name, Value) specifies additional name-value
            %   pair arguments as described below:
            %
            %   Optional Name-Value Pairs
            %   -------------------------
            %   'Group'        A character vector or string scalar
            %                  specifying the group of the label.
            %                  Default: 'None'
            %
            %   'LabelColor'   A 3-element row vector specifiying the RGB
            %                  value of the color of the label. RGB values
            %                  need to be in the range [0 ,1]. Yellow (RGB
            %                  = [1 1 0]) is reserved for the color of
            %                  selected labels.
            %                  Default: ''
            %
            %   'Description'  A character vector or string scalar
            %                  describing the label.
            %                  Default: ' '
            %
            %   Example: Add Labels Related to a Driving Scene
            %   ----------------------------------------------
            %   % Create a labelDefinitionCreator object.
            %   ldc = labelDefinitionCreator();
            %
            %   % Add a label called Car.
            %   addLabel(ldc, 'Car', labelType.Rectangle);
            %
            %   % Add a label called Pedestrian of color red (RGB = [1 0 0])
            %   addLabel(ldc, 'Pedestrian', 'Rectangle', 'LabelColor', [1 0 0]);
            %
            %   % Inspect the label created.
            %   ldc
            %
            %   % Add another label StopSign with a group name and a description.
            %   addLabel(ldc, 'StopSign', 'Rectangle', 'Group', 'TrafficSign', 'Description', 'Bounding boxes for stop signs');
            %
            %   % Inspect the labels created again.
            %   ldc
            %
            %   See also labelDefinitionCreator/removeLabel, labelDefinitionCreator/addSublabel, labelDefinitionCreator/addAttribute, labelType.
            
            parser = inputParser;
            parser.FunctionName = 'addLabel';
            parser.addRequired('labelName', @labelDefinitionCreator.checkLabelName);
            parser.addRequired('typeOfLabel', @labelDefinitionCreator.checkLabelType);
            parser.addParameter('LabelColor','' ,@labelDefinitionCreator.checkLabelColor);
            parser.addParameter('Group', 'None', @labelDefinitionCreator.checkLabelName);
            parser.addParameter('Description', ' ', @labelDefinitionCreator.checkDescription);
            parser.parse(varargin{:});
            inputs = parser.Results;
            labelName = char(inputs.labelName);
            checksublabeltype = false;
            typeOfLabel = labelDefinitionCreator.string2labelType(inputs.typeOfLabel, checksublabeltype);
            labelColor = inputs.LabelColor;
            labelGroup = char(inputs.Group);
            labelDescription = char(inputs.Description);
            
            if noLabelsLoaded(this)
                createFirstLabel(this, labelName, typeOfLabel, labelColor, labelGroup, labelDescription);
            else
                appendLabel(this, labelName, typeOfLabel, labelColor ,labelGroup, labelDescription);
            end
        end
        
        %------------------------------------------------------------------
        function addSublabel(this, varargin)
            %addSublabel Add a new sublabel.
            %   addSublabel(ldc, labelName, sublabelName, typeOfSublabel)
            %   adds a new sublabel with name sublabelName and type
            %   typeOfSublabel to the label hierarchy under label labelName
            %   in the label definition table created using the
            %   labelDefinitionCreator ldc. labelName is a character vector
            %   or string scalar that uniquely identifies the Label under
            %   which this sublabel is to be created. sublabelName is a
            %   character vector or string scalar that uniquely identifies
            %   the sublabel created. typeOfSublabel is a labelType
            %   enumeration or a character vector declaring the type of sub
            %   label to be added. It can take on values 'Rectangle', 'RotatedRectangle',
            %   'Line', 'Polygon' or 'ProjectedCuboid'.
            %
            %   addSublabel(..., Name, Value) specifies additional
            %   name-value pair arguments as described below:
            %
            %   Optional Name-Value Pairs
            %   -------------------------
            %   'Description'  A character vector or string scalar 
            %                  describing the sublabel.
            %                  Default: ' '
            %
            %   'LabelColor'   A 3-element row vector specifiying the RGB
            %                  value of the color of the label. RGB values
            %                  need to be in the range [0 ,1]. Yellow (RGB
            %                  = [1 1 0]) is reserved for the color of
            %                  selected labels.
            %                  Default: Assigned color same as that of
            %                  sub-label.
            %                  Default: ''
            %
            %   Example: Add sublabels related to a driving scene
            %   -------------------------------------------------
            %   % Create a labelDefinitionCreator object.
            %   ldc = labelDefinitionCreator();
            %
            %   % Add a label called Vehicle.
            %   addLabel(ldc, 'Vehicle', 'Rectangle');
            %
            %   %  Add sublabels called LicensePlate to Vehicle that is of
            %   %  color blue (RGB = [0 0 1])
            %   addSublabel(ldc, 'Vehicle', 'LicensePlate', 'Rectangle', 'LabelColor', [0 0 1]);
            %
            %   % Add a sublabel called Wheel to the Vehicle label.
            %   addSublabel(ldc, 'Vehicle', 'Wheel', 'rect');
            %
            %   % Inspect the labels created.
            %   ldc
            %
            %   % Add another label called TrafficLight.
            %   addLabel(ldc, 'TrafficLight', labelType.Rectangle, 'Description', 'Bounding boxes for traffic light');
            %
            %   % Add sublabels called RedLight and GreenLight to TrafficLight.
            %   addSublabel(ldc, 'TrafficLight', 'RedLight', 'Rectangle');
            %   addSublabel(ldc, 'TrafficLight', 'GreenLight', labelType.Rectangle);
            %
            %   % Inspect the labels created.
            %   ldc
            %
            %   See also labelDefinitionCreator/addLabel, labelDefinitionCreator/addAttribute, labelType.
            
            parser = inputParser;
            parser.FunctionName = 'addSublabel';
            parser.addRequired('labelName', @labelDefinitionCreator.checkLabelName);
            parser.addRequired('sublabelName', @labelDefinitionCreator.checkSublabelName);
            parser.addRequired('typeOfSublabel', @labelDefinitionCreator.checkLabelType);
            parser.addParameter('LabelColor', '' ,@labelDefinitionCreator.checkLabelColor);
            parser.addParameter('Description', ' ', @labelDefinitionCreator.checkDescription);
            parser.parse(varargin{:});
            inputs = parser.Results;
            labelName = char(inputs.labelName);
            sublabelName = char(inputs.sublabelName);
            checksublabeltype = true;
            typeOfSublabel = labelDefinitionCreator.string2labelType(inputs.typeOfSublabel, checksublabeltype);
            labelColor = inputs.LabelColor;
            sublabelDescription = char(inputs.Description);
            
            % verify that label exists. if not, ask to create first.
            idx = getLabelRowIndex(this, labelName);
            
            % sub-label color same as label-color if empty
            if isempty(inputs.LabelColor)
                inputs.LabelColor = this.labelDefTable.LabelColor(idx);
                labelColor = inputs.LabelColor{1};
            end
            % Disable adding sublabels for unsupported labelTypes.
            this.checkUnsupportedTypes(idx, 'sublabel');
            
            % Construct the sublabel structure, and include it in the
            % hierarchy if it already exists.
            hierarchyStruct = this.getHierarchyAt(idx);
            % if hierarchy exists, check if sublabel name already
            % exists. If yes, there's a clash.
            % else, add the sublabel field to the hierarchy.
            if isfield(hierarchyStruct, sublabelName)
                if labelDefinitionCreator.isAttribute(hierarchyStruct.(sublabelName))
                    error(message('vision:labelDefinitionCreator:AlreadyDefined', sublabelName, vision.getMessage('vision:labeler:NewAttributeButtonTitle'), labelName));
                elseif labelDefinitionCreator.isSublabel(hierarchyStruct.(sublabelName))
                    error(message('vision:labelDefinitionCreator:AlreadyDefined', sublabelName, vision.getMessage('vision:labeler:NewSublabelButtonTitle'), labelName));
                end
            end
            sublabelStruct = struct('Type', typeOfSublabel, 'Description', sublabelDescription, 'LabelColor', labelColor);
            hierarchyStruct.(sublabelName) = sublabelStruct;
            this.labelDefTable.Hierarchy{idx} = hierarchyStruct;
        end        
        
        %------------------------------------------------------------------
        function addAttribute(this, varargin)
            %addAttribute Add a new Attribute.
            %   addAttribute(ldc, labelName, attributeName,
            %   typeOfAttribute, attributeDefault) adds a new Attribute
            %   with name attributeName and type typeOfAttribute to the
            %   Label hierarchy under Label labelName in the Label
            %   Definition table created using the labelDefinitionCreator
            %   ldc. labelName is a character vector or string scalar that
            %   uniquely identifies the Label under which this attribute is
            %   to be created. labelName can also refer to a sublabel when
            %   specified as 'labelName/sublabelName'. attributeName is a
            %   character vector or string scalar that uniquely identifies
            %   the attribute created. typeOfAttribute is an attributeType
            %   enumeration or a character vector declaring the type of
            %   attribute to be added. It can take on values 'List',
            %   'Logical', 'Numeric', or 'String'. attributeDefault is the
            %   default value of the Attribute to be created. It must be a
            %   numeric scalar for Numeric attributes, a character vector
            %   or string scalar for String attributes, a logical scalar
            %   for Logical attributes, and a cell array of character
            %   vectors or an array of string scalars for List attributes.
            %   In List attributes, the first entry in the cell array is
            %   the default value.
            %
            %   addAttribute(..., Name, Value) specifies additional
            %   name-value pair arguments as described below:
            %
            %   Optional Name-Value Pairs
            %   -------------------------
            %   'Description'  A character vector or string scalar
            %                  describing the nature of the attribute.
            %                  Default: ' '
            %
            %   Example: Add Label Definitions related to Traffic Light
            %   -------------------------------------------------------
            %   % Create a labelDefinitionCreator object.
            %   ldc = labelDefinitionCreator();
            %
            %   % Add a label TrafficLight.
            %   addLabel(ldc, 'TrafficLight', labelType.Rectangle, 'Description', 'Bounding boxes for stop signs');
            %
            %   % Add a sublabel RedLight, and an attribute isOn to the sublabel.
            %   addSublabel(ldc, 'TrafficLight', 'RedLight', 'rect');
            %   addAttribute(ldc, 'TrafficLight/RedLight', 'isOn', 'logical', false);
            %
            %   % Add a sublabel GreenLight, and an attribute isOn to the sublabel.
            %   addSublabel(ldc, 'TrafficLight', 'GreenLight', labelType.Rectangle);
            %   addAttribute(ldc, 'TrafficLight/GreenLight', 'isOn', 'logical', false);
            %
            %   % Inspect the labels created.
            %   ldc
            %
            %   See also labelDefinitionCreator/addLabel, labelDefinitionCreator/addSublabel, attributeType.            
            
            parser = inputParser;
            parser.FunctionName = 'addAttribute';
            parser.addRequired('labelName', @labelDefinitionCreator.checkLabelName);
            parser.addRequired('attributeName', @labelDefinitionCreator.checkAttributeName);
            parser.addRequired('typeOfAttribute', @labelDefinitionCreator.checkAttributeType);
            parser.addRequired('defaultValue'); % checked after parsing.
            parser.addParameter('Description', ' ', @labelDefinitionCreator.checkDescription);
            parser.parse(varargin{:});
            inputs = parser.Results;
            labelName = inputs.labelName;
            attributeName = inputs.attributeName;
            typeOfAttribute = labelDefinitionCreator.string2attributeType(inputs.typeOfAttribute);
            attributeDefaults = inputs.defaultValue;
            attributeDescription = char(inputs.Description);
            labelDefinitionCreator.checkAttributeDefault(attributeDefaults, typeOfAttribute);
            
            [labelName, sublabelName] = labelDefinitionCreator.splitLabelNames(labelName);
            if ~isempty(sublabelName)
                addSublabelAttribute(this, labelName, sublabelName, attributeName, typeOfAttribute, attributeDefaults, attributeDescription);
                return;
            end

            idx = getLabelRowIndex(this, labelName);
            % Disable adding attributes for unsupported labelTypes.
            this.checkUnsupportedTypes(idx, 'attribute');

            hierarchyStruct = this.getHierarchyAt(idx);
            if isfield(hierarchyStruct, attributeName)
                if labelDefinitionCreator.isAttribute(hierarchyStruct.(attributeName))
                    error(message('vision:labelDefinitionCreator:AlreadyDefined', attributeName, vision.getMessage('vision:labeler:NewAttributeButtonTitle'), labelName));
                elseif labelDefinitionCreator.isSublabel(hierarchyStruct.(attributeName))
                    error(message('vision:labelDefinitionCreator:AlreadyDefined', attributeName, vision.getMessage('vision:labeler:NewSublabelButtonTitle'), labelName));
                end
            end
            attrStruct = labelDefinitionCreator.createAttrStruct(typeOfAttribute, attributeDefaults, attributeDescription);                
            hierarchyStruct.(attributeName) = attrStruct;
            this.labelDefTable.Hierarchy{idx} = hierarchyStruct;
        end
    end
    
    %----------------------------------------------------------------------
    % Remove elements
    methods (Access = public)
        %------------------------------------------------------------------
        function removeLabel(this, labelName)
            %removeLabel Remove an existing Label.
            %   removeLabel(ldc, labelName) removes an existing label with
            %   name labelName from the label definitions created using the
            %   labelDefinitionCreator object ldc. labelName is a character
            %   vector or string scalar that uniquely identifies the name
            %   of the label to be removed.
            %
            %   Note
            %   ----
            %   Removing a label will remove all sublabels and attributes
            %   defined in its hierarchy.
            %
            %   Example: Remove Labels related to a Driving Scene
            %   -------------------------------------------------
            %   % Create a labelDefinitionCreator object.
            %   ldc = labelDefinitionCreator();
            %
            %   % Add a label called Vehicle.
            %   addLabel(ldc, 'Vehicle', labelType.Rectangle);
            %
            %   % Add a label called StopSign.
            %   addLabel(ldc, 'StopSign', labelType.Rectangle, 'Description', 'Bounding boxes for stop signs');
            %
            %   % Inspect the labels created.
            %   ldc
            %
            %   % Remove the label called Vehicle.
            %   removeLabel(ldc, 'Vehicle');
            %
            %   % Confirm that the label Vehicle has been removed.
            %   ldc
            %
            %   See also labelDefinitionCreator/removeAttribute, labelDefinitionCreator/addLabel, labelDefinitionCreator/addSublabel, labelDefinitionCreator/addAttribute.
            
            labelDefinitionCreator.checkLabelName(labelName);
            idx = getLabelRowIndex(this, labelName);
            this.labelDefTable(idx,:) = [];
        end 
        
        %------------------------------------------------------------------
        function removeSublabel(this, labelName, sublabelName)
            %removeSublabel Remove an existing Sublabel.
            %   removeSublabel(ldc, labelName, sublabelName) removes the
            %   sublabel with name sublabelName from the label named
            %   labelName in the labelDefinitionCreator object ldc.
            %   labelName is a character vector or string scalar that
            %   uniquely identifies the name of the label from which the
            %   sublabel is to be removed. sublabelName is a character
            %   vector or string scalar that uniquely identifies the name
            %   of the sublabel to be removed.
            %
            %   Note
            %   ----
            %   Removing a sublabel will remove all attributes defined in 
            %   its hierarchy.
            %            
            %   Example: Remove Sublabels related to a Driving Scene
            %   ----------------------------------------------------
            %   % Create a labelDefinitionCreator object.
            %   ldc = labelDefinitionCreator();
            %
            %   % Add a label called Vehicle.
            %   addLabel(ldc, 'Vehicle', labelType.Rectangle);
            %
            %   % Add another label called TrafficLight.
            %   addLabel(ldc, 'TrafficLight', labelType.Rectangle, 'Description', 'Bounding boxes for traffic light');
            %
            %   % Add sublabels called RedLight, GreenLight and BlueLight to TrafficLight.
            %   addSublabel(ldc, 'TrafficLight', 'RedLight', labelType.Rectangle);
            %   addSublabel(ldc, 'TrafficLight', 'GreenLight', labelType.Rectangle);
            %   addSublabel(ldc, 'TrafficLight', 'BlueLight', labelType.Rectangle);
            %
            %   % Inspect the labels created.
            %   ldc
            %
            %   % Remove the sublabel BlueLight from the label TrafficLight.
            %   removeSublabel(ldc, 'TrafficLight', 'BlueLight');
            %
            %   % Inspect the labels created.
            %   ldc
            %
            %   See also labelDefinitionCreator/removeLabel, labelDefinitionCreator/addLabel, labelDefinitionCreator/addSublabel.
            
            narginchk(3,3);
            labelDefinitionCreator.checkLabelName(labelName);
            labelDefinitionCreator.checkSublabelName(sublabelName);
            
            idx = getLabelRowIndex(this, labelName);

            if(isfield(this.labelDefTable.Hierarchy{idx}, sublabelName))
                if numel(fieldnames(this.labelDefTable.Hierarchy{idx})) > 3
                    this.labelDefTable.Hierarchy{idx} = labelDefinitionCreator.removeFieldFromStruct(this.labelDefTable.Hierarchy{idx} , sublabelName, labelName);
                else
                    this.labelDefTable.Hierarchy{idx} = [];
                end
            else
                error(message('vision:labelDefinitionCreator:DoesNotExist', sublabelName, labelName));
            end
        end
        
        %------------------------------------------------------------------
        function removeAttribute(this, labelName, attributeName)
            %removeAttribute Remove an existing Attribute.
            %   removeAttribute(ldc, labelName, attributeName) removes the
            %   attribute with name attributeName from a label or sublabel
            %   named labelName in the labelDefinitionCreator object ldc.
            %   labelName is a character vector or string scalar that
            %   uniquely identifies the name of the label from which the
            %   sublabel is to be removed. labelName can also refer to a
            %   sublabel when specified as 'labelName/sublabelName'.
            %   attributeName is a character vector or string scalar that
            %   uniquely identifies the name of the attribute to be
            %   removed.
            %
            %   Example: Remove Attributes related to a Driving Scene
            %   -----------------------------------------------------
            %   % Create a labelDefinitionCreator object.
            %   ldc = labelDefinitionCreator();
            %
            %   % Add a label called Vehicle.
            %   addLabel(ldc, 'Vehicle', labelType.Rectangle);
            %
            %   % Add another label called TrafficLight.
            %   addLabel(ldc, 'TrafficLight', labelType.Rectangle, 'Description', 'Bounding boxes for traffic light');
            %
            %   % Add sublabels called RedLight and GreenLight to TrafficLight.
            %   addSublabel(ldc, 'TrafficLight', 'RedLight', labelType.Rectangle);
            %   addSublabel(ldc, 'TrafficLight', 'GreenLight', labelType.Rectangle);
            %
            %   % Add attributes isOn and Color to sublabel RedLight.
            %   addAttribute(ldc, 'TrafficLight/RedLight', 'isOn', attributeType.Logical, false);
            %   addAttribute(ldc, 'TrafficLight/RedLight', 'Color', attributeType.String, 'Red');
            %
            %   % Inspect the labels created.
            %   ldc
            %
            %   % Remove the Color attributes from sublabel RedLight.
            %   removeAttribute(ldc, 'TrafficLight/RedLight', 'Color');
            %
            %   % Inspect the labels created.
            %   ldc
            %
            %   See also labelDefinitionCreator/removeLabel, labelDefinitionCreator/addLabel, labelDefinitionCreator/addAttribute.
            
            labelDefinitionCreator.checkLabelName(labelName);
            labelDefinitionCreator.checkAttributeName(attributeName);
            
            [labelName, sublabelName] = labelDefinitionCreator.splitLabelNames(labelName);
            idx = getLabelRowIndex(this, labelName);
            if isempty(sublabelName)
                % Remove attribute from a label.
                if(isfield(this.labelDefTable.Hierarchy{idx}, attributeName))
                    if numel(fieldnames(this.labelDefTable.Hierarchy{idx})) > 3
                        this.labelDefTable.Hierarchy{idx} = labelDefinitionCreator.removeFieldFromStruct( ...
                            this.labelDefTable.Hierarchy{idx}, ...
                            attributeName, labelName);
                    else
                        this.labelDefTable.Hierarchy{idx} = [];
                    end
                else
                    error(message('vision:labelDefinitionCreator:DoesNotExist', attributeName, labelName));
                end
            else
                % Removing attribute from a sublabel.
                if(isfield(this.labelDefTable.Hierarchy{idx}, sublabelName))
                    this.labelDefTable.Hierarchy{idx}.(sublabelName) = labelDefinitionCreator.removeFieldFromStruct( ...
                                    this.labelDefTable.Hierarchy{idx}.(sublabelName), ...
                                    attributeName, sublabelName);
                else
                    error(message('vision:labelDefinitionCreator:DoesNotExist', sublabelName, labelName));
                end
            end         
        end     
    end
    
    %----------------------------------------------------------------------
    % Edit descriptions.
    methods (Access = public)
        
        %------------------------------------------------------------------
        function editLabelGroup(this, labelName, groupName)
            %editLabelGroup Modify group of a label.
            %   editLabelGroup(ldc, labelName, groupName) updates the
            %   assigned group of a label with name labelName in the
            %   labelDefinitionCreator object ldc. labelName is a character
            %   vector or string scalar that uniquely identifies the name
            %   of the label whose assigned group is being changed.
            %   groupName is a character vector or string scalar that
            %   contains the group name for the label.
            %
            %   Example: Assign Labels to New Group
            %   -----------------------------------
            %   % Create a labelDefinitionCreator object.
            %   ldc = labelDefinitionCreator();
            %
            %   % Add a label called Car with group Vehicle.
            %   addLabel(ldc, 'Car', labelType.Rectangle, 'Group', 'Vehicle');
            %
            %   % Add a label called Truck with group FourWheeler.
            %   addLabel(ldc, 'Truck', labelType.Rectangle, 'Group', 'FourWheeler');
            %
            %   % Modify the group of Car.
            %   editLabelGroup(ldc, 'Car', 'FourWheeler');
            %
            %   % Inspect the labels created.
            %   ldc
            %
            %   See also labelDefinitionCreator/editGroupName.
            
            labelDefinitionCreator.checkLabelName(labelName);
            labelDefinitionCreator.checkGroupName(groupName);
            
            labelNames = this.listLabels();
            labelIdx = find(strcmpi(labelNames, labelName), 1);
            
            if ~isempty(labelIdx)
                this.labelDefTable.Group{labelIdx} = groupName;
            else
                % Error telling the labelNames does not exist
                error(message('vision:labelDefinitionCreator:LabelDoesNotExist',...
                    vision.getMessage('vision:labeler:NewLabelButtonTitle'),...
                    labelName));                
            end            
        end

        %------------------------------------------------------------------
        function editLabelDescription(this, labelName, description)
            %editLabelDescription Modify description of a label or sublabel.
            %   editLabelDescription(ldc, labelName, description) updates
            %   the Description of a Label or Sublabel with name labelName
            %   in the labelDefinitionCreator object ldc. labelName is a
            %   character vector or string scalar that uniquely identifies
            %   the name of the label of which the description is to be
            %   updated. labelName can also refer to a Sublabel when
            %   specified as 'labelName/sublabelName'. description is a
            %   character vector or string scalar that contains the new
            %   description for the label or sublabel.
            %
            %   Example: Modify Descriptions of Labels
            %   --------------------------------------
            %   % Create a labelDefinitionCreator object.
            %   ldc = labelDefinitionCreator();
            %
            %   % Add a label called Vehicle.
            %   addLabel(ldc, 'Vehicle', labelType.Rectangle);
            %
            %   % Add another label called TrafficLight.
            %   addLabel(ldc, 'TrafficLight', labelType.Rectangle, 'Description', 'Bounding boxes for traffic light');
            %
            %   % Modify the description of Vehicle.
            %   editLabelDescription(ldc, 'Vehicle', 'Bounding boxes for vehicles');
            %
            %   % Inspect the labels created.
            %   ldc
            %
            %   See also labelDefinitionCreator/editAttributeDescription.
            
            labelDefinitionCreator.checkLabelName(labelName);
            labelDefinitionCreator.checkDescription(description);
            description = char(description);
            
            [labelName, sublabelName] = labelDefinitionCreator.splitLabelNames(labelName);
            idx = getLabelRowIndex(this, labelName);
            if isempty(sublabelName)
                % Edit label description for a parent label.
                this.labelDefTable.Description{idx} = description;
                if any(contains(this.labelDefTable.Properties.VariableNames, 'Hierarchy'))
                    if ~isempty(this.labelDefTable.Hierarchy{idx})
                        this.labelDefTable.Hierarchy{idx}.Description = description;
                    end
                end
            else
                % Edit label description for a sublabel.
                if(isfield(this.labelDefTable.Hierarchy{idx}, sublabelName))
                    this.labelDefTable.Hierarchy{idx}.(sublabelName).Description = description;
                else
                    error(message('vision:labelDefinitionCreator:DoesNotExist', sublabelName, labelName));
                end
            end  
        end

        %------------------------------------------------------------------
        function editAttributeDescription(this, labelName, attributeName, newDescription)
            %editAttributeDescription Modify description of an attribute.
            %   editAttributeDescription(ldc, labelName, attributeName,
            %   description) updates the description of an attribute with
            %   name attributeName that is under the label or sublabel
            %   named labelName in the labelDefinitionCreator object ldc.
            %   labelName is a character vector or string scalar that
            %   uniquely identifies the name of the label of which the
            %   description is to be updated. labelName can also refer to a
            %   sublabel when specified as 'labelName/sublabelName'.
            %   attributeName is a character vector or string scalar that
            %   uniquely identifies the name of the attribute whose
            %   description is to be updated. description is a character
            %   vector or string scalar that contains the new description
            %   for the attribute.
            %
            %   Example: Modify Descriptions of Attributes
            %   ------------------------------------------
            %   % Create a labelDefinitionCreator object.
            %   ldc = labelDefinitionCreator();
            %
            %   % Add a label called Vehicle.
            %   addLabel(ldc, 'Vehicle', labelType.Rectangle);
            %
            %   % Add another label called TrafficLight.
            %   addLabel(ldc, 'TrafficLight', labelType.Rectangle, 'Description', 'Bounding boxes for traffic light');
            %
            %   % Add a sublabel called RedLight for label TrafficLight.
            %   addSublabel(ldc, 'TrafficLight', 'RedLight', labelType.Rectangle, 'Description', 'Bounding boxes for red light');
            %
            %   % Add an attribute called isOn for sublabel RedLight.
            %   addAttribute(ldc, 'TrafficLight/RedLight', 'isOn', attributeType.Logical, false);
            %
            %   % Modify attribute description for isOn.
            %   editAttributeDescription(ldc, 'TrafficLight/RedLight', 'isOn', 'Logical status of light: true if turned on');
            %
            %   % Inspect the labels created.
            %   ldc
            %
            %   See also labelDefinitionCreator/editLabelDescription.
            
            labelDefinitionCreator.checkLabelName(labelName);
            labelDefinitionCreator.checkAttributeName(attributeName);
            labelDefinitionCreator.checkDescription(newDescription);
            newDescription = char(newDescription);
            
            [labelName, sublabelName] = labelDefinitionCreator.splitLabelNames(labelName);
            idx = getLabelRowIndex(this, labelName);
            if isempty(sublabelName)
                % Edit attribute description for a parent label.
                if(isfield(this.labelDefTable.Hierarchy{idx}, attributeName))
                    this.labelDefTable.Hierarchy{idx}.(attributeName).Description = newDescription;
                else
                    error(message('vision:labelDefinitionCreator:DoesNotExist', attributeName, labelName));
                end
            else
                % Edit attribute description for a sublabel.
                if(isfield(this.labelDefTable.Hierarchy{idx}, sublabelName))
                    if(isfield(this.labelDefTable.Hierarchy{idx}.(sublabelName), attributeName))
                        this.labelDefTable.Hierarchy{idx}.(sublabelName).(attributeName).Description = newDescription;
                    else
                        error(message('vision:labelDefinitionCreator:DoesNotExist', attributeName, sublabelName));
                    end
                else
                    error(message('vision:labelDefinitionCreator:DoesNotExist', sublabelName, labelName));
                end
            end  
        end 
        
        %------------------------------------------------------------------
        function editGroupName(this, oldGroupName, newGroupName)
            %editGroupName Rename an existing group.
            %   editGroupName(ldc, oldname, newname) changes the name of
            %   the group oldname to newname. oldname and newname are
            %   character vectors or string scalars containing the old and
            %   new names of the group respectively. This method changes
            %   the group name in all the label definitions having the
            %   group oldname.
            %
            %   Example: Rename Groups
            %   --------------------------------------
            %   % Create a labelDefinitionCreator object.
            %   ldc = labelDefinitionCreator();
            %
            %   % Add labels called Car and Truck with group Vehicle.
            %   addLabel(ldc, 'Car', labelType.Rectangle, 'Group', 'Vehicle');
            %   addLabel(ldc, 'Truck', labelType.Rectangle, 'Group', 'Vehicle');
            %
            %   % Change the name of the group to FourWheeler.
            %   editGroupName(ldc, 'Vehicle', 'FourWheeler');
            %
            %   % Inspect the labels created.
            %   ldc
            %
            %   See also labelDefinitionCreator/editLabelGroup.

            labelDefinitionCreator.checkGroupName(oldGroupName);
            labelDefinitionCreator.checkGroupName(newGroupName);
            
            existingGroupNames = this.listGroups();
            
            groupIdx = find(strcmpi(existingGroupNames, oldGroupName));
            
            if ~isempty(groupIdx)
                this.labelDefTable.Group(groupIdx) = {newGroupName};
            else
                error(message('vision:labelDefinitionCreator:LabelDoesNotExist',...
                    vision.getMessage('vision:labeler:LabelGroupEditBox'),...
                    oldGroupName));                
            end

        end
    end       
    
    %----------------------------------------------------------------------
    % List Labels and Attributes.
    %----------------------------------------------------------------------
    methods (Access = public, Hidden)
        %------------------------------------------------------------------
        function labelNames = listLabels(this, varargin)
            %listLabels List Labels and Sublabels defined.
            %   labelNames = listLabels(ldc) lists the Labels and Sublabels
            %   defined in the labelDefinitionCreator ldc as a N-by-1
            %   array of strings.
            %
            %   Example: List labels in a Bird Label Definition Hierarchy
            %   ---------------------------------------------------------
            %   % Create a labelDefinitionCreator.
            %   ldc = labelDefinitionCreator();
            %
            %   % Create a bird hierarchy with beaks and wings.
            %   addLabel(ldc, 'Bird', labelType.Rectangle);
            %   addSublabel(ldc, 'Bird', 'Beak', labelType.Rectangle);
            %   addSublabel(ldc, 'Bird', 'Wing', labelType.Rectangle);
            %
            %   ldc.addAttribute(ldc, 'Bird', 'Color', attributeType.List, {'White', 'Blue', 'Pink'});
            %
            %   % Now list all the labels in the definition.
            %   labelNames = listLabels(ldc)
            %   
            %   See also labelDefinitionCreator/listAttributes.
            narginchk(1,2);
            
            if nargin == 1
                includeSublabels = false;
            else
                validateattributes(varargin{1}, {'logical'}, {'scalar'}, 'listLabels', 'includeSublabels', 2);
                includeSublabels = varargin{1};
            end
            
            labelNames = [];
            if noLabelsLoaded(this)
                labelNames = []; 
            else
                numLabels = height(this.labelDefTable);
                for idx = 1:numLabels
                    % List Labels.
                    if numLabels == 1
                        thislabelName = this.labelDefTable.Name;
                    else
                        thislabelName = this.labelDefTable.Name{idx};
                    end
                    labelNames = [labelNames; string(thislabelName)]; %#ok<AGROW>
                     if includeSublabels
                         hierarchy = this.labelDefTable.Hierarchy{idx};
                         if ~isempty(hierarchy)
                            hierFields = fieldnames(hierarchy);
                            for fIdx = 1:numel(hierFields)
                                % List Sublabels.
                                if labelDefinitionCreator.isSublabel(hierarchy.(hierFields{fIdx}))
                                    labelNames = [labelNames; ...
                                    string([this.labelDefTable.Name{idx}, '/', hierFields{fIdx}])]; %#ok<AGROW>
                                end
                            end
                         end
                     end
                end
            end
        end
        
        %------------------------------------------------------------------
        function attributeNames = listAttributes(this, varargin)
            %listAttributes List Attributes defined.
            %   attributeNames = listAttributes(ldc) lists the Attributes
            %   defined in the labelDefinitionCreator ldc as a N-by-1
            %   array of strings.
            %
            %   attributeNames = listAttributes(ldc, labelName) lists the
            %   Attributes defined for the label with name labelName.
            %   labelName is a character vector or string scalar specifying
            %   the name of the label or sublabel of which the attributes
            %   are listed. labelName could also refer to a Sublabel when
            %   it is specified as 'labelName/sublabelName'.
            %
            %   Example: List labels in a Bird Label Definition Hierarchy
            %   ---------------------------------------------------------
            %   % Create a labelDefinitionCreator.
            %   ldc = labelDefinitionCreator();
            %
            %   % Create a bird label hierarchy including beaks and wings.
            %   addLabel(ldc, 'Bird', labelType.Rectangle);
            %   addSublabel(ldc, 'Bird', 'Beak', labelType.Rectangle);
            %   addSublabel(ldc, 'Bird', 'Wing', labelType.Rectangle);
            %
            %   % Add attributes for the Bird itself, and for the beak.
            %   addAttribute(ldc, 'Bird', 'Color', attributeType.List, {'White', 'Blue', 'Pink'});
            %   addAttribute(ldc, 'Bird/Beak', 'Shape', attributeType.List, {'Pointy', 'Curved', 'Flat'});
            %
            %   % Now list the attribute names.
            %   attributeNames = listAttributes(ldc, 'Bird');
            %   
            %   See also labelDefinitionCreator/listLabels.
            
            narginchk(1,2);
            
            attributeNames = [];
            
            if nargin == 1
                labelName = [];
                sublabelName = [];
            else
                labelName = varargin{1};
                labelDefinitionCreator.checkLabelName(labelName);
                [labelName, sublabelName] = labelDefinitionCreator.splitLabelNames(labelName);
            end
            
            if noLabelsLoaded(this)
                attributeNames = []; 
            else    
                for lidx = 1:size(this.labelDefTable, 1)
                    thislabelname = this.labelDefTable.Name{lidx};
                    if ~isempty(labelName) && ~strcmpi(thislabelname, labelName)
                        continue;
                    end
                    hierarchy = this.labelDefTable.Hierarchy{lidx};
                    if ~isempty(hierarchy)
                        hierFields = fieldnames(hierarchy);
                        for idx = 1:numel(hierFields)
                            % List Attributes of Labels.
                            if (labelDefinitionCreator.isAttribute(hierarchy.(hierFields{idx})) && isempty(sublabelName))
                                attributeNames = [attributeNames; string([thislabelname, '/', hierFields{idx}])]; %#ok<AGROW>
                            end
                            % Now look in Sublabels.
                            if labelDefinitionCreator.isSublabel(hierarchy.(hierFields{idx}))
                                thissublabelname = hierFields{idx};
                                sublabelhier = hierarchy.(thissublabelname);
                                sublabelhierFields = fieldnames(sublabelhier);
                                for sidx = 1:numel(sublabelhierFields)
                                    % List Attributes of Sublabels.
                                    if ~isempty(labelName) && ~strcmpi([thislabelname ,'/', thissublabelname], [labelName, '/', sublabelName])
                                        continue;
                                    end
                                    if labelDefinitionCreator.isAttribute(sublabelhier.(sublabelhierFields{sidx}))
                                        attributeNames = [attributeNames; string([thislabelname, '/', thissublabelname, '/', sublabelhierFields{sidx}])]; %#ok<AGROW>
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        
        %------------------------------------------------------------------
        function groupNames = listGroups(this)
            if noLabelsLoaded(this)
                groupNames = []; 
            else
                groupNames= this.labelDefTable.Group;
            end
        end        
    end
    
    %----------------------------------------------------------------------
    % Private: Helper methods.
    methods (Access = private)
        %------------------------------------------------------------------
        function TF = noLabelsLoaded(this)
            TF = size(this.labelDefTable, 1) == 0;
        end
        
        %------------------------------------------------------------------
        function createFirstLabel(this, labelName, typeOfLabel, labelColor, labelGroup, labelDescription)
            % Inputs are already validated in public API.
            
            % Make sure that this is indeed the first row. Error message is
            % internal only.
            assert(size(this.labelDefTable,1) == 0, 'createFirstLabel: Attempting to insert first row into table, when rows already exist.');
            
            [labelName, sublabelName] = labelDefinitionCreator.splitLabelNames(labelName);
            if isempty(sublabelName)
                if typeOfLabel == labelType.PixelLabel
                    this.labelDefTable = table({labelName}, typeOfLabel, {labelColor}, {1}, {labelGroup}, {labelDescription}, {[]}); % all pixellabelIDs start from 1.
                else              
                    this.labelDefTable = table({labelName}, typeOfLabel, {labelColor}, {[]}, {labelGroup}, {labelDescription}, {[]});
                end
                this.labelDefTable.Properties.VariableNames = {'Name', 'Type', 'LabelColor','PixelLabelID', 'Group', 'Description', 'Hierarchy'};
            else
                % defining a sublabel prior to defining the label.
                error(message('vision:labelDefinitionCreator:DefineLabelFirst', vision.getMessage('vision:labeler:NewSublabelButtonTitle'), vision.getMessage('vision:labeler:Sublabels')));
            end
        end
        
        %------------------------------------------------------------------
        function appendLabel(this, labelName, typeOfLabel, labelColor, labelGroup, labelDescription)
            % Inputs are already validated in public API.
            
            % Make sure that we are indeed appending to the table. Error
            % message is internal only.
            assert(size(this.labelDefTable,1) ~= 0, 'appendLabel: Attempting to append rows into table, when first row does not exist.');
            
            % Check if name clashes with existing label
            labelnames = listLabels(this, true); % includeSublabels
            if any(cellfun(@(s) isequal(labelName, s), labelnames))
                error(message('vision:labelDefinitionCreator:NameClashWithExistingLabel', labelName));
            end
            
            [labelName, sublabelName] = labelDefinitionCreator.splitLabelNames(labelName);
            if isempty(sublabelName)
                
                % The labels are ordered by the group names in the label
                % definition table. This is to mimic the app behavior.
                % Hence we are calculating here the id for inserting the
                % labels
                existingGroupNames = this.listGroups();
                lastGroupIdx = find(strcmp(existingGroupNames, labelGroup), 1, 'last');
                
                if isempty(lastGroupIdx)
                    lastGroupIdx = numel(existingGroupNames);
                end
                    
                if typeOfLabel == labelType.PixelLabel
                    nextPixelLabelID = getNextPixelLabelID(this);
                    this.labelDefTable = [this.labelDefTable(1:lastGroupIdx,:);...
                        {labelName, typeOfLabel, {labelColor}, {nextPixelLabelID}, {labelGroup}, {labelDescription}, []};...
                        this.labelDefTable(lastGroupIdx+1:end,:)];
                else                
                    this.labelDefTable =[this.labelDefTable(1:lastGroupIdx,:);...
                        {labelName, typeOfLabel, {labelColor}, {[]}, {labelGroup}, {labelDescription}, []};...
                        this.labelDefTable(lastGroupIdx+1:end,:)];
                end
            else
                error(message('vision:labelDefinitionCreator:ReferenceNotCreate', labelName, sublabelName));
            end
        end
                
        %------------------------------------------------------------------
        function addSublabelAttribute(this, labelName, sublabelName, attributeName, typeOfAttribute, attributeDefaults, attributeDescription)
            % Inputs are already validated in the public API. No input
            % validation done for private method.        
            
            idx = getLabelRowIndex(this, labelName);
            
            % Disable adding attributes for unsupported labelTypes.
            this.checkUnsupportedTypes(idx, 'attribute');

            hierarchyStruct = this.getHierarchyAt(idx);
            if ~isfield(hierarchyStruct, sublabelName)
                error(message('vision:labelDefinitionCreator:SublabelNotDefined', sublabelName, labelName));
            end
            if isfield(hierarchyStruct.(sublabelName), attributeName)
                if labelDefinitionCreator.isAttribute(hierarchyStruct.(sublabelName).(attributeName))
                    error(message('vision:labelDefinitionCreator:AlreadyDefined', attributeName, vision.getMessage('vision:labeler:NewAttributeButtonTitle'), sublabelName));
                end
            end
            attrStruct = labelDefinitionCreator.createAttrStruct(typeOfAttribute, attributeDefaults, attributeDescription);
            hierarchyStruct.(sublabelName).(attributeName) = attrStruct;
            this.labelDefTable.Hierarchy{idx} = hierarchyStruct;
        end        
        
        %------------------------------------------------------------------
        function nextID = getNextPixelLabelID(this)
            
            maxID = intmax('uint8');
            
            if noLabelsLoaded(this)
                nextID = 1;
            else
                % Find next available integer value.
                idArray = [];
                for idx = 1:height(this.labelDefTable)
                    val = this.labelDefTable.PixelLabelID{idx};
                    if isnumeric(val) && ~isempty(val)
                        idArray(end+1) = val; %#ok<AGROW>
                    end
                end
                % Get all the entries in 1:1:maxD which are NOT in idArray,
                % and return the minimum value from it.
                nextID = min(setdiff(1:1:maxID, idArray));
            end
            
            if nextID > maxID
                % Disallow creating more than 255 pixel labels: our
                % representation of label matrices is going to break at
                % this point. Internal only error message.
                error(message('vision:labelDefinitionCreator:MaxPixelLabelID', num2str(maxID)));
            end
            
        end
        
        %------------------------------------------------------------------
        function idx = getLabelRowIndex(this, labelName)
            isError = false;
            % Check if no labels have been created at all. In this case,
            % error.
            if isempty(this.labelDefTable)
                isError = true;
            else
                % labelDefTable has non-zero number of entries.
                if (height(this.labelDefTable) == 1) 
                    % If number of entries is 1:
                    if strcmpi(this.labelDefTable.Name, labelName)
                        idx = 1;
                    else
                        idx = [];
                    end
                else
                    % More than one entry. Check in cell array.
                    idx = find(ismember(this.labelDefTable.Name, labelName), 1);
                end
                % If no match, error.
                if isempty(idx)
                    isError = true;
                elseif numel(idx) ~= 1
                    % Internal only error.
                    error('labelDefinitionCreator:getLabelRowIndex Error: duplicate label names in rows.');
                end
            end
            
            if isError
                error(message('vision:labelDefinitionCreator:AddLabelFirst', labelName, vision.getMessage('vision:labelDefinitionCreator:SublabelsOrAttributes')));
            end
        end
        
        %------------------------------------------------------------------
        function hierarchyStruct = getHierarchyAt(this, idx)
            hierarchyStruct = this.labelDefTable.Hierarchy(idx);
            if isempty(hierarchyStruct{1})
                % If empty, create it and return.
                hierarchyStruct = struct('Type', this.labelDefTable.Type(idx));
                hierarchyStruct.Description = this.labelDefTable.Description{idx};
            else
               hierarchyStruct = hierarchyStruct{1}; 
            end
        end
        
        %------------------------------------------------------------------
        function checkUnsupportedTypes(this, idx, type)
            % type needs to be either 'attribute' or 'sublabel'.
            if (this.labelDefTable.Type(idx) == labelType.PixelLabel) || ...
                    (this.labelDefTable.Type(idx) == labelType.Scene) || ...
                    (this.labelDefTable.Type(idx) == labelType.Custom)
                if strcmpi(type, 'attribute')
                    error(message('vision:labelDefinitionCreator:InvalidLabelType', vision.getMessage('vision:labeler:Attributes')));
                elseif strcmpi(type, 'sublabel')
                    error(message('vision:labelDefinitionCreator:InvalidLabelType', vision.getMessage('vision:labeler:Sublabels')));
                end
            end            
        end
        
        %------------------------------------------------------------------
        function value = showLabelDetail(this, labelName)
            value = [];
            for lidx = 1:size(this.labelDefTable, 1)
                thislabelname = this.labelDefTable.Name{lidx};
                if ~isempty(thislabelname) && strcmpi(thislabelname, labelName)
                    % name refers to a label.
                    value.Name = string(thislabelname);
                    value.Type = this.labelDefTable.Type(lidx);
                    value.LabelColor = this.labelDefTable.LabelColor(lidx);
                    value.Group = string(this.labelDefTable.Group{lidx});
                    value.Attributes = [];
                    value.Sublabels = [];
                    
                    hierarchy = this.labelDefTable.Hierarchy{lidx};
                    if ~isempty(hierarchy)
                        hierFields = fieldnames(hierarchy);
                        for idx = 1:numel(hierFields)
                            % List Attributes of Labels.
                            if labelDefinitionCreator.isAttribute(hierarchy.(hierFields{idx}))
                                thisattributename = hierFields{idx};
                                value.Attributes = [value.Attributes, string(thisattributename)];
                            elseif labelDefinitionCreator.isSublabel(hierarchy.(hierFields{idx}))
                                thisattributename = hierFields{idx};
                                value.Sublabels = [value.Sublabels, string(thisattributename)];
                            end
                        end
                    end
                    value.Description = this.labelDefTable.Description{lidx};
                    break
                end
            end
        end
        
        %------------------------------------------------------------------
        function value = showSublabelDetail(this, labelName)
            value = [];
            for lidx = 1:size(this.labelDefTable, 1)
                thislabelname = this.labelDefTable.Name{lidx};
                hierarchy = this.labelDefTable.Hierarchy{lidx};
                if ~isempty(hierarchy)
                    hierFields = fieldnames(hierarchy);
                    for idx = 1:numel(hierFields)
                        if labelDefinitionCreator.isSublabel(hierarchy.(hierFields{idx}))
                            thissublabelname = hierFields{idx};
                            if ~isempty(thissublabelname) && ...
                                    (strcmpi([thislabelname, '/', thissublabelname], labelName) || ...
                                    strcmpi([thislabelname, '\', thissublabelname], labelName))
                                % found sublabel to show details for.
                                value.Name = string(thissublabelname);
                                value.Type = hierarchy.(thissublabelname).Type;
                                if isprop(hierarchy.(thissublabelname), 'labelColor')
                                    value.LabelColor = hierarchy.(thissublabelname).LabelColor;
                                end
                                value.Attributes = [];
                                value.Sublabels = [];
                                sublabelhier = hierarchy.(thissublabelname);
                                sublabelhierFields = fieldnames(sublabelhier);
                                for sidx = 1:numel(sublabelhierFields)
                                    % Include attributes of sublabels
                                    % in detail view.
                                    if labelDefinitionCreator.isAttribute(sublabelhier.(sublabelhierFields{sidx}))
                                        thisattributename = sublabelhierFields{sidx};                                            
                                        value.Attributes = [value.Attributes, string(thisattributename)];
                                    end
                                end
                                
                                value.Description = hierarchy.(thissublabelname).Description;
                                break
                            end
                        end
                    end
                end
            end 
        end
        
        %------------------------------------------------------------------
        function value = showAttributeDetail(this, labelName)
            value = [];
            for lidx = 1:size(this.labelDefTable, 1)
                thislabelname = this.labelDefTable.Name{lidx};
                hierarchy = this.labelDefTable.Hierarchy{lidx};
                if ~isempty(hierarchy)
                    hierFields = fieldnames(hierarchy);
                    for idx = 1:numel(hierFields)
                        % Attribute at the level of label.
                        if labelDefinitionCreator.isAttribute(hierarchy.(hierFields{idx}))
                            thisattributename = hierFields{idx};
                            if ~isempty(thisattributename) && ...
                                    (strcmpi([thislabelname, '/', thisattributename], labelName) || ...
                                     strcmpi([thislabelname, '\', thisattributename], labelName))
                                % found attribute to show details for.
                                value.Name = string(thisattributename);
                                if isfield(hierarchy.(thisattributename), 'DefaultValue')
                                    value.Type = labelDefinitionCreator.getAttrTypeFromDefaultValue( ...
                                             hierarchy.(thisattributename).DefaultValue);
                                    value.DefaultValue = hierarchy.(thisattributename).DefaultValue;
                                elseif isfield(hierarchy.(thisattributename), 'ListItems')
                                    value.Type = attributeType.List;
                                    value.ListItems = hierarchy.(thisattributename).ListItems;
                                else
                                    assert(false, 'Incorrect attribute specification.');
                                end
                                value.Description = hierarchy.(thisattributename).Description;
                            end
                        elseif labelDefinitionCreator.isSublabel(hierarchy.(hierFields{idx}))
                            thissublabelname = hierFields{idx};
                            sublabelhier = hierarchy.(thissublabelname);
                            sublabelhierFields = fieldnames(sublabelhier);
                            for sidx = 1:numel(sublabelhierFields)
                                % Include attributes of sublabels
                                % in detail view.
                                if labelDefinitionCreator.isAttribute(sublabelhier.(sublabelhierFields{sidx}))
                                    thisattributename = sublabelhierFields{sidx};                                            
                                    if ~isempty(thisattributename) && ...
                                       (strcmpi([thislabelname, '/', thissublabelname, '/', thisattributename], labelName) || ...
                                        strcmpi([thislabelname, '\', thissublabelname, '\', thisattributename], labelName))
                                        % found attribute to show details for.
                                        value.Name = string(thisattributename);
                                        if isfield(sublabelhier.(thisattributename), 'DefaultValue')
                                            value.Type = labelDefinitionCreator.getAttrTypeFromDefaultValue( ...
                                                     sublabelhier.(thisattributename).DefaultValue);
                                            value.DefaultValue = sublabelhier.(thisattributename).DefaultValue;
                                        elseif isfield(sublabelhier.(thisattributename), 'ListItems')
                                            value.Type = attributeType.List;
                                            value.ListItems = sublabelhier.(thisattributename).ListItems;
                                        else
                                            assert(false, 'Incorrect attribute specification.');
                                        end
                                        value.Description = sublabelhier.(thisattributename).Description;
                                    end
                                end
                            end 
                        end
                    end
                end
            end 
        end
    end
    
    %----------------------------------------------------------------------
    % Private: static helper methods.
    methods (Access = private, Static)
        %------------------------------------------------------------------
        function validTable = constructValidTable(inputTable, status)
            % This helper method adds PixelLabelData and Hierarchy columns
            % to all input tables, so that it is consistent internally
            % (even if the fields are empty) and ready to be updated.
            numLabels = height(inputTable);
            if ~status.haspixellabelID
                pixellabelID = cell(numLabels, 1);
            else
                pixellabelID = inputTable.PixelLabelID;
            end
            if ~status.hashierarchy
                hierarchy = cell(numLabels, 1);
            else
                hierarchy = inputTable.Hierarchy;
            end
            if ~status.hascolor
                color = cell(numLabels, 1);
                color(:) = {''};
            else
                color = inputTable.LabelColor;
            end
            
            validTable = table(inputTable.Name, inputTable.Type, color, pixellabelID, inputTable.Group, inputTable.Description, hierarchy);
            validTable.Properties.VariableNames = {'Name', 'Type', 'LabelColor', 'PixelLabelID', 'Group', 'Description', 'Hierarchy'};
        end
        
        %------------------------------------------------------------------
        function [labelName, sublabelName] = splitLabelNames(inputName)
            nameCell = strsplit(inputName, {'/','\'});
            if numel(nameCell) == 1
                labelName = nameCell{1};
                sublabelName = [];
            elseif numel(nameCell) == 2
                labelName = nameCell{1};
                sublabelName = nameCell{2};
                labelDefinitionCreator.checkSublabelName(sublabelName);
            else
                error(message('vision:labelDefinitionCreator:IncorrectSpecification'));
            end
            if ~isvarname(labelName) 
                error(message('vision:labelDefinitionCreator:NameInvalid', labelName));
            end
            if ~isempty(vision.internal.labeler.validation.invalidNames(labelName))
                error(message('vision:labeler:LabelNameIsReserved', labelName));
            end
        end
        
        %------------------------------------------------------------------
        function outStruct = removeFieldFromStruct(inStruct, componentName, parentName)
            if(isfield(inStruct, componentName))
                outStruct = rmfield(inStruct, componentName);
            else
                error(message('vision:labelDefinitionCreator:DoesNotExist', componentName, parentName));
            end            
        end
        
        %------------------------------------------------------------------
        function attrStruct = createAttrStruct(typeOfAttribute, attributeDefaults, attributeDescription)
            attrStruct = struct('Description', attributeDescription);
            if isequal(typeOfAttribute, attributeType.List)
                attrStruct.ListItems = attributeDefaults;
            else
                attrStruct.DefaultValue = attributeDefaults;
            end
        end
        
        %------------------------------------------------------------------
        function trimmedTable = removePixelColumnIfEmpty(labelDefTable)
            % At this point, it is guaranteed that labelDefTable contains
            % PixelLabelID column.
            trimmedTable = labelDefTable;
            if isempty(labelDefTable)
                return
            end
            isPixelEmpty = true;
            for idx = 1:height(labelDefTable)
                if ~isempty(labelDefTable.PixelLabelID{idx})
                    isPixelEmpty = false;
                    break;
                end
            end
            if isPixelEmpty
                trimmedTable.PixelLabelID = [];
            end
        end
        
        %------------------------------------------------------------------
        function trimmedTable = removeHierarchyColumnIfEmpty(labelDefTable)
            % At this point, it is guaranteed that labelDefTable contains
            % Hierarchy column.
            trimmedTable = labelDefTable;
            if isempty(labelDefTable)
                return
            end            
            isHierarchyEmpty = true;
            for idx = 1:height(labelDefTable)
                if ~isempty(labelDefTable.Hierarchy{idx})
                    isHierarchyEmpty = false;
                    break;
                end
            end
            if isHierarchyEmpty
                trimmedTable.Hierarchy = [];
            end
        end

        %------------------------------------------------------------------
        function typeOut = attributeTypeToStr(typeIn)
            typeOut = ['Attribute: ', char(typeIn)];
        end        
        
        %------------------------------------------------------------------
        function attrType = getAttrTypeFromDefaultValue(defValue)
            % Return attribute type with default value as input.
            if isstring(defValue) || ischar(defValue)
                attrType = attributeType.String;
            elseif islogical(defValue)
                attrType = attributeType.Logical;
            elseif isnumeric(defValue)
                attrType = attributeType.Numeric;
            elseif iscell(defValue)
                attrType = attributeType.List;
            else
                % Internal only error
                error('labelDefinitionCreator:getTypeFromDefaultValue: unknown input type');
            end
        end       
    end
    
    %----------------------------------------------------------------------
    % Private: static validation methods.
    methods (Access = private, Static)
        %------------------------------------------------------------------
        function [inputarg, status] = parseInputs(varargin)
            
            narginchk(1,2);
            inputarg = varargin{1};
            
            status = struct('islabelDef', false, ...
                             'haspixellabelID', false, ...
                             'hashierarchy', false, ...
                             'isempty', true);
            
            if isempty(inputarg)
                % do nothing.
            elseif istable(inputarg{1})                
                % Check if it is a valid Label Definition.
                inputarg = vision.internal.labeler.validation.checkLabelDefinitions(inputarg{1});
                status.islabelDef = true;
                status.haspixellabelID = ~isempty(find(ismember(inputarg.Properties.VariableNames,'PixelLabelID'), 1));
                status.hashierarchy = ~isempty(find(ismember(inputarg.Properties.VariableNames,'Hierarchy'), 1));
                status.hascolor = ~isempty(find(ismember(inputarg.Properties.VariableNames,'LabelColor'), 1));
                
                inputarg = labelDefinitionCreator.constructValidTable(inputarg, status);
                status.isempty = false;
            else
            % if anything else, error out.
                error(message('vision:labelDefinitionCreator:InvalidInputs'));
            end
        end
        
        %------------------------------------------------------------------
        function tf = checkLabelName(value)
            validateattributes(value, {'char', 'string'}, {'scalartext'}, mfilename, vision.getMessage('vision:labelDefinitionCreator:CharOrString'));
            tf = true;
        end 
        
        %------------------------------------------------------------------
        function tf = checkLabelColor(value)
            validateattributes(value, {'double'}, {'>=', 0, '<=', 1,'nrows',1,'ncols',3}, mfilename, "color value needs");
            % Label and sub-label cannot be assigned yellow color
            % [R G B] ~= [1 1 0].
            if isequal(value , [1 1 0])
                error(message('vision:labelDefinitionCreator:InvalidColor'));
            end
            tf = true;
        end
        
        %------------------------------------------------------------------
        function tf = checkSublabelName(value)
            validateattributes(value, {'char', 'string'}, {'scalartext'}, mfilename, 'sublabelName');
            % Make sure sublabelName is a valid variable name.
            if ~isvarname(value)
                error(message('vision:labelDefinitionCreator:NameInvalid', value));
            end
            if ~isempty(vision.internal.labeler.validation.invalidNames(value))
                error(message('vision:labeler:LabelNameIsReserved', value));
            end
            tf = true;
        end
        
        %------------------------------------------------------------------
        function tf = checkAttributeName(value)
            validateattributes(value, {'char', 'string'}, {'scalartext'}, mfilename, 'attributeName');
            % Make sure attributeName is a valid variable name.
            if ~isvarname(value)
                error(message('vision:labelDefinitionCreator:NameInvalid', value));
            end
            if ~isempty(vision.internal.labeler.validation.invalidNames(value))
                error(message('vision:labeler:LabelNameIsReserved', value));
            end
            tf = true;
        end
        
        %------------------------------------------------------------------
        function tf = checkLabelType(value)
            if isenum(value)
                validateattributes(value, {'labelType'}, {'scalar'}, mfilename, 'typeOfLabel');
            else
                validateattributes(value, {'string', 'char'}, {'scalartext'}, mfilename, 'typeOfLabel');
            end
            tf = true;
        end
        
        %------------------------------------------------------------------
        function tf = checkAttributeType(value)
            if isenum(value)
                validateattributes(value, {'attributeType'}, {'scalar'}, mfilename, 'typeOfAttribute');
            else
                validateattributes(value, {'string', 'char'}, {'scalartext'}, mfilename, 'typeOfAttribute');
            end
            tf = true;
        end        
        
        %------------------------------------------------------------------
        function type = string2labelType(value, isSublabel)
            % Return labelType with string/char inputs.
            if isa(value, 'string') || isa(value, 'char')
                % Allow the shortform string 'r' for 'Rectangle'
                if strcmp(value,'r') || strcmp(value,'R')
                    value = 'Rectangle';
                end
                validvalue = validatestring(value, ...
                   {'Rectangle', 'RotatedRectangle', 'Point', 'Line', ...
                   'ProjectedCuboid', 'Polygon', 'Scene', 'PixelLabel', 'Custom'});
                switch validvalue
                    case 'Rectangle'
                        validtype = labelType.Rectangle;
                    case 'RotatedRectangle'
                        validtype = labelType.RotatedRectangle;
                    case 'Line'
                        validtype = labelType.Line;
                    case 'Point'
                        validtype = labelType.Point;
                    case 'Polygon'
                        validtype = labelType.Polygon;
                    case 'ProjectedCuboid'
                        validtype = labelType.ProjectedCuboid;                        
                    case 'Scene'
                        validtype = labelType.Scene;
                    case 'PixelLabel'
                        validtype = labelType.PixelLabel;
                    case 'Custom'
                        validtype = labelType.Custom;
                    otherwise
                        assert(false, 'Incorrect labelType specification');
                end
                type = validtype;
            else
                % Has to be a labelType (checked in checkLabelType)
                type = value;
            end
            
            if isSublabel
                % Sublabels cannot be of type PixelLabel, Scene, or Custom.
                if (value == labelType.PixelLabel) || ...
                   (value == labelType.Scene) || ...
                   (value == labelType.Custom)
                   error(message('vision:labelDefinitionCreator:InvalidSubLabelType'));
                end
            end
        end
        
        %------------------------------------------------------------------
        function type = string2attributeType(value)
            % Return labelType or attributeType with string/char inputs.
            if isa(value, 'string') || isa(value, 'char')
                validvalue = validatestring(value, ...
                             {'List', 'Logical', 'Numeric', 'String'});
                switch validvalue
                    case 'List'
                        validtype = attributeType.List;
                    case 'Logical'
                        validtype = attributeType.Logical;
                    case 'Numeric'
                        validtype = attributeType.Numeric;
                    case 'String'
                        validtype = attributeType.String;
                    otherwise
                        assert(false, 'Incorrect attributeType specification');
                end
                type = validtype;
            else
                % Has to be a attributeType (checked in checkAttributeType)
                type = value;
            end            
        end        
        
        %------------------------------------------------------------------
        function checkAttributeDefault(value, typeOfAttribute)
            % Validate that the default value provided matches the type of
            % the attribute specified. 
            if typeOfAttribute == attributeType.List
                validateattributes(value, {'cell'}, {'vector'}, mfilename, 'attributeDefault');
                for idx = 1:numel(value)
                    if ~(ischar(value{idx}) || isstring(value{idx}))
                        error(message('vision:labelDefinitionCreator:ListItemsInvalid'));
                    end
                end
            elseif typeOfAttribute == attributeType.Logical
                validateattributes(value, {'logical'}, {'scalar'}, mfilename, 'attributeDefault');
            elseif typeOfAttribute == attributeType.Numeric
                validateattributes(value, {'numeric'}, {'scalar'}, mfilename, 'attributeDefault');
            elseif typeOfAttribute == attributeType.String
                validateattributes(value, {'char', 'string'}, {'scalartext'}, mfilename, 'attributeDefault');
            end
        end
        
        %------------------------------------------------------------------
        function tf = checkDescription(value)
            validateattributes(value, {'char', 'string'}, {'scalartext'}, mfilename, 'Description');
            tf = true;
        end
        
        %------------------------------------------------------------------        
        function tf = checkGroupName(value)
            validateattributes(value, {'char', 'string'}, {'scalartext'}, mfilename, 'Group');
            
            if ~isvarname(value)
                error(message('vision:labelDefinitionCreator:NameInvalid', value));
            end
            
            tf = true;            
        end
        
        %------------------------------------------------------------------
        function tf = isSublabel(inStruct)
            tf = false;
            if (isfield(inStruct, 'Type') && ...
                ~(isfield(inStruct, 'DefaultValue') || isfield(inStruct, 'ListItems')) && ...
                isfield(inStruct, 'Description'))
                tf = true;
            end
        end
        
        %------------------------------------------------------------------
        function tf = isAttribute(inStruct)
            tf = false;
            if (~isfield(inStruct, 'Type') && ...
                (isfield(inStruct, 'DefaultValue') || isfield(inStruct, 'ListItems')) && ...
                isfield(inStruct, 'Description'))
                tf = true;
            end
        end
    end
    
end