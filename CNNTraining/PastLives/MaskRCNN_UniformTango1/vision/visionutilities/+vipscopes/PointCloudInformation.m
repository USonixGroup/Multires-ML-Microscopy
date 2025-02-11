classdef PointCloudInformation < matlabshared.scopes.BaseDialog
    %PointCloudInformation   Define the PointCloudInformation class.
    
    %   Copyright 2022 The MathWorks, Inc.

    properties
        DataType;
        DisplayDataType;
        Orientation;
    end
    
    properties (SetAccess = protected)
        PlaybackInfo;
        SourceType;
        SourceLocation;
        PointCloudSize;
    end
    
    properties (Access = protected)
        DataReleasedListener;
        SourceNameChangedListener;
    end
    
    methods
        function this = PointCloudInformation(hScopeApp)
            %PointCloud   Construct the PointCloud class.
            
            this@matlabshared.scopes.BaseDialog("Point Cloud Information", hScopeApp);
            
            % Listen to the DataReleased and SourceNameChanged events.
            % These must be reflected in the display, but are done in ways
            % that do not otherwise trigger updates (i.e., by colormap
            % scaling changes)
            this.DataReleasedListener = addlistener(hScopeApp, 'DataReleased', ...
                @(h, ev) dataReleased(this));
            this.SourceNameChangedListener = addlistener(hScopeApp, 'SourceNameChanged', ...
                @(h, ev) sourceNameChanged(this));
        end
        
        function set.DisplayDataType(this, newType)
            
            this.DisplayDataType = newType;
            
            % Only update data if dialog open
            hDlg = this.Dialog;
            if ~isempty(hDlg)
                update(this);
                refresh(hDlg);
            end
        end
    end
    
    methods (Hidden)
        function dlgstruct = getDialogSchema(this,arg)
            %GetDialogSchema Construct VideoInfo dialog
            
            % Get the datasource-specific dialog plug-in
            % A vector of structs, containing fields:
            %   .Title: title of next section of info section
            %   .Widgets: a cell-array passed to DDG2ColText
            %
            if ~isempty(this.Application.DataSource)
                % If the source is invalid, clear the display data type
                if strcmp(this.Application.DataSource.ErrorStatus,'failure')
                    this.DisplayDataType = '';
                end
            end
            
            infoStruct = [GetCommonInfo(this) this.PlaybackInfo];
            
            % Merge all info groups into one, contiguous group
            % (Basically, ignore separate group names, unlike
            %  key help which preserves group names in the dialog)
            %
            infoStruct = local_MergeGroups(infoStruct);
            
            % Create DDG groups from help database
            %
            Ngroups = numel(infoStruct);
            DDG_Group = cell(1, Ngroups);
            for i=1:Ngroups
                DDG_Group{i} = DDG2ColText(infoStruct(i).Title, ...
                    infoStruct(i).Widgets, i);
            end
            
            % Collect all groups into a panel
            %
            cAll.Tag = 'PointCloudInfoPanel';
            cAll.Type = 'panel';
            cAll.Items = DDG_Group;
            cAll.LayoutGrid = [Ngroups 1];
            
            % Return top-level DDG dialog structure
            %
            dlgstruct                     = getDialogSchema@uiservices.BaseDialog(this, arg);
            dlgstruct.ExplicitShow        = false;
            dlgstruct.DialogTitle         = this.TitlePrefix;
            dlgstruct.Items               = {cAll};
            dlgstruct.StandaloneButtonSet = {'OK'};
            dlgstruct.DialogTag           = 'PointCloudInfo';
        end
        
        function update(this)
            %UPDATE Update Point Cloud Info object to react to a new movie (source data object)
            
            source = this.Application.DataSource;
            ptCloud  = this.Application.Visual;
            if isempty(source)
                return;
            end
            
            if any(strcmp(source.ErrorStatus,{'cancel','failure'}))
                % If the source is invalid, clear the display data type and color space
                this.DisplayDataType = '';
                return;
            end
            
            % Need to reset these properties in response to a new data source:
            this.SourceType = source.Type;
            maxDimensions   = getMaxDimensions(source, 1);

            if (~(numel(maxDimensions)>2))
                return;
            end

            this.PointCloudSize  = sprintf('%d H x %d W x %d', maxDimensions(1:3));
            sourceNameChanged(this, false);
            
            if ptCloud.IsOrganized
                s = 'Organized';
            else
                s = 'Unorganized';
            end
            
            this.Orientation = s; 
            this.DataType   = getDataTypes(source, 1);
            
            % Update the keyboard help datasource-specific entries:
            %
            this.PlaybackInfo = getDataInfo(source.Controls);
            this.TitlePrefix  = getTitleString(this);
            
        end
    end
end

% -------------------------------------------------------------------------
function sourceNameChanged(this, update)
%SOURCENAMECHANGED Update the source name.

this.SourceLocation = this.Application.DataSource.Name;

% Force an update if it is not specifically suppressed with a false flag.
if (nargin < 2 || update) && ~isempty(this.Dialog)
    refresh(this.Dialog);
end
end

% -------------------------------------------------------------------------
function group = local_MergeGroups(infoStruct)
% Merge all separate groups into a single 'Video Info' group
% Ignore group names, etc

% Merge all widgets
w = {};
for i=1:numel(infoStruct)
    w = [w; infoStruct(i).Widgets]; %#ok
end
group.Title = 'Point Cloud Info';
group.Widgets = w;
end

% -------------------------------------------------------------------------
function common_group = GetCommonInfo(this)
% Get "Common Group" of DDG help entries

% Widgets are defined as 'Tags','Value'. Tags are also the message Ids used
% to get the actual display strings in DDG2ColText()

common_group.Title = 'Common';
if strcmp(this.SourceType, 'Streaming')
    common_group.Widgets = { ...
        'LabelFrameSize', this.PointCloudSize; ...
        'LabelSrcDataType',  this.DataType; ...
        'LabelDisplayDataType', this.DisplayDataType};
else
    common_group.Widgets = { ...
        'LabelSrcType',  this.SourceType; ...
        'LabelSrcName',  this.SourceLocation; ...
        'LabelFrameSize',   this.PointCloudSize; ...
        'LabelSrcDataType',  this.DataType; ...
        'LabelDisplayDataType', this.DisplayDataType};
end
end

% -------------------------------------------------------------------------
function grp = DDG2ColText(groupName, entries, grpIdx)
%DDG2ColText Create a 2-column group of text widgets.
%   Creates a DDG group of text widgets in a 2-column
%   format.
%
%  groupName: visible name of the group widget
%  entries: Nx2 cell-array of strings to render
%           using text widgets in a 2-column format
%  grpIdx: row-coordinate to assign to group

% Construct individual text widgets for
% each key binding and description
%
numEntries = size(entries,1); % # Rows
allW = cell(1, 2*numEntries);  % all widgets for this group
for indx = 1:numEntries
    % Construct text widgets for next description and key
    % Store interleaved widgets,
    %  [description1, key1, description2, key2, ...]
    %
    % Name
    w.Type    = 'text';
    w.Tag     = entries{indx,1};
    w.Name    = [uiscopes.message(entries{indx,1}) ':'];
    w.Alignment = 5;  % 4: ctr right, 6: ctr left
    w.Bold    = 0;
    w.RowSpan = [indx indx];
    w.ColSpan = [1 1];
    allW{2*indx-1} = w;
    
    % Value
    w.Type    = 'text';
    w.Tag     = [w.Tag 'Value'];
    w.Name    = ['  ' entries{indx,2}];
    w.Alignment = 5;
    w.Bold = 0;
    w.RowSpan = [indx indx];
    w.ColSpan = [2 2];
    allW{2*indx} = w;
end

% Construct Group widget
%
grp.Tag = 'PointCloudInfoGroupBox';
grp.Type = 'group';
grp.Name = groupName;
grp.Items = allW;  % all widgets
grp.LayoutGrid = [numEntries+1 2]; % internal to group
grp.RowStretch = [zeros(1, numEntries) 1];
grp.ColStretch = [1 1];
grp.RowSpan = [grpIdx grpIdx];   % external for parent
grp.ColSpan = [1 1];
end

% -------------------------------------------------------------------------
function dataReleased(this)

this.PlaybackInfo    = [];
this.DisplayDataType = '';
this.SourceType = '';
this.SourceLocation = '';
this.ImageSize = '';
this.DataType = '';

if isa(this.Dialog, 'DAStudio.Dialog')
    this.Dialog.refresh;
end
end

% [EOF]
