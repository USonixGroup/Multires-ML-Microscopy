function def = mergeLabelDefinitions(allDefinitions)
% Merge all label definitions input allDefinitions, which is a cell array
% of tables. Merging is a three step process:
%
% 1) Standardize all tables by making them have the same variables. This
%    enables the creation of one large table and makes merging data easier.
% 2) Group definition table rows by Name using findgroups.
% 3) Use splitapply to merge data for each label Name. 

% Copyright 2023 The MathWorks, Inc.

% Scan input label definitions for optional variables that might exist.
anyLabelColor   = iScanForVariable(allDefinitions, "LabelColor");
anyPixelLabelID = iScanForVariable(allDefinitions, "PixelLabelID");
anyGroup        = iScanForVariable(allDefinitions, "Group");
anyDescription  = iScanForVariable(allDefinitions, "Description");
anyHierarchy    = iScanForVariable(allDefinitions, "Hierarchy");

% Name and Type are required variables for every label definition table.
defVarNames = ["Name", "Type"];

% If any input definition contains one of the optional variables, then all
% tables must have that variable. Add missing columns. 
if anyLabelColor 
    defVarNames = [defVarNames, "LabelColor"];
end

if anyPixelLabelID
    defVarNames = [defVarNames "PixelLabelID"];
end

if anyGroup
    defVarNames = [defVarNames "Group"];
end

if anyDescription
    defVarNames = [defVarNames, "Description"];
end

if anyHierarchy
    defVarNames = [defVarNames "Hierarchy"];
end

% Standardize label definition data to have same variables and data format.
for i = 1:numel(allDefinitions)

    nrows = height(allDefinitions{i});

    if anyLabelColor
        allDefinitions{i} = iAddVariableIfMissing(allDefinitions{i}, "LabelColor",  repmat({[]},nrows,1));
    
        % LabelColor values may be numeric or cell data. Convert any
        % numeric data to cell data so that color data across multiple
        % groundTruth can be merged. Using cell data allows for empty color
        % values. 
        allDefinitions{i} = iConvertLabelColorToCell(allDefinitions{i});
    end

    if anyPixelLabelID
        allDefinitions{i} = iAddVariableIfMissing(allDefinitions{i}, "PixelLabelID", repmat({[]},nrows,1));
    end
    
    if anyGroup
        allDefinitions{i} = iAddVariableIfMissing(allDefinitions{i}, "Group", repmat({'None'},nrows,1));
    end

    if anyDescription
        allDefinitions{i} = iAddVariableIfMissing(allDefinitions{i}, "Description", repmat({' '},nrows,1));
    end
  
    if anyHierarchy
        allDefinitions{i} = iAddVariableIfMissing(allDefinitions{i}, "Hierarchy", repmat({[]},nrows,1));
    end
    
    % Set table variables to a prescribed order to enable vertcat across
    % all input groundTruth label definitions.
    allDefinitions{i} = allDefinitions{i}(:, defVarNames);
    
end

% Combine all label definition tables.
def = vertcat(allDefinitions{:});

% Merge label definitions by grouping definitions by label name and then
% merge within each group.
G = findgroups(def(:,1));

% Use splitapply to group the data to manually process the grouped items. 
groupedData = splitapply(@iGroupData,def,G);

% Merge label data within each group. Using splitapply here is not feasible
% as it does not properly capture exception causes. By processing the
% groups manually we are able to throw better error messages. 
data = cell(size(groupedData,1),1);
for ii = 1:size(groupedData,1)
    data{ii} = nMergeLabelDefinitions(groupedData{ii,:});
end

% Stack the data to produce a table.
def = vertcat(data{:});

    function out = nMergeLabelDefinitions(varargin)
        % The input length is the same as the number of variables in the
        % combined label definition table. Go through each input and merge
        % the data as needed.
        name = varargin{1};
     
        row = cell(1,numel(varargin));

        % Name: There may be multiple if label names are repeated in input
        % label definitions, the values will be all the same because we
        % group by name. Pick the first one.
        row{1} = varargin{1}(1);

        % Type: Assert all definitions have the same type for a given Name.
        iAssertAllTypesAreEqual(name, varargin{2})
        row{2} = varargin{2}(1);

        idx = 2;
        
        if anyLabelColor
            idx = idx + 1;
            row{idx} = iMergeLabelColor(varargin{idx});
        end

        if anyPixelLabelID
            % Choose the first pixel label ID. The scenario where a class
            % has a different pixel label ID is managed after we see the
            % total number of pixel label classes aggregated across all the
            % input label definitions.
            idx = idx + 1;
            row{idx} = varargin{idx}(1);
        end

        if anyGroup
            idx = idx + 1;           
            row{idx} = iMergeGroups(varargin{idx});
        end
       
        if anyDescription
            idx = idx + 1;
            row{idx} = iMergeDescriptions(varargin{idx});
        end

        if anyHierarchy
            % Hierarchy
            idx = idx + 1;
            iAssertAllHierarchiesAreEqual(name, varargin{idx})
            row{idx} = varargin{idx}(1);
        end

        out = cell2table(row);

    end

def.Properties.VariableNames = cellstr(defVarNames);

% Manage merging of pixel label IDs across all label definitions. Here we
% reset all IDs from 1 to the number of pixel label classes in the merged
% groundTruth. This requires us to rewrite pixel label data, which is done
% in the merge method.
if anyPixelLabelID
    pxType = def.Type == labelType.PixelLabel;
    newIDs = (1:sum(pxType))';
    def.PixelLabelID(pxType) = num2cell(double(newIDs)); 
end

% Merge colors across 
if anyLabelColor
    def.LabelColor = iMergeLabelColorAcrossClasses(def.LabelColor);
end

end

%--------------------------------------------------------------------------
function color = iMergeLabelColor(color)
% Merge colors by choosing the first non-empty color value. If all are
% empty, return empty. This function merges inter-class color labels. If
% there are more than one non-empty color value, the first one is selected.
%
% A second merge pass is required after all definitions are merged to
% resolve intra-class conflicts; see iMergeLabelColorAcrossClasses.

nonempty = cellfun(@(x)~isempty(x),color);
idx = find(nonempty,1);
if isempty(idx)
    color = {[]};
else
    color = color(idx);
end

end

%--------------------------------------------------------------------------
function colors = iMergeLabelColorAcrossClasses(colors)

nonempty = cellfun(@(x)~isempty(x),colors);

if any(nonempty)
    colorVector = vertcat(colors{:});
    uniqueColors = unique(colorVector,'rows','stable');

    % Randomly create unique colors.
    while size(uniqueColors,1) < size(colorVector,1)
        candidateColor = rand(1,3);
        uniqueColors = unique([uniqueColors; candidateColor],'rows','stable');
    end

    % Assign colors back to non-empty entries. Leave empty entries as-is. If
    % the merged groundTruth is loaded into the labeler, the app automatically
    % fills in missing colors.
    uniqueColors = num2cell(uniqueColors,2);
    colors(nonempty) = uniqueColors;
end
end

%--------------------------------------------------------------------------
function group = iMergeGroups(groups)
% When all are None, use None as the group. Otherwise, choose the first
% group name that does not equal None. This way if there is a conflict, the
% first group name wins. 
isNone = strcmp('None',groups);
if all(isNone)
    group = {'None'};
else
    idx = find(~isNone,1);
    group = groups(idx);
end
end

%--------------------------------------------------------------------------
function out = iGroupData(varargin)
out = varargin;
end

%--------------------------------------------------------------------------
function desc = iMergeDescriptions(desc)
% Use the first non-empty description. If all are empty, then return empty.
nonempty = cellfun(@(x)~isempty(deblank(x)),desc);
idx = find(nonempty,1);
if isempty(idx)
    desc = desc{1};
else
    desc = desc{idx};
end
end

%--------------------------------------------------------------------------
function tbl = iAddVariableIfMissing(tbl, varname, values)
if iIsVariableMissing(tbl, varname)
    tbl = addvars(tbl, values, NewVariableNames=varname);
end
end

%--------------------------------------------------------------------------
function found = iScanForVariable(allDefinitions, varname)
found = false;
for i = 1:numel(allDefinitions)
    found = found || ~iIsVariableMissing(allDefinitions{i}, varname);
    if found
        break;
    end
end
end

%--------------------------------------------------------------------------
function iAssertAllTypesAreEqual(name, types)
if numel(name) > 1 && ~all(types(1)==types)
    % When there are more than one class name, all class names must have
    % the same label type.
    error(message('vision:groundTruth:mergeLabelNameHasMultipleTypes',name{1}));
end
end

%--------------------------------------------------------------------------
function iAssertAllHierarchiesAreEqual(name, hierarchies)
if numel(name) > 1 && ~iAllEqualHierarchy(hierarchies)
    error(message('vision:groundTruth:mergeLabelNameHasMultipleHierarchies',name{1}));
end
end

%--------------------------------------------------------------------------
function tf = iIsVariableMissing(tbl, varname)
tf = ~any(strcmp(varname,tbl.Properties.VariableNames));
end

%-------------------------------------------------------------------------- 
function allEqual = iAllEqualHierarchy(h)
% Input is a cell array of hierarchy data. It may be [] or a struct. 

allEqual = true;
for i = 2:numel(h)
    if isempty(h{1})
        allEqual = allEqual & isempty(h{i});
    else
        allEqual = allEqual & isequal(h{1},h{i});
    end
    if ~allEqual
        break;
    end
end
end

%-------------------------------------------------------------------------- 
function def = iConvertLabelColorToCell(def)
if ~iscell(def.LabelColor)
    def.LabelColor = num2cell(def.LabelColor,2);
end
end