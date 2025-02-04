classdef groundTruthHelper < handle
    % groundTruthHelper is a helper class to share some code across
    % groundtruth and groundTruthLidar.
    
    % Copyright 2020 The MathWorks, Inc.
    
    methods(Access = public, Static)
        
        %------------------------------------------------------------------
        function [selectedLabelDefs, selectedLabelNames] = selectLabelsByGroup(groups, allGroupNames, labelDefs)
            if isstring(groups) || ischar(groups)
                groups = cellstr(groups);      
            end
            
            % Expect the list of labels provided to be unique
            groups = unique(groups,'stable');    
            
            % Check if all the groups specified exists
            for idx = 1:numel(groups)
                if ~ismember(groups{idx}, allGroupNames)
                    error(message('vision:groundTruth:groupNotFound', groups{idx}));
                end
            end
            
            [~,indices] = ismember(allGroupNames, groups);
            
            labelIndices = find(indices);
            
            selectedLabelDefs   = labelDefs(labelIndices,:);   
            selectedLabelTypes = selectedLabelDefs.Type;
            pixelLabelTypeIndices = (selectedLabelTypes == labelType.PixelLabel);
            
            labelIndices(pixelLabelTypeIndices) = [];
            nonPxlLabelDefs   = labelDefs(labelIndices,:);
            selectedLabelNames = nonPxlLabelDefs.Name;
            
            if any(pixelLabelTypeIndices)
                selectedLabelNames{end+1} = 'PixelLabelData';
            end
            
            if ischar(selectedLabelNames) || isstring(selectedLabelNames)
                selectedLabelNames = cellstr(selectedLabelNames);
            end
        end    
        
        %------------------------------------------------------------------
        function indexList = labelName2Index(labelNames, allLabelNames)
            
            % Find column indices into LabelData.
            indexList = zeros(numel(labelNames),1);
            for n = 1 : numel(labelNames)
                idx = find( strcmp(allLabelNames, labelNames{n}) );
                if ~isscalar(idx)
                    error(message('vision:groundTruth:labelNotFound',labelNames{n}))
                end
                indexList(n) = idx;
            end
        end
        
        %------------------------------------------------------------------
        function labels = validateLabelNameOrType(labels, allLabelNames, labelTypes)
            
            validateattributes(labels, ...
                {'char', 'string', 'cell', 'labelType'}, ...
                {'nonempty','vector'}, mfilename, 'label names or types');
            
            if isstring(labels) || ischar(labels)
                labels = cellstr(labels);      
            end
            
            if iscellstr(labels)
                pixelLabels = allLabelNames( labelTypes == labelType.PixelLabel );
                
                if ~isempty(intersect(labels,pixelLabels))
                    error(message('vision:groundTruth:pixelLabelSelectByNameNotSupported'))
                end
            end
            
            if ~(iscellstr(labels) || isa(labels, 'labelType'))
                error(message('vision:groundTruth:invalidLabelSpecification'))
            end
             
            if isa(labels, 'labelType')                                
                
                labType = labels;
                tempLabels = {};
                for idx = 1:numel(labType)
                    tempLabels = [tempLabels; allLabelNames( labelTypes == labType(idx) )]; %#ok<AGROW>
                end
                labels = tempLabels;
                if isempty(labels)
                    error(message('vision:groundTruth:typeNotPresent', char(labType)))
                end
            end
            
            % Expect the list of labels provided to be unique
            labels = unique(labels,'stable');
        end
    end
end