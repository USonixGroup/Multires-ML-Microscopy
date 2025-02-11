classdef InvalidGroundTruthDataSource < vision.internal.EnforceScalarHandle
    % InvalidGroundTruthDataSource Class representing invalid
    % ground truth data sources. Used when data sources are not found while
    % loading groundTruth objects.
    
    % Copyright 2018 The MathWorks, Inc.
    
    properties(SetAccess = private)
        % Source Holds invalid data source as cellstr, char array, or [].
        Source
    end
    
    methods
        
        %------------------------------------------------------------------
        function this = InvalidGroundTruthDataSource(src)
            this.Source = src;
        end
             
        %------------------------------------------------------------------
        function s = serialize(this)
            % Serialize invalid data sources by returning the invalid data
            % source. This is called directly from groundTruth.saveobj and
            % is not intended in normal save/load workflows.
            s = this.Source;
        end
        
    end
    
    methods(Access = private)
        % This object is not allowed to be saved. Serialization of
        % groundTruth with invalid data sources is handled in
        % groundTruth.saveobj().
        saveobj(~)
    end
end