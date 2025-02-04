%vision.labeler.mixin.BlockedImageAutomation Mixin for performing blocked image automation.
%   The BlockedImageAutomation mixin is a base class for AutomationAlgorithm objects that
%   define blockedImage properties needed for writing a blockedImage automation algorithm.
%
%   Note that the BlockedImageAutomation mixin can only be added to automation algorithms
%   to be used by the Image Labeler App.
%
%
%   Application Programming Interface Specification
%   -----------------------------------------------

%   See also vision.labeler.AutomationAlgorithm.

% Copyright 2020-2023 The MathWorks, Inc.

classdef (Abstract) BlockedImageAutomation < handle
    
    %----------------------------------------------------------------------
    % Use these properties to query information about the state of
    % algorithm execution.
    %----------------------------------------------------------------------
    % BlockedImageAutomation apply argument properties    
    properties
                          
        InclusionThreshold = 0     
        
        PadMethod = "replicate"
        
        PadPartialBlocks = false
        
        Resume = false
                
        BorderSize = 0
        
        BatchSize = 1
    
    end
    
    properties (Constant, Access = private)
        
        DisplayWaitbar = true
               
    end
       
    properties (Access = private)
        
        % Need a UI to set it
        BlockSize = [1024 1024]
        
        % This variable determines the region of Image to perform
        % automation in. It can either be the entire image, a freehand
        % region drawn on the image or just the present block being shown
        % in the app.If a freehand is drawn on figure to be used as mask,
        % this variable is updated automatically using that freehand ROI.
        BlockLocationSet
                        
        % It will be pulled from the Figure UI
        Level = 1
        
        UseParallel = false
        
        OutputLocation
        
        Adapter
        
        MaskSize = [8000 8000]
        
        WorldStart = [0.5 0.5]
        
        WorldEnd = [8000.5 8000.5]
                    
    end
     
    
    methods  (Hidden)
        % implementation is given to user
        function updateBlockLocationSet(this, bim, mode, ROIPositions)
            if ~isa(bim,'blockedImage')
                bim = blockedImage(bim);
            end
            
            switch mode
                case {'CurrentRegion','CustomRegion'}
                    % create a binary mask out of provided ROI regions to automate
                    % on
                    ROILabelIDs = ones(numel(ROIPositions),1);
                    
                    bmask = polyToBlockedImage(ROIPositions, ROILabelIDs, this.MaskSize, ...
                                              'WorldStart', this.WorldStart, ...
                                              'WorldEnd', this.WorldEnd);
                                  
                    % convert mask to blockLocationSet
                    this.BlockLocationSet = selectBlockLocations(bim, 'Levels', this.Level, ...
                        'Masks', bmask, ...
                        'InclusionThreshold', this.InclusionThreshold,...
                        'BlockSize', this.BlockSize);
                    
                case 'WholeImage'
                    this.BlockLocationSet = selectBlockLocations(bim, 'Levels', this.Level, ...                 
                        'BlockSize', this.BlockSize);
                    
                otherwise
                    error('Wrong Mode Setting');
                    
            end
       
        end
    end
    
     methods (Hidden)
        %------------------------------------------------------------------
        % Use this method to identify if the concrete automation algorithm
        % class inherits the blocked automation mixin.
        %------------------------------------------------------------------
        function tf = isBlockedAutomation(this)
            tf = isa(this, 'vision.labeler.mixin.BlockedImageAutomation');
        end
        
        function updateResolutionLevel(this, newValue)
            this.Level = newValue;
        end
        function updateBlockSizeRows(this, newValue)
            this.BlockSize(1) = newValue;
        end
        
        function updateBlockSizeColumns(this, newValue)
            this.BlockSize(2) = newValue;
        end
        
        function updateUseParallel(this, newState)
            if newState
                % Set the adapter that works with UseParallel
                this.Adapter = images.blocked.MATBlocks;
                
                this.UseParallel = true;
            else
                this.UseParallel = false;
            end
        end
           
        function updateWorldLimits(this, worldStart, worldEnd)
            this.WorldStart = worldStart;
            this.WorldEnd = worldEnd;
        end
        
        function updateMaskSize(this, maskSize)
            % only update when input maskSize is smaller than a coarse
            % default mask size of [8000 8000]
            if all(maskSize < 8000)
                this.MaskSize = maskSize;
            end
        end
        
        function level = getLevel(this)
            level = this.Level;
        end
        
        function blockSize = getBlockSize(this)
            blockSize = this.BlockSize;
        end
        function bls = getBlockLocationSet(this)
            bls = this.BlockLocationSet;
        end
                          
    end
    
    methods  (Abstract)
    %   RES = blockedImageAutomationAlgorithm(this, BSTRUCT)
    %   blockedImageAutomationAlgorithm is invoked on each image chosen for
    %   blockedImage automation in the ImageLabeler app. Use this method to
    %   execute the blockedImage Automation Algorithm to compute labels.
    %   Assign labels based on the algorithm in this method. This method
    %   takes BSTRUCT as an input and returns RES as the output.
    %
    %   Input
    %   -----  
    %   BSTRUCT is a struct with the following fields:
    %       Data      - A block of array data from BIM.            
    %       Start     - Array subscripts of the first element in the
    %                   block. If BorderSize is specified, this
    %                   subscript can be out-of-bounds for edge
    %                   blocks.
    %       End       - Array subscripts of the last element in the
    %                   block. If BorderSize is specified, this
    %                   subscript can be out-of-bounds for edge
    %                   blocks.
    %       Blocksub  - The block subscripts of the current block.
    %       BorderSize- The value of the BorderSize parameter.
    %       BlockSize - The value of the BlockSize parameter.
    %                   Note: size(data) can be less than this value
    %                   for border blocks when PadPartialValue is
    %                   false.
    %       BatchSize - The value of the BatchSize parameter.
    %
    %   Output
    %   ------
    %   The output format of RES depends on type of automation algorithm
    %   and is explained below.
    %
    %   Algorithms without pixel labels 
    %   -------------------------------
    %   For automation algorithms without pixel labels, RES
    %   must be a scalar struct with fields Type, Name,
    %   Position and optionally Attributes. The Attributes field exists
    %   only when labels with attributes have been defined.
    %
    %   The fields of the struct are described below:
    %
    %   Type        A <a href="matlab:help('labelType')">labelType</a> enumeration that defines the type of label.
    %               Type can have values Rectangle, Line, Projected
    %               cuboid, Cuboid or Scene.
    %
    %   Name        A character vector specifying a label name that
    %               returns true for checkLabelDefinition. Only
    %               existing label names previously defined in the
    %               labeler app can be used.
    %
    %   Position    Positions of the labels. The type of label determines
    %               the format of the position data.
    % 
    %                 |------------------------------------------------
    %                 | LABEL TYPE | DESCRIPTION
    %                 |------------|-----------------------------------
    %                 | Rectangle  | P-by-1 cell array specifying P Rectangles 
    %                 |            | each containing 1-by-4 vector specifying position 
    %                 |            | of bounding box locations as 
    %                 |            | [x y w h] OR Multiple Rectangle ROIs
    %                 |            | can be specified as an M-by-4
    %                 |            | matrix.
    %                 |------------|-----------------------------------
    %                 | Line       | P-by-1 cell array specifying P polylines  
    %                 |            | each containing N-by-2 vector specifying N points
    %                 |            | along each polyline as:
    %                 |            |
    %                 |            | [x1,y1; x2,y2;...xN,yN] 
    %                 |------------|-----------------------------------
    %                 | Projected  | P-by-1 cell array specifying P projected cuboids  
    %                 | Cuboid     | each containing 1-by-8 vector specifying position
    %                 |            | of primary and secondary faces as 
    %                 |            | [x1 y1 w1 h1 x2 y2 w2 h2] OR Multiple 
    %                 |            | Projected cuboid ROIs can be 
    %                 |            | specified as an M-by-8 matrix.        
    %   ---------------------------------------------------------------
    %
    %   Attributes  An array of structs representing the attributes
    %               contained by the automated labels. Each attribute
    %               is specified as a field of the struct, with the
    %               name of the field representing the name of the
    %               attribute and the value of the field representing
    %               the value of the attribute.
    %
    %   Algorithms with pixel labels
    %   ----------------------------
    %   For automation algorithms with pixel labels, RES must be
    %   a <a href="matlab:helpview('vision','categoricalLabelMatrix')">Categorical label matrix</a>, where each category represents a
    %   pixel label.
    %
    %   Note
    %   ----
    %   For automation algorithms without pixel labels, the field
    %   Position must be in World Coordinate system. This can easily be
    %   achieved by adding the bstruct field 'Start' X and Y indices to the
    %   output of automation algorithm.
    %
    %   To get the correct 'X' coordinate, add bstruct.Start(2) to get the
    %   position of the automation output in world coordinates.
    %
    %   To get the correct 'Y' coordinate, add bstruct.Start(1) to get the
    %   position of the automation output in world coordinates.
    %
    %   BRES is a blockedImage with classUnderlying value equal to the
    %   class of RES.
        res = blockedImageAutomationAlgorithm(this, bstruct)
    end
    
    methods (Access = protected)
        % A convenience function and let user call it in their run.
        % bres = blockedImageApply(I);
        function bres = blockedImageApply(this,I)
            if this.UseParallel
                % Instantiate OutputLocation
                this.OutputLocation = tempname;
                % Start a local pool if one does not exist already
                pool = gcp('nocreate');
                parallelApplyNV = {'OutputLocation',this.OutputLocation,...
                    'Adapter', this.Adapter};
            else
                parallelApplyNV = {};
            end
            
            % apply() for Current Region and Draw Regions
            if isempty(this.BlockLocationSet.BlockOrigin)
                error(getString( message('vision:imageLabeler:EmptyBlockLocationSetBlockedImageAutomation')));
            end
            
            bres = apply(I,@this.blockedImageAutomationAlgorithm, ...
                'BlockLocationSet', this.BlockLocationSet, ...
                'DisplayWaitbar', this.DisplayWaitbar, ...
                'PadMethod', this.PadMethod, ...
                'PadPartialBlocks', this.PadPartialBlocks, ...
                'Resume', this.Resume, ...
                'BorderSize', this.BorderSize, ...
                'BatchSize', this.BatchSize, ...
                'UseParallel', this.UseParallel, parallelApplyNV{:});
        end
        
    end
    
end
