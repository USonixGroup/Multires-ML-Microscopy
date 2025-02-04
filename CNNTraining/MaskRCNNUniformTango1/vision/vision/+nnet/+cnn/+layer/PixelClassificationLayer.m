classdef PixelClassificationLayer < nnet.cnn.layer.Layer & nnet.internal.cnn.layer.Externalizable
    % PixelClassificationLayer  Pixel classification output layer.
    % 
    %   A pixel classification layer is used as the output layer for a network
    %   that performs semantic image segmentation. Use pixelClassificationLayer
    %   to create this layer.
    %
    %   PixelClassificationLayer properties:
    %       Name          - A name for the layer.
    %       Classes       - The categories into which the pixels are 
    %                       classified.
    %       ClassWeights  - The weight assigned to each class.
    %       OutputSize    - The size of the output.
    %       LossFunction  - The loss function that is used for training.
    %
    % Example
    % -------
    % % Create a output layer.
    % layer = pixelClassificationLayer()
    %
    % See also semanticseg, pixelLabelImageGenerator, pixelLabelDatastore,
    %          pixelLabelImageGenerator/countEachLabel, trainNetwork.

    % Copyright 2017-2024 The MathWorks, Inc.

    properties(Dependent)
        % Name   A name for the layer
        %   The name for the layer. If this is set to '', then a name will
        %   be automatically set at training time.
        Name
        
        % Classes (categorical)  The categories into which the pixels are
        % classified.
        %   A categorical column vector whose elements are the distinct
        %   classes to classify the input image to the layer. It can be
        %   set passing a string or categorical vector, a cell vector of
        %   character vectors, or 'auto'. When 'auto' is specified, the
        %   classes are automatically set during training. Default: 'auto'.
        Classes
        
        % ClassWeights   The weight associated with each class
        %   Class weights are stored as a vector W. The weight W(k)
        %   corresponds to k-th category in Classes. Use class weighting to
        %   balance classes when there are underrepresented classes in the
        %   training data. If there are no class weights, ClassWeights is
        %   'none'.
        ClassWeights
    end
    
    properties(SetAccess = private, Dependent, Hidden)
        % ClassNames   The names of the classes
        %   A cell array containing the names of the classes. This will be
        %   automatically determined at training time. Prior to training,
        %   it will be empty.
        ClassNames
    end
    
    properties(SetAccess = private, Dependent)
        % OutputSize   The size of the output
        %   The size of the output. This will be determined at training
        %   time. Prior to training, it is set to 'auto'.
        OutputSize
    end
    
    properties(SetAccess = private)
        % LossFunction   The loss function for training
        %   The loss function that will be used during training. Possible
        %   values are:
        %       'crossentropyex'    - Cross-entropy for exclusive outputs.
        LossFunction = 'crossentropyex';
    end
    
    methods
        function val = get.OutputSize(this)
            if(isempty(this.PrivateLayer.OutputSize))
                val = 'auto';
            else
                val = this.PrivateLayer.OutputSize;
            end
        end
        
        function val = get.ClassNames(this)
            val = this.PrivateLayer.ClassNames(:);
        end
        
        function val = get.Classes(this)
            if isempty(this.PrivateLayer.Categories)
                val = 'auto';
            else
                val = this.PrivateLayer.Categories(:);
            end
        end
        
        function this = set.Classes(this, val)
            iAssertValidClasses(val);
            classes = iConvertClassesToCanonicalForm(val);
            try 
                iCheckConsistencyClassesAndClassWeights(classes,...
                    this.PrivateLayer.ClassWeights);            
            catch exception
                iAddSolutionAndRethrow(exception)
            end
            this.PrivateLayer.Categories = classes;
        end
        
        function w = get.ClassWeights(this)
            if isempty(this.PrivateLayer.ClassWeights)
                w = 'none';
            else
                w = this.PrivateLayer.ClassWeights;
            end
        end
        
        function this = set.ClassWeights(this, val)
            w = iCheckAndFormatClassWeights(val, 'PixelClassificationLayer');
            iCheckConsistencyClassesAndClassWeights(...
                this.PrivateLayer.Categories, w);            
            this = this.updateClassWeights(w);
        end
        
        function val = get.Name(this)
            val = this.PrivateLayer.Name;
        end
        
        function this = set.Name(this, val)
            iAssertValidLayerName(val);
            this.PrivateLayer.Name = char(val);
        end  
    end
    
    methods
        function this = PixelClassificationLayer(privateLayer)
            this.PrivateLayer = privateLayer;
        end
        
        function out = saveobj(this)
            
            privateLayer = this.PrivateLayer;
            out.Version = 3.0;
            out.Name = privateLayer.Name;
            out.OutputSize   = privateLayer.OutputSize;
            out.Categories   = privateLayer.Categories;
            out.ClassWeights = privateLayer.ClassWeights;
            out.ChannelDim = privateLayer.ChannelDim;
        end
    end
    
    methods(Access = private)
        function this = updateClassWeights(this, w)
            name = this.PrivateLayer.Name;
            categories = this.PrivateLayer.Categories;
            outputSize = this.PrivateLayer.OutputSize;            
            this.PrivateLayer = nnet.internal.cnn.layer.SpatialCrossEntropy(...
                name, categories, w, outputSize);
        end
    end
    
    methods(Hidden, Static)
        %------------------------------------------------------------------
        function this = loadobj(in)
            if in.Version <= 1
                in = iUpgradeFromVersionOneToVersionTwo(in);
                in = iUpgradeVersionTwoToVersionThree(in);
            elseif in.Version == 2
                in = iUpgradeVersionTwoToVersionThree(in);
            end            
            this = iLoadPixelClassificationLayerFromCurrentVersion(in);            
        end
    end
    
    methods(Hidden, Access = protected)
        %------------------------------------------------------------------
        function [description, type] = getOneLineDisplay(this)
            
            numClasses = numel(this.ClassNames);
            
            if numClasses==0
                classString = '';
                
            elseif numClasses==1
                classString = getString(message('vision:semanticseg:oneLineDisplayOneClass', this.ClassNames{1}));
                
            elseif numClasses==2
                classString = getString(message(...
                    'vision:semanticseg:oneLineDisplayTwoClasses',...
                    this.ClassNames{1},...
                    this.ClassNames{2}));
                
            elseif numClasses>=3
                classString = getString(message(...
                    'vision:semanticseg:oneLineDisplayNClasses',...
                    this.ClassNames{1},...
                    this.ClassNames{2},...
                    int2str(numClasses-2)));
            end
            
            if strcmp(this.ClassWeights, 'none')
                description = getString(message(...
                    'vision:semanticseg:oneLineDisplay', classString));
            else
                description = getString(message(...
                    'vision:semanticseg:oneLineDisplayWeighted', classString));
            end
            
            type = getString(message('vision:semanticseg:OneLineDispName'));
        end
        
        function groups = getPropertyGroups( this )
            if numel(this.Classes) < 11 && ~ischar(this.Classes)
                propertyList = struct;
                propertyList.Name = this.Name;
                propertyList.Classes = this.Classes';
                propertyList.ClassWeights = this.ClassWeights;
                propertyList.OutputSize = this.OutputSize;
                groups = [
                    matlab.mixin.util.PropertyGroup(propertyList, '');
                    this.propertyGroupHyperparameters( {'LossFunction'} )
                    ];
            else
                generalParameters = {'Name' 'Classes' 'ClassWeights' 'OutputSize'};
                groups = [
                    this.propertyGroupGeneral( generalParameters )
                    this.propertyGroupHyperparameters( {'LossFunction'} )
                    ];
            end
        end
        
    end
    
    methods(Hidden, Static)
        %------------------------------------------------------------------
        function params = parseInputs(varargin)
            p = inputParser();
            
            p.addParameter('Name', '', @nnet.internal.cnn.layer.paramvalidation.validateLayerName);
            p.addParameter('ClassNames', 'auto');
            p.addParameter('ClassWeights', 'none');
            p.addParameter('Classes', 'auto', @iAssertValidClasses);
            
            p.parse(varargin{:});
            
            userInput = p.Results;

            names   = iCheckAndFormatClassNames(userInput.ClassNames);
            classes = iConvertClassesToCanonicalForm(userInput.Classes);            
            weights = iCheckAndFormatClassWeights(userInput.ClassWeights,...
                'pixelClassificationLayer');
            
            if ~isempty(names)
                if ~isempty(classes)
                    error(message('vision:semanticseg:ClassesAndClassNamesNVP'))
                else
                    iCheckConsistencyClassNamesAndClassWeights(names, ...
                        weights);
                    classes = categorical(names,names);
                end
            else
                 iCheckConsistencyClassesAndClassWeights(classes, weights);
            end
            
            params.Name            = char(userInput.Name);
            params.Categories      = classes;
            params.ClassWeights    = weights;            
        end
    end
end

%--------------------------------------------------------------------------
function names = iCheckAndFormatClassNames(names)
    if ~(isvector(names) && (ischar(names) || iscellstr(names) || isstring(names)))        
        error(message('vision:semanticseg:InvalidClassNames'));
    end          
    
    names = string(names);        
    isAuto = numel(names)== 1 && names == "auto";
    
    if isAuto
        names = cell(0,1);
    else
        if numel(unique(names)) ~= numel(names)
            error(message('vision:semanticseg:NonUniqueClassNames'));
        end
        % return names as cellstr
        names = reshape(cellstr(names),[],1);
    end
    
end

%--------------------------------------------------------------------------
function w = iCheckAndFormatClassWeights(w, functionName)
    if ischar(w) || isstring(w)
        
        validatestring(w, {'none'}, functionName, 'ClassWeights');
        w = [];
    else
        validateattributes(w, {'numeric'}, ...
            {'vector', 'positive', 'finite', 'real', 'nonempty', 'nonsparse'}, ...
            functionName, 'ClassWeights');
        
        % Gather weights to CPU to prevent loss computation on the GPU.
        w = gather(w);
        
        w = reshape(double(w),[],1);
        
    end
end

%--------------------------------------------------------------------------
function iAssertValidLayerName(name)
iEvalAndThrow(@()...
    nnet.internal.cnn.layer.paramvalidation.validateLayerName(name));
end

%--------------------------------------------------------------------------
function S = iUpgradeFromVersionOneToVersionTwo( S )
% iUpgradeFromVersionOneToVersionTwo     In all layers with version <= 1
% the property ClassNames which has to be replaced with Categories as in 
% version 2. See g1667032. Here set Categories to categorical array with 
% categories as ClassNames and ordinality false.
S.Version = 2.0;
S.Categories = categorical(S.ClassNames, S.ClassNames);
end

%--------------------------------------------------------------------------
function S = iUpgradeVersionTwoToVersionThree(S)
% iUpgradeVersionTwoToVersionThree   Upgrade a v2 (2017b-2018b) saved struct
% to a v3 saved struct. This means adding a "ChannelDim" property.

S.Version = 3;
% In releases prior to 19a, only 4-D data was supported.
S.ChannelDim = 3;
end

%--------------------------------------------------------------------------
function layer = iLoadPixelClassificationLayerFromCurrentVersion(in)
% Create internal layer. Gather class weights in case they were saved as
% gpuArray objects. 
internalLayer = nnet.internal.cnn.layer.SpatialCrossEntropy(...
    in.Name, in.Categories, gather(in.ClassWeights), in.OutputSize);
internalLayer.ChannelDim = in.ChannelDim;
layer = nnet.cnn.layer.PixelClassificationLayer(internalLayer);
end

%--------------------------------------------------------------------------
function iCheckConsistencyClassesAndClassWeights(classes, weights)
if ~isempty(weights)
    if isempty(classes)
        error(message('vision:semanticseg:ClassesRequired'))
    elseif numel(weights) ~= numel(classes)
        error(message('vision:semanticseg:ClassesWeightsMismatch'))
    end
end
end

%--------------------------------------------------------------------------
function iCheckConsistencyClassNamesAndClassWeights(names, weights)
if ~isempty(weights) && numel(weights) ~= numel(names)
    error(message('vision:semanticseg:ClassNamesWeightsMismatch'))
end
end

%--------------------------------------------------------------------------
function iAssertValidClasses(value)
iEvalAndThrow(@()...
    nnet.internal.cnn.layer.paramvalidation.validateClasses(value)); 
end

%--------------------------------------------------------------------------
function tf = iIsAuto(val)
tf = isequal(string(val), "auto");
end

%--------------------------------------------------------------------------
function classes = iConvertClassesToCanonicalForm(classes)
if iIsAuto(classes)
    classes = categorical();
else
    classes = ...
        nnet.internal.cnn.layer.paramvalidation...
            .convertClassesToCanonicalForm(classes);
end
end

%--------------------------------------------------------------------------
function iAddSolutionAndRethrow(exception)
solutionId = "SolutionToModifyClasses";
newId = string(exception.identifier)+solutionId;
solutionMsg = string(message("vision:semanticseg:"+solutionId));
newMsg = string(exception.message)+" "+solutionMsg; 
throwAsCaller(MException(newId, newMsg))
end

%--------------------------------------------------------------------------
function iEvalAndThrow(func)
% Omit the stack containing internal functions by throwing as caller
try
    func();
catch exception
    throwAsCaller(exception)
end
end
