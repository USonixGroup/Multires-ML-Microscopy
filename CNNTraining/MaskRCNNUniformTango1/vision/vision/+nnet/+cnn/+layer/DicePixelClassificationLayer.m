classdef DicePixelClassificationLayer < nnet.cnn.layer.Layer & nnet.internal.cnn.layer.Externalizable
    % DicePixelClassificationLayer  Dice Pixel classification output layer.
    % 
    %   A dice pixel classification layer is used as the output layer for a
    %   network that performs semantic image segmentation using generalized
    %   dice loss. Use dicePixelClassificationLayer to create this layer.
    %
    %   DicePixelClassificationLayer properties:
    %       Name          - A name for the layer.
    %       Classes       - The categories into which the pixels are
    %                       classified.
    %       OutputSize    - The size of the output.
    %       LossFunction  - The loss function that is used for training.
    %
    % Example
    % -------
    % % Create a output layer.
    % layer = dicePixelClassificationLayer()
    %
    % See also semanticseg, pixelLabelDatastore, trainNetwork.

    % Copyright 2019-2024 The MathWorks, Inc.

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
        %     'generalizedDiceLoss'
        LossFunction = 'generalizedDiceLoss';
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
            this.PrivateLayer.Categories = classes;
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
        function this = DicePixelClassificationLayer(privateLayer)
            this.PrivateLayer = privateLayer;
        end
        
        function out = saveobj(this)        
            privateLayer = this.PrivateLayer;
            out.Version = 1.0;
            out.Name = privateLayer.Name;
            out.OutputSize = privateLayer.OutputSize;
            out.Categories   = privateLayer.Categories;
            out.ChannelDim = privateLayer.ChannelDim;
        end
    end
     
    methods(Hidden, Static)
        %------------------------------------------------------------------
        function this = loadobj(in)            
            internalLayer = nnet.internal.cnn.layer.GeneralizedDiceLoss(...
                in.Name, in.Categories, in.OutputSize, in.ChannelDim);
            this = nnet.cnn.layer.DicePixelClassificationLayer(internalLayer);
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
            
            description = getString(message(...
                    'vision:semanticseg:diceOneLineDisplay', classString));
                        
            type = getString(message('vision:semanticseg:diceOneLineDispName'));
        end
        
        function groups = getPropertyGroups( this )
            if numel(this.Classes) < 11 && ~ischar(this.Classes)
                propertyList = struct;
                propertyList.Name = this.Name;
                propertyList.Classes = this.Classes';
                propertyList.OutputSize = this.OutputSize;
                groups = [
                    matlab.mixin.util.PropertyGroup(propertyList, '');
                    this.propertyGroupHyperparameters( {'LossFunction'} )
                    ];
            else
                generalParameters = {'Name' 'Classes' 'OutputSize'};
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
            p.addParameter('Classes', 'auto', @iAssertValidClasses);
            p.parse(varargin{:});
            
            userInput = p.Results;
            classes = iConvertClassesToCanonicalForm(userInput.Classes);                       
            
            params.Name = char(userInput.Name);
            params.Categories = classes;
        end
    end
end

%--------------------------------------------------------------------------
function iAssertValidLayerName(name)
iEvalAndThrow(@()...
    nnet.internal.cnn.layer.paramvalidation.validateLayerName(name));
end

%--------------------------------------------------------------------------
function iAssertValidClasses(value)
iEvalAndThrow(@()...
    nnet.internal.cnn.layer.paramvalidation.validateClasses(value));
if ~iIsAuto(value)
    names = string(value);
    % There should be 2 or more classes
    if numel(names)<2
        error(message('vision:semanticseg:MultiClass'));
    end
end    
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
function iEvalAndThrow(func)
% Omit the stack containing internal functions by throwing as caller
try
    func();
catch exception
    throwAsCaller(exception)
end
end
