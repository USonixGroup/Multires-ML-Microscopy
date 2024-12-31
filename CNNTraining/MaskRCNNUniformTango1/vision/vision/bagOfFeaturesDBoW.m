classdef bagOfFeaturesDBoW < handle

%  Copyright 2024 MathWorks, Inc.

%#codegen
    properties(GetAccess = public, SetAccess = private)
        DepthLevel
        BranchingFactor
        Normalization
    end

    properties(Hidden, Access = public)
        TreeProperties
        SerializedBag = string.empty
        FileName = string.empty
    end

    properties(Access = private)
        BowObj
    end

    methods (Access = public)
        function this = bagOfFeaturesDBoW(varargin)
            narginchk(1,5);
            
            vararg1 = varargin{1};
            if coder.target('MATLAB')
                this.BowObj = vision.internal.BagOfVisualWordsDBoW();
            else
                this.BowObj = vision.internal.buildable.BagOfFeaturesDBoWBuildable();
            end

            % Create the bag using an imagedatastore object
            if isa(vararg1, "matlab.io.datastore.ImageDatastore")
                if coder.target('MATLAB')
                    [inputObj, params] = bagOfFeaturesDBoW.parseInputs(varargin{:});
                    bagOfFeaturesDBoW.setVocabularyParameters(this, params);
                    featureArray = extractDescriptorsFromImageDS(this, inputObj);
                    createBagFromFeatures(this.BowObj, featureArray, this.DepthLevel, this.BranchingFactor, this.Normalization);
                else
                    coder.internal.error("vision:bagOfFeaturesDBoW:codegenNotSupported");
                end

            % Create the bag using a binaryFeatures object
            elseif iscell(vararg1) && ~isempty(vararg1)
                if coder.target('MATLAB')
                    [inputObj, params] = bagOfFeaturesDBoW.parseInputs(varargin{:});
                    bagOfFeaturesDBoW.setVocabularyParameters(this, params);
                    featureArray = cell(1,numel(inputObj));
                    for i=1:numel(inputObj)
                        featureArray{i} = inputObj{i}.Features;
                    end
                    createBagFromFeatures(this.BowObj, featureArray, this.DepthLevel, this.BranchingFactor, this.Normalization);
                else
                    coder.internal.error("vision:bagOfFeaturesDBoW:codegenNotSupported");
                end
            % Load a saved vocabulary file using the filename
            elseif nargin == 1
                validateattributes(vararg1, {'string', 'char'}, ...
                                    {}, mfilename, 'filename');
                if (~isfile(vararg1))
                    coder.internal.error("vision:bagOfFeaturesDBoW:fileNotFound", vararg1);
                end
                this.FileName = vararg1;
                
                [this.DepthLevel, this.BranchingFactor, this.Normalization] = createBagFromFile(this.BowObj, convertStringsToChars(vararg1));
                this.TreeProperties = [this.DepthLevel, this.BranchingFactor];
                
            else
                % Error for an invalid input
                coder.internal.error("vision:bagOfFeaturesDBoW:invalidInputType");
            end
            this.SerializedBag = getSerializedBag(this.BowObj);
                        
        end
    end

    methods (Hidden, Access = public)
        function bag = getFileName(this)
           bag = this.FileName;
        end
    end

    methods(Hidden, Access = protected)

        function features = extractDescriptorsFromImageDS(this, imds)
            % Extracts descriptors from all images in an ImageDatastore.

            numImages = numel(imds.Files);
            features = cell(numImages,1);
            for i = 1:numImages
                img = imds.readimage(i); % read in an image from the set
                descriptors = extractDescriptorsFromImage(this,img);
                features{i} = descriptors.Features;
            end
        end

        function descriptors = extractDescriptorsFromImage(~,image)
            % Extracts descriptors from a single image.

            grayImage = im2gray(image);
            points = detectORBFeatures(grayImage);
            descriptors = extractFeatures(grayImage, points);
        end
    end

    methods (Static, Hidden)
        %------------------------------------------------------------------
        % Save object
        %------------------------------------------------------------------
        function that = saveobj(this)
            that.SerializedBag         = this.SerializedBag;
        end
    end

    methods (Static, Hidden)
        %------------------------------------------------------------------
        % Load object
        %------------------------------------------------------------------
        function this = loadobj(that)
            % Create object with dummy features
            this = bagOfFeaturesDBoW({binaryFeatures(uint8(1))});
            this.BowObj = vision.internal.BagOfVisualWordsDBoW();
            % Re-create internal bag using the serialized form
            [this.DepthLevel, this.BranchingFactor, this.Normalization] = createBagFromCharArray(this.BowObj, convertStringsToChars(that.SerializedBag));
            this.TreeProperties = [this.DepthLevel, this.BranchingFactor];
        end
    end

    methods (Hidden, Static, Access = protected)
        function d = getDefaultSettings
            % Static, protected method to get the default settings for the
            % object.
            d.TreeProperties = [5,10];
            d.Normalization = "L1";
            d.DepthLevel = d.TreeProperties(1);
            d.branchingFactor = d.TreeProperties(2);
        end

        function checkImageDatastore(imds, name)
            % Validating the datatype for an imageDatastore as the input

            varName = 'imds';
            validateattributes(imds,...
                               {'matlab.io.datastore.ImageDatastore'}, ...
                               {'nonempty'},...
                               name, varName);
        end

        function checkBinaryFeatureSet(featureSet, name)
            % Validating the datatype for binaryFeatures as the input

            varName = 'features';
            validateattributes(featureSet,...
                               {'binaryFeatures'}, ...
                               {'nonempty'},...
                               name, varName);
        end

        function checkVocabFile(vocabFileName, name)
            % Validating the datatype for a file input

            varName = 'vocabularyFileName';
            validateattributes(vocabFileName,...
                               {'char','string'}, ...
                               {'vector','nonempty'},...
                               name, varName);
        end

        function checkTreeProperties(props)
            % Validating Tree Properties

            validateattributes(props,{'numeric'},...
                               {'vector','nonsparse','real','nonempty','integer','finite','numel',2},...
                               'bagOfFeaturesDBoW', 'TreeProperties');

            % The number of levels must be >= 1.
            numLevels = props(1);
            validateattributes(numLevels,{'numeric','integer','scalar'}, {'>=' 1}, ...
                               'bagOfFeaturesDBoW', 'TreeProperties(1)');

            % The branching factor must be >= 2.
            branchingFactor = props(2);
            validateattributes(branchingFactor,{'numeric','integer','scalar'}, {'>=' 2}, ...
                               'bagOfFeaturesDBoW', 'TreeProperties(2)');
        end

        function checkInputObj(inObj, name)
            % Input argument validation

            if isa(inObj, 'matlab.io.datastore.ImageDatastore')
                bagOfFeaturesDBoW.checkImageDatastore(inObj,name)
            elseif iscell(inObj) && ~isempty(inObj)
                for i=1:numel(inObj)
                    bagOfFeaturesDBoW.checkBinaryFeatureSet(inObj{i},name)
                end
            else
                bagOfFeaturesDBoW.checkVocabFile(inObj,name)
            end
        end

        function [inputObj,params] = parseInputs(varargin)
            % Parsing the inputs to validate the allowed types

            d = bagOfFeaturesDBoW.getDefaultSettings;
            parser = inputParser;
            addRequired(parser, 'inputObj', @(x)bagOfFeaturesDBoW.checkInputObj(x,mfilename));
            addParameter(parser, 'TreeProperties', d.TreeProperties, @(x) bagOfFeaturesDBoW.checkTreeProperties(x));
            addParameter(parser, 'Normalization', d.Normalization, @(x) any(validatestring(x,{'L1', 'L2'})));
            parse(parser, varargin{:});

            inputObj = parser.Results.inputObj;
            params.TreeProperties = double(parser.Results.TreeProperties);
            params.Normalization = char(parser.Results.Normalization);
        end

        function setVocabularyParameters(this, params)
            % Setter for the object properties

            this.TreeProperties = params.TreeProperties;
            this.Normalization = params.Normalization;
            this.DepthLevel = params.TreeProperties(1);
            this.BranchingFactor = params.TreeProperties(2);
        end
    end
end