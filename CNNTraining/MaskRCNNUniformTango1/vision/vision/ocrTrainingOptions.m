classdef ocrTrainingOptions

% Copyright 2022-2023 The MathWorks, Inc.
    
    properties

        OutputLocation (1,1) string {mustBeTextScalar, mustBeNonzeroLengthText, mustBeFolder, vision.internal.inputValidation.checkWritePermissions} = pwd
        
        MaxEpochs {iMustBeNumericPositiveInteger} = 5

        InitialLearnRate {iMustBeFloatPositiveFinite} = 10e-4

        Verbose (1,1) logical = true

        VerboseFrequency {iMustBeNumericPositiveInteger} = 50

        CheckpointPath (1,1) string {mustBeTextScalar, iValidateCheckpointPath} = ""

        CheckpointFrequency {iMustBeNumericPositiveInteger} = 1

        CheckpointFrequencyUnit (1,1) string = "epoch"

        ValidationData = []

        ValidationFrequency {iMustBeNumericPositiveInteger} = 50

        ValidationPatience {iMustBeNumericPositiveIntegerOrInfinite} = Inf

        OutputNetwork (1,1) string = "auto"

        SolverName (1,1) string = "adam"

        GradientDecayFactor {iValidateADAMOptimParams} = 0.5

        SquareGradientDecayFactor {iValidateADAMOptimParams} = 0.999

        Momentum {iMustBeFloatFinite, mustBeInRange(Momentum,0,1)} = 0.5
        
        Shuffle (1,1) string = "once"

        CharacterSetSource (1,1) string = "auto"
    end
    
    properties(Access = protected)
        Version = 1.0;
    end

    methods
        function obj = ocrTrainingOptions(options)
            arguments
                options.OutputLocation {mustBeTextScalar, mustBeNonzeroLengthText, mustBeFolder, vision.internal.inputValidation.checkWritePermissions} = pwd
                options.MaxEpochs {iMustBeNumericPositiveInteger} = 5
                options.InitialLearnRate {iMustBeFloatPositiveFinite} = 10e-4
                options.Verbose (1,1) logical = true
                options.VerboseFrequency {iMustBeNumericPositiveInteger} = 50
                options.CheckpointPath {mustBeTextScalar, iValidateCheckpointPath} = ""
                options.CheckpointFrequency {iMustBeNumericPositiveInteger} = 1
                options.CheckpointFrequencyUnit (1,1) string = "epoch"
                options.ValidationData = []
                options.ValidationFrequency {iMustBeNumericPositiveInteger} = 50
                options.ValidationPatience {iMustBeNumericPositiveIntegerOrInfinite} = Inf
                options.OutputNetwork (1,1) string = "auto"
                options.SolverName (1,1) string = "adam"
                options.GradientDecayFactor {iValidateADAMOptimParams} = 0.5
                options.SquareGradientDecayFactor {iValidateADAMOptimParams} = 0.999
                options.Momentum {iMustBeFloatFinite, mustBeInRange(options.Momentum,0,1)} = 0.5
                options.Shuffle (1,1) string = "once"
                options.CharacterSetSource (1,1) string = "auto"
            end
            
                obj.OutputLocation = options.OutputLocation;
                obj.MaxEpochs = options.MaxEpochs;
                obj.InitialLearnRate = options.InitialLearnRate;
                obj.Verbose = options.Verbose;
                obj.VerboseFrequency = options.VerboseFrequency;
                obj.CheckpointPath = options.CheckpointPath;
                obj.CheckpointFrequency = options.CheckpointFrequency;
                obj.CheckpointFrequencyUnit = options.CheckpointFrequencyUnit;
                obj.ValidationData = options.ValidationData;
                obj.ValidationFrequency = options.ValidationFrequency;
                obj.ValidationPatience = options.ValidationPatience;
                obj.OutputNetwork = options.OutputNetwork;
                obj.SolverName = options.SolverName;
                obj.GradientDecayFactor = options.GradientDecayFactor;
                obj.SquareGradientDecayFactor = options.SquareGradientDecayFactor;
                obj.Momentum = options.Momentum;
                obj.Shuffle = options.Shuffle;
                obj.CharacterSetSource = options.CharacterSetSource;
        end
    end

    methods(Static)
        %------------------------------------------------------------------
        % load object
        %------------------------------------------------------------------
        function this = loadobj(that)

            this = ocrTrainingOptions(OutputLocation = that.OutputLocation, ...
                                      MaxEpochs = that.MaxEpochs, ...
                                      InitialLearnRate = that.InitialLearnRate, ...
                                      Verbose = that.Verbose, ...
                                      VerboseFrequency = that.VerboseFrequency, ...
                                      CheckpointPath = that.CheckpointPath, ...
                                      CheckpointFrequency = that.CheckpointFrequency, ...
                                      CheckpointFrequencyUnit = that.CheckpointFrequencyUnit,...
                                      ValidationData = that.ValidationData, ...
                                      ValidationFrequency = that.ValidationFrequency, ...
                                      ValidationPatience = that.ValidationPatience, ...
                                      OutputNetwork = that.OutputNetwork, ...
                                      SolverName = that.SolverName, ...
                                      GradientDecayFactor = that.GradientDecayFactor, ...
                                      SquareGradientDecayFactor = that.SquareGradientDecayFactor, ...
                                      Momentum = that.Momentum, ...
                                      Shuffle = that.Shuffle, ...
                                      CharacterSetSource=that.CharacterSetSource);
            this.Version = that.Version;
        end
    end
    
    methods
        %------------------------------------------------------------------
        % Save object.
        %------------------------------------------------------------------
        function that = saveobj(this)
            that.OutputLocation = this.OutputLocation;
            that.MaxEpochs = this.MaxEpochs;
            that.InitialLearnRate = this.InitialLearnRate;
            that.Verbose = this.Verbose;
            that.VerboseFrequency = this.VerboseFrequency;
            that.CheckpointPath = this.CheckpointPath;
            that.CheckpointFrequency = this.CheckpointFrequency;
            that.CheckpointFrequencyUnit = this.CheckpointFrequencyUnit;
            that.ValidationData = this.ValidationData;
            that.ValidationFrequency = this.ValidationFrequency;
            that.ValidationPatience = this.ValidationPatience;
            that.OutputNetwork = this.OutputNetwork;
            that.SolverName = this.SolverName;
            that.GradientDecayFactor = this.GradientDecayFactor;
            that.SquareGradientDecayFactor = this.SquareGradientDecayFactor;
            that.Momentum = this.Momentum;
            that.Shuffle = this.Shuffle;
            that.Version = this.Version;
            that.CharacterSetSource = this.CharacterSetSource;
        end

        %------------------------------------------------------------------
        % Setters for numerical properties to cast double values.
        %------------------------------------------------------------------
        function this = set.MaxEpochs(this,newValue)
            this.MaxEpochs = double(newValue);
        end

        %------------------------------------------------------------------
        function this = set.InitialLearnRate(this,newValue)
            this.InitialLearnRate = double(newValue);
        end

        %------------------------------------------------------------------
        function this = set.VerboseFrequency(this,newValue)
            this.VerboseFrequency = double(newValue);
        end

        %------------------------------------------------------------------
        function this = set.CheckpointFrequency(this,newValue)
            this.CheckpointFrequency = double(newValue);
        end

        %------------------------------------------------------------------
        function this = set.ValidationData(this,newValue)
            
            if ~isempty(newValue)
                vision.internal.ocr.validateOCRDataStoreInput(newValue);
            end
            
            this.ValidationData = newValue;
        end

        %------------------------------------------------------------------
        function this = set.ValidationFrequency(this,newValue)
            this.ValidationFrequency = double(newValue);
        end

        %------------------------------------------------------------------
        function this = set.ValidationPatience(this,newValue)
            this.ValidationPatience = double(newValue);
        end

        %------------------------------------------------------------------
        function this = set.GradientDecayFactor(this,newValue)
            this.GradientDecayFactor = double(newValue);
        end

        %------------------------------------------------------------------
        function this = set.SquareGradientDecayFactor(this,newValue)
            this.SquareGradientDecayFactor = double(newValue);
        end

        %------------------------------------------------------------------
        function this = set.Momentum(this,newValue)
            this.Momentum = double(newValue);
        end

        %------------------------------------------------------------------
        % Setters for string properties.
        %------------------------------------------------------------------
        function this = set.CheckpointFrequencyUnit(this, newValue)
            this.CheckpointFrequencyUnit = validatestring(newValue, ...
                ["epoch","iteration"], mfilename, "CheckpointFrequencyUnit");
        end

        %------------------------------------------------------------------
        function this = set.OutputNetwork(this, newValue)
            this.OutputNetwork = validatestring(newValue, ...
                ["auto", "best-training-loss", "best-validation-loss", "last-iteration"], ...
                mfilename, "OutputNetwork");
        end

        %------------------------------------------------------------------
        function this = set.SolverName(this, newValue)
            this.SolverName = validatestring(newValue, ...
                ["adam", "sgdm"], mfilename, "SolverName");
        end

        %------------------------------------------------------------------
        function this = set.Shuffle(this, newValue)
            this.Shuffle = validatestring(newValue, ...
                ["once", "never"], mfilename, "Shuffle");
        end

        %------------------------------------------------------------------
        function this = set.CharacterSetSource(this, newValue)
            this.CharacterSetSource = validatestring(newValue, ...
                ["auto", "ground-truth-data", "base-model"], mfilename, "CharacterSetSource");
        end
    end
end

%--------------------------------------------------------------------------
function iValidateADAMOptimParams(x)
    arguments
        x (1,1) {iMustBeFloatFinite, mustBeInRange(x,0,1,"exclude-upper")}
    end
end

%--------------------------------------------------------------------------
function iMustBeFloatFinite(x)
    arguments
        x (1,1) {mustBeFloat, mustBeFinite}
    end
end

%--------------------------------------------------------------------------
function iMustBeFloatPositiveFinite(x)
    arguments
        x (1,1) {mustBeFloat, mustBePositive, mustBeFinite}
    end
end


%--------------------------------------------------------------------------
function iMustBeNumericPositiveInteger(x)
    arguments
        x (1,1) {mustBeNumeric, mustBePositive, mustBeInteger}
    end
end

%--------------------------------------------------------------------------
function iMustBeNumericPositiveIntegerOrInfinite(x)
    arguments
        x (1,1) {mustBeNumeric, mustBePositive}
    end
    
    if ~isinf(x)
        validateattributes(x, {'numeric'}, {'integer'});
    end
end

%--------------------------------------------------------------------------
function iValidateCheckpointPath(x)
    
    % Support empty path which is provided as default value for CheckpointPath.
    if x ~= ""
        mustBeFolder(x)
        vision.internal.inputValidation.checkWritePermissions(x)
    end 
end