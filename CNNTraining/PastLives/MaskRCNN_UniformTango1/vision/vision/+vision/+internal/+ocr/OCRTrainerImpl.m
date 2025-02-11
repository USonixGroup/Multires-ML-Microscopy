classdef OCRTrainerImpl < handle
%

%   Copyright 2022-2023 The MathWorks, Inc.
    

    properties (Access = private)
        
        % Inputs to the constructor.
        TrainingDs
        ModelName
        BaseModel
        Options

        % Validation data.
        ValidationDs
        
        % Temporary directories.
        TemporaryDir
        ModelDir
        TrainingGTruthDir
        ValidationGTruthDir
        BaseCompsDir
        InternalCheckpointDir
        
        % Temporary files.
        TrainingBoxFiles % Cell array of box file names used for training.
        TrainingLstmfFiles % Cell array of lstmf file names used for training.
        ValidationLstmfFiles % Cell array of lstmffile names used for validation.
        TrainingLSTMFilesList % File name of the file containing the list of training lstmf files.
        ValidationLSTMFilesList % File name of the file  containing the list of validation lstmf files.
        GTruthUnicharsetFile % Unicharset file generated from ground truth data.
        StartModelUnicharsetFile % Unicharset file used to generate start model.
        StartModel % A starter model to begin the training with.
        
        NumTrain % Number of training samples.
        
        TrainerObj % A vision.internal.OCRTrainer object.
        
        StartTime % Start time of training process.

        InternalCheckpoint % Internal checkpoint used to create final model.

        TessDataPrefix % TESSDATA_PREFIX environment variable value at the start of training.

        IsResumeWorkflow % Flag to determine if base model is checkpoint.

        % File name prefixes.
        InternalCheckpointPrefix
        BaseCompsPathWithPrefix

        % Buffers to log training progress.        
        InfoStruct
        IterationLog
        
        % Variables to track training loop progress.
        CurrentIteration
        CurrentEpoch
        HasTrainingConverged
        BestValidationLoss
        BestTrainingLoss
        ValidationPatienceCount
    end

    %----------------------------------------------------------------------
    %  Public APIs
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        %  Step 0: Constructor
        %------------------------------------------------------------------
        function this = OCRTrainerImpl(trainingData, modelName, baseModel, options)

            % Log start time.
            this.StartTime   =  cputime;

            % Store inputs as properties for later use.
            this.TrainingDs  =  copy(trainingData);
            if ~isempty(options.ValidationData)
                this.ValidationDs =  copy(options.ValidationData);
            end
            
            this.BaseModel = baseModel;
            this.ModelName = char(modelName);
            this.Options = options;
            
            % Clear the TESSDATA_PREFIX environment variable to make sure that ocr uses
            % tessdata files specified using filepaths.
            this.TessDataPrefix = iUnsetTessDataPrefix();
            
            % Constructor the internal trainer object.
            this.TrainerObj = vision.internal.OCRTrainer();

            % Initialize training progress log buffers.
            this.InfoStruct = struct('TrainingRMSE'   , [], 'TrainingCharacterError'  , [], 'TrainingWordError'   , [],  ...
                                     'ValidationRMSE' , [], 'ValidationCharacterError', [], 'ValidationWordError', [],  ...
                                     'BaseLearnRate'  , [], 'OutputModelIteration'    ,  0, 'FinalValidationRMSE', 0);

            this.IterationLog = struct('Epoch', 0, 'Iteration', 0, 'Time', 0, 'BaseLearnRate', 0, ...
                                       'TrainingRMSE'  , 0, 'TrainingCharacterError'  , 0, 'TrainingWordError' , 0, ...
                                       'ValidationRMSE', 0, 'ValidationCharacterError', 0, 'ValidationWordError', 0);

            % Initialize best losses to Inf.
            this.BestTrainingLoss = Inf;
            this.BestValidationLoss = Inf;
            
            % Check if a checkpoint is used for training.
            this.IsResumeWorkflow = contains(baseModel, '.checkpoint.traineddata');

            % Create temporary directories to use for training.
            this.createTemporaryDirectories();

            if this.IsResumeWorkflow
                % Rename the checkpoint file to have .traineddata extension
                % instead of .checkpoint.traineddata.
                this.BaseModel = backupCheckpoint(this.BaseModel, this.BaseCompsDir);
            end

            this.defineAutoBehavior();
        end

        %------------------------------------------------------------------
        % Step 1: Generate LSTMF files.
        %------------------------------------------------------------------
        function generateLSTMFFiles(this, printer)

            % Generate training LSTMF files.
            [this.TrainingLstmfFiles, this.TrainingBoxFiles] = ...
                generateLSTMFFilesImpl(this.TrainingDs, this.ModelName, ...
                this.TrainingGTruthDir, printer, 'vision:trainOCR:preparingTraining');
    
            if ~isempty(this.ValidationDs)
                % Generate validation LSTMF files if validation data is specified.
                this.ValidationLstmfFiles = ...
                    generateLSTMFFilesImpl(this.ValidationDs, this.ModelName, ...
                    this.ValidationGTruthDir, printer, 'vision:trainOCR:preparingValidation');
            end

            % Write LSTMF files onto text files.
            this.writeLstmfFileLists();
        end

        %------------------------------------------------------------------
        % Step 2: Extract tesseract components from the base model file.
        %------------------------------------------------------------------
        function extractBaseModelComponents(this)
            
            % Construct a base file name for tesseract components.
            this.BaseCompsPathWithPrefix = fullfile(this.BaseCompsDir, this.ModelName);

            % Extract tesseract components.
            tesseractExtractTessdata(this.BaseModel, this.BaseCompsPathWithPrefix);
        end

        %------------------------------------------------------------------
        % Step 3: Create unicharset file based on character set source.
        %------------------------------------------------------------------
        function createUnicharset(this)

            % Extract unicharset from ground truth.
            this.extractUnicharsetFromGTruth();

            % Select unicharset based on character set source.
            baseModelUnicharsetFile = strcat(this.BaseCompsPathWithPrefix,'.lstm-unicharset');
            switch this.Options.CharacterSetSource
                case "ground-truth-data"
                    unicharsetFile   = this.GTruthUnicharsetFile;
                case "base-model"
                    unicharsetFile   = baseModelUnicharsetFile;
            end

            % Copy the selected unicharset file to model directory.
            this.StartModelUnicharsetFile = fullfile(this.ModelDir, 'unicharset');
            copyfile(unicharsetFile, this.StartModelUnicharsetFile);

            % Verify successful file creation.
            iCheckIfFilesExist(this.StartModelUnicharsetFile);
        end

        %------------------------------------------------------------------
        % Step 4: Create a starter traineddata model file.
        %------------------------------------------------------------------
        function starterModelCharacterSet = createStarterModelFile(this)
        
            inputUnicharset = this.StartModelUnicharsetFile;

            % Define model properties.
            modelProperties.StrokeEncodingDir = ...
                fullfile(toolboxdir('vision'),'visionutilities','tessdata_best'); 
            modelProperties.ModelName = this.ModelName;
            modelProperties.IsLanguageRTL = false;
            modelProperties.DoPassThroughRecoder = false;
            modelProperties.Version = 'DummyVersionText';

            % Create starter model file.
            tesseractCreateStarterModelFile(inputUnicharset, modelProperties, this.TemporaryDir);
            
            % Store model file path.
            modelDir = fullfile(this.TemporaryDir, this.ModelName);
            this.StartModel = fullfile(modelDir, strcat(this.ModelName, '.traineddata'));
            
            % Verify successful file creation.
            iCheckIfFilesExist(this.StartModel);

            % Return character set used in the start model.
            starterModelCharacterSet = tesseractReadTraineddata(char(this.StartModel));

            % Sort character set alphabetically based on ASCII values.
            starterModelCharacterSet = sort(starterModelCharacterSet);
        end

        %------------------------------------------------------------------
        % Step 5: Perform training.
        %------------------------------------------------------------------
        function convergenceReason = runTraining(this)
            
            % Define parameters for training.
            params.ContinueFrom = strcat(this.BaseCompsPathWithPrefix, '.lstm');
            params.StartModel = this.StartModel;
            params.BaseModel  = this.BaseModel;

            params.TrainListfile = char(this.TrainingLSTMFilesList);
            params.ValidationListfile = char(this.ValidationLSTMFilesList);
            
            params.ModelOutput = char(fullfile(this.Options.CheckpointPath, this.ModelName));
            
            params.LearningRate  = this.Options.InitialLearnRate;
            
            if this.Options.SolverName == "adam"
                params.NetMode = 192;
                params.Momentum = this.Options.GradientDecayFactor;
                params.AdamBeta = this.Options.SquareGradientDecayFactor;
            elseif this.Options.SolverName == "sgdm"
                params.NetMode = 64;
                params.Momentum = this.Options.Momentum;
            end

            % Unexposed constant parameters.
            params.DebugInterval = -1;
            params.MaxImageMB = 200;
            params.SequentialTraining = false;
            params.RandomlyRotate = false;

            % Initialize training with the parameters.
            this.TrainerObj.initialize(params);

            % Run training.
            convergenceReason = this.trainingLoop();
        end

        %------------------------------------------------------------------
        % Step 6: Generate final model file.
        %------------------------------------------------------------------
        function [finalModel, info] = generateFinalModelFile(this)
    
            % Get the iteration number from the internal checkpoing file name.
            internalCheckpointBaseName = this.ModelName + "-" + this.InternalCheckpointPrefix;
            temp = strsplit(this.InternalCheckpoint, internalCheckpointBaseName);
            temp = strsplit(temp{2}, "_");
            iteration = str2double(temp{2});
            this.InfoStruct.OutputModelIteration = iteration;
            
            % Populate validation rmse corresponding to the output model iteration.
            if ~isempty(this.ValidationDs)
                this.InfoStruct.FinalValidationRMSE = this.InfoStruct.ValidationRMSE(iteration);
            end

            % Get name of the final model.
            finalModel = fullfile(this.Options.OutputLocation, strcat(this.ModelName, '.traineddata'));

            % Copy checkpoint to the output location with the final model name.
            [status, msg] = copyfile(this.InternalCheckpoint, finalModel);
            if ~status
                error(message("vision:trainOCR:finalModelCopyFileFailure", msg))
            end

            % Verify successful file creation.
            iCheckIfFilesExist(finalModel);

            % Return info struct.
            info = this.InfoStruct;
        end

        %------------------------------------------------------------------
        % Clean up training artifacts.
        %------------------------------------------------------------------
        function cleanUp(this)
            if ~isempty(this.TessDataPrefix)
                setenv('TESSDATA_PREFIX',this.TessDataPrefix);
            end

            rmdir(this.TemporaryDir,'s');
        end
    end

    %----------------------------------------------------------------------
    %  Helpers related to steps 0-3.
    %----------------------------------------------------------------------
    methods (Access = private)
        
        %------------------------------------------------------------------
        % Helper to create the following temporary directories to use for
        % training.
        %            <ModelName>-training
        %            <ModelName>-training/training-ground-truth
        %            <ModelName>-training/validation-ground-truth
        %            <ModelName>-training/best-checkpoint
        %            <ModelName>-training/<BaseModelName>
        %            <ModelName>-training/<ModelName>
        %------------------------------------------------------------------
        function createTemporaryDirectories(this)
 
            % Create <ModelName>-training.
            this.TemporaryDir = char(strcat(fullfile(this.Options.OutputLocation, this.ModelName), '-training'));
            createDir(this.TemporaryDir);
            
            % Create <ModelName>-training/training-ground-truth.
            this.TrainingGTruthDir = fullfile(this.TemporaryDir, 'training-ground-truth');
            createDir(this.TrainingGTruthDir);
            
            % Create <ModelName>-training/validation-ground-truth.
            if ~isempty(this.ValidationDs)
                this.ValidationGTruthDir = fullfile(this.TemporaryDir, 'validation-ground-truth');
                createDir(this.ValidationGTruthDir);
            end

             % Create <ModelName>-training/<ModelName>.
            this.ModelDir = fullfile(this.TemporaryDir, this.ModelName);
            createDir(this.ModelDir);

            % Create <ModelName>-training/<BaseModelName>.
            if this.IsResumeWorkflow
                [~, baseModelName, ~] = fileparts(char(this.BaseModel));
                [~, baseModelName, ~] = fileparts(baseModelName);
            else
                [~, baseModelName, ~] = fileparts(char(this.BaseModel));
            end
            this.BaseCompsDir = fullfile(this.TemporaryDir, baseModelName);
            createDir(this.BaseCompsDir);

            % Create <ModelName>-training/best-checkpoint.
            if ~isempty(this.ValidationDs) && ...
               this.Options.OutputNetwork == "best-validation-loss"

                this.InternalCheckpointPrefix = 'best-validation';
            elseif this.Options.OutputNetwork == "best-training-loss"
                
                this.InternalCheckpointPrefix = 'best-training';
            else
                this.InternalCheckpointPrefix = 'last';
            end
            this.InternalCheckpointDir = fullfile(this.TemporaryDir, ...
                [this.InternalCheckpointPrefix '-checkpoint']);
            createDir(this.InternalCheckpointDir);
            
            %--------------------------------------------------------------
            function createDir(name)
                if ~isfolder(name)
                    [status, msg] = mkdir(name);
                    if ~status
                        error(message("vision:trainOCR:mkdirError", name, msg))
                    end
                end
            end
        end

        %------------------------------------------------------------------
        % Helper to define the behavior of trainOCR when some parameters of
        % trainingOptions are set to "auto".
        %------------------------------------------------------------------
        function defineAutoBehavior(this)

            % Auto behavior of OutputNetwork.
            if this.Options.OutputNetwork == "auto" 
                if isempty(this.ValidationDs)
                    this.Options.OutputNetwork = "best-training-loss";
                else
                    this.Options.OutputNetwork = "best-validation-loss";
                end
            end

            % Auto behavior of CharacterSetSource.
            if this.Options.CharacterSetSource == "auto" 
                % Use base-model as character set source when resuming
                % from checkpoints and ground-truth otherwise.
                if this.IsResumeWorkflow
                    this.Options.CharacterSetSource = "base-model";
                else
                    this.Options.CharacterSetSource = "ground-truth-data";
                end
            end
        end

        %------------------------------------------------------------------
        % Helper to write the list of lsmtf files onto text files.
        %------------------------------------------------------------------
        function writeLstmfFileLists(this)

            % Write training lstmf files list.
            this.TrainingLSTMFilesList = fullfile(this.ModelDir, 'list.train');
            this.NumTrain = vision.internal.ocr.writeLstmfFileLists(...
                this.TrainingLstmfFiles, this.Options.Shuffle, this.TrainingLSTMFilesList);

            % Write validation lstmf files list.
            this.ValidationLSTMFilesList = fullfile(this.ModelDir, 'list.eval');
            vision.internal.ocr.writeLstmfFileLists(this.ValidationLstmfFiles, ...
                this.Options.Shuffle, this.ValidationLSTMFilesList);

            % Verify successful file creation.
            iCheckIfFilesExist(this.TrainingLSTMFilesList);
            iCheckIfFilesExist(this.ValidationLSTMFilesList);
        end

        %------------------------------------------------------------------
        % Helper to extract unicharset from ground truth data.
        %------------------------------------------------------------------
        function extractUnicharsetFromGTruth(this)
            this.GTruthUnicharsetFile = fullfile(this.ModelDir, 'gtruth.unicharset');
            normMode = 2;
            tesseractExtractUnicharset(this.GTruthUnicharsetFile, normMode, ...
                this.TrainingBoxFiles{:}); 
            iCheckIfFilesExist(this.GTruthUnicharsetFile);
        end
    end


    %----------------------------------------------------------------------
    %  Implementation of trainer and validator.
    %----------------------------------------------------------------------
    methods (Access = private)

        %------------------------------------------------------------------
        % Training loop.
        %------------------------------------------------------------------
        function convergenceReason = trainingLoop(this)

            % Setup training progress reporter and start.
            reporter = this.setupTrainingProgressReporter();
            reporter.start();

            % Initialize loop parameters.
            this.HasTrainingConverged = false;
            this.CurrentIteration = 0;
            this.CurrentEpoch = 0;
            this.ValidationPatienceCount = 0;
            for epoch = 1:this.Options.MaxEpochs
                % Track current epoch.
                this.CurrentEpoch = this.CurrentEpoch + 1;

                for i = 1:this.NumTrain
                    
                    % Track current iteration.
                    this.CurrentIteration = this.CurrentIteration + 1;

                    % Train the OCR model on a single sample.
                    this.TrainerObj.trainOnSample();

                    % Save checkpoint for iteration.
                    this.saveIterationCheckpoint();

                    % Run validation.
                    this.runValidation();

                    % Log iteration data.
                    this.logIterationData();

                    % Save best checkpoint.
                    if this.Options.OutputNetwork == "best-training-loss" && ...
                       this.IterationLog.TrainingRMSE < this.BestTrainingLoss

                        this.BestTrainingLoss = this.IterationLog.TrainingRMSE;
                        this.saveInternalCheckpoint();
                    end

                    % Compute time elapsed and report it.
                    this.IterationLog.Time = cputime - this.StartTime;
                    reporter.reportIteration(this.IterationLog);

                    % Check convergence.
                    convergenceReason = this.checkConvergence();

                    if this.HasTrainingConverged
                        % Break the loop if converged.
                        break;
                    end
                end

                % Save checkpoint for epoch.
                this.saveEpochCheckpoint();

                if this.HasTrainingConverged
                    % Break the loop if converged.
                    break;
                end
            end

            if isempty(this.InternalCheckpoint)
                % Save at least one checkpoint if no internal checkpoints
                % were saved. This is the case when Options.OutputNetwork 
                % is set to "last-training".
                this.saveInternalCheckpoint();
            end

            % Check if iterations reached maximum epochs.
            if strlength(convergenceReason) == 0
                convergenceReason = "vision:trainOCR:reachedMaximumEpochs";
            end

            % Compute final time elapsed and report it.
            this.IterationLog.Time = cputime - this.StartTime;
            reporter.finish(this.IterationLog);
        end

        %------------------------------------------------------------------
        % Validation step.
        %------------------------------------------------------------------
        function runValidation(this)

            if ~isempty(this.ValidationDs) && ...
                mod(this.CurrentIteration, this.Options.ValidationFrequency) == 0
                
                % Run validation.
                validationResult = this.TrainerObj.runValidation();

                % Log validation data.
                this.logValidationData(validationResult);

                % Save best checkpoint.
                if this.Options.OutputNetwork == "best-validation-loss" && ...
                   validationResult.RMSE < this.BestValidationLoss

                    this.BestValidationLoss = validationResult.RMSE;
                    this.saveInternalCheckpoint();
                else
                    this.ValidationPatienceCount = this.ValidationPatienceCount + 1;
                end
            end
        end
    end

    %----------------------------------------------------------------------
    %  Helpers for trainer and validator.
    %----------------------------------------------------------------------
    methods (Access = private)

        %------------------------------------------------------------------
        % Helper to setup the reporter that prints training progress table.
        %------------------------------------------------------------------
        function reporter = setupTrainingProgressReporter(this)
            reporter = vision.internal.ocr.VectorReporter();

            if this.Options.Verbose

                if ~isempty(this.ValidationDs)
                    columnStrategy = vision.internal.ocr.RecognitionColumnsWithValidationStatistics();
                else
                    columnStrategy = vision.internal.ocr.RecognitionColumns();
                end

                pd = vision.internal.ocr.ProgressDisplayer(columnStrategy);
                pd.Frequency = this.Options.VerboseFrequency;
                reporter.add(pd);
            end
        end

        %------------------------------------------------------------------
        % Helper to check convergence.
        %------------------------------------------------------------------
        function convergenceReason = checkConvergence(this)
            
            this.HasTrainingConverged = ...
                this.ValidationPatienceCount >= this.Options.ValidationPatience;

            if this.HasTrainingConverged
                convergenceReason = "vision:trainOCR:reachedValidationPatience";
            else
                convergenceReason = "";
            end
        end

        %------------------------------------------------------------------
        % Helper to log iteration data during training.
        %------------------------------------------------------------------
        function logIterationData(this)

            [errorRates, learningRate] = this.TrainerObj.errorRates();

            % Log iteration data.
            this.IterationLog.TrainingRMSE = errorRates.RMSE;
            this.IterationLog.TrainingCharacterError = errorRates.CharError;
            this.IterationLog.TrainingWordError = errorRates.WordError; 
            this.IterationLog.BaseLearnRate = learningRate;
            this.IterationLog.Epoch = this.CurrentEpoch;
            this.IterationLog.Iteration = this.CurrentIteration;

            % Log info struct.
            this.InfoStruct.TrainingRMSE = [this.InfoStruct.TrainingRMSE, this.IterationLog.TrainingRMSE];
            this.InfoStruct.TrainingCharacterError = [this.InfoStruct.TrainingCharacterError, this.IterationLog.TrainingCharacterError];
            this.InfoStruct.TrainingWordError = [this.InfoStruct.TrainingWordError, this.IterationLog.TrainingWordError];
            this.InfoStruct.ValidationRMSE = [this.InfoStruct.ValidationRMSE, this.IterationLog.ValidationRMSE];
            this.InfoStruct.ValidationCharacterError = [this.InfoStruct.ValidationCharacterError, this.IterationLog.ValidationCharacterError];
            this.InfoStruct.ValidationWordError = [this.InfoStruct.ValidationWordError, this.IterationLog.ValidationWordError];
            this.InfoStruct.BaseLearnRate = [this.InfoStruct.BaseLearnRate, this.IterationLog.BaseLearnRate];
        end

        %------------------------------------------------------------------
        % Helper to log validation data.
        %------------------------------------------------------------------
        function logValidationData(this, validationResult)

            this.IterationLog.ValidationRMSE = validationResult.RMSE;
            this.IterationLog.ValidationCharacterError = validationResult.CharError;
            this.IterationLog.ValidationWordError = validationResult.WordError;
        end

        %------------------------------------------------------------------
        % Helper to construct checkpoint name.
        %------------------------------------------------------------------
        function checkpointName = getCheckpointName(this, basename)

            timestamp = datetime('now', 'Format', 'yyyy_MM_dd_hh_mm_ss');
            checkpointName = string(basename)  + "_" + ...
                           this.CurrentIteration   + "_" + ... 
                             string(timestamp)     + ".checkpoint.traineddata";
        end

        %------------------------------------------------------------------
        % Helper to construct internal checkpoint name with the chosen
        % suffix out of "-fast", "-best-training", "-best-validation".
        %------------------------------------------------------------------
        function checkpointName = getInternalCheckpointName(this, basename)
            
            basename = basename + "-" + this.InternalCheckpointPrefix;
            checkpointName = getCheckpointName(this, basename);
        end

        %------------------------------------------------------------------
        % Helper to save external checkpoints in the user requested
        % location and frequency. It is called every epoch.
        %------------------------------------------------------------------
        function saveEpochCheckpoint(this)

            % Do not proceed if the checkpoint frequency unit is not epoch 
            % or if the user has not requested checkpoints to be saved.
            if this.Options.CheckpointFrequencyUnit ~= "epoch" || ...
                strtrim(this.Options.CheckpointPath) == ""
                return
            end

            if mod(this.CurrentEpoch, this.Options.CheckpointFrequency) == 0
                this.saveExternalCheckpoint();
            end
        end

        %------------------------------------------------------------------
        % Helper to save external checkpoints in the user requested
        % location and frequency. It is called every iteration.
        %------------------------------------------------------------------
        function saveIterationCheckpoint(this)

            % Do not proceed if the checkpoint frequency unit is not iteration 
            % or if the user has not requested checkpoints to be saved.
            if this.Options.CheckpointFrequencyUnit ~= "iteration" || ...
                strtrim(this.Options.CheckpointPath) == ""
                return
            end

            if mod(this.CurrentIteration, this.Options.CheckpointFrequency) == 0
                this.saveExternalCheckpoint();
            end
        end

        %------------------------------------------------------------------
        % Helper to save internal checkpoints that can be converted into a
        % final model.
        %------------------------------------------------------------------
        function saveInternalCheckpoint(this)

            if ~isempty(this.InternalCheckpoint)
                % Delete previously saved internal checkpoint to avoid
                % taking up disk space.
                delete(this.InternalCheckpoint)
            end
            
            checkpointName = getInternalCheckpointName(this, this.ModelName);
    
            this.InternalCheckpoint = fullfile(this.InternalCheckpointDir, checkpointName);

            this.TrainerObj.saveCheckpoint(char(this.InternalCheckpoint));
        end

        %------------------------------------------------------------------
        % Helper to save external checkpoints that were requested by users.
        %------------------------------------------------------------------
        function saveExternalCheckpoint(this)
            
            checkpointName = getCheckpointName(this, this.ModelName);
    
            checkpoint = fullfile(this.Options.CheckpointPath, checkpointName);

            this.TrainerObj.saveCheckpoint(char(checkpoint));
        end
    end
end

%--------------------------------------------------------------------------
% Helper to backup checkpoint.
%--------------------------------------------------------------
function newfilename = backupCheckpoint(checkpoint, outputDir)
    
    % Trim the extension twice to get rid of .checkpoint & .traineddata.
    [~, baseName, ~] = fileparts(char(checkpoint));
    [~, baseName, ~] = fileparts(baseName);

    % Rename the file to have .traineddata extesion.
    newfilename = fullfile(outputDir, strcat(baseName, '.traineddata'));

    % Copy backup.
    copyfile(checkpoint, newfilename)
end

%--------------------------------------------------------------------------
% Implementation of LSTMF file generator.
%--------------------------------------------------------------------------
function [lstmfFiles, boxFiles] = generateLSTMFFilesImpl(ds, modelName, lstmfdir, printer, progressMsgID)

    boxFiles  = {};
    lstmfFiles  = {};

    fileCount = 0;

    % Initialize progress information.
    printer.linebreak;
    printer.linebreak;
    msg = iPrintProgress(printer, progressMsgID, "", ds.progress);
    
    while hasdata(ds)
        % Increment file counter.
        fileCount = fileCount + 1;
        
        try
            % Read data.
            [data, info] = read(ds);

            % Validate data contents.
            vision.internal.ocr.validateOCRDataStoreContents(data);
        catch ex

            % Capture file name to display in error message.
            if iscell(info) % when training data is a combined datastore of imds, bxds and txtds.
                filename = info{1}.Filename;
            else % when training data is an image data with a custom read function that returns a 1-by-3 cell array.
                filename = info.Filename;
            end

            error(message('vision:trainOCR:dsReadError', filename, ex.message))
        end
        
        % Parse the read data.
        I = data{:,1};
        bboxes = data{:,2};
        gtruths = data{:,3};

        for boxCount = 1:size(bboxes, 1)
            
            % Create a unique file name.
            boxID = sprintf('%s.val%d_%d', modelName, fileCount, boxCount);
            fileBasename = fullfile(lstmfdir, boxID);
            
            % Write a box file.
            bbox = bboxes(boxCount, :);
            gtruth = char(gtruths(boxCount));
            [boxFile, boxImage] = vision.internal.ocr.generateBoxFile(fileBasename, bbox, gtruth, I);

            % Write image. (for debugging purposes)
            imageFile = strcat(fileBasename, ".jpg");
            imwrite(boxImage, imageFile);

            % Generate an LSMTF file.
            lstmfFile = vision.internal.ocr.generateLSTMFFile(boxImage, boxFile, fileBasename);

            if exist(lstmfFile,'file') == 2
                % Append the file names and gtruth to their respective lists.
                boxFiles = [boxFiles; {boxFile}]; %#ok
                lstmfFiles = [lstmfFiles; {lstmfFile}]; %#ok
            end
        end

        % Update progress information on the commandline.
        msg = iPrintProgress(printer, progressMsgID, msg, ds.progress);
    end

    % Verify successful file creation.
    iCheckIfFilesExist(boxFiles);
    iCheckIfFilesExist(lstmfFiles);
end

% -------------------------------------------------------------------------
function prefix = iUnsetTessDataPrefix()

    prefix = getenv('TESSDATA_PREFIX');
    if ~isempty(prefix)
        setenv('TESSDATA_PREFIX','');
    end
end

% -------------------------------------------------------------------------
function iCheckIfFilesExist(files)

    files = cellstr(files);
    filesExist = cellfun(@(x)exist(x,'file') == 2, files);
    
    if ~all(filesExist)
        missing = files(~filesExist);
        filename = sprintf('%s\n ', missing{:});
        error('Failed to create %s.', filename);
    end
end

%--------------------------------------------------------------------------
function iUpdateMessage(printer, prevMessage, nextMessage)

    % Figure how much to delete.
    backspace = sprintf(repmat('\b',1,numel(prevMessage)));

    % Update printed message using backspace.
    printer.print([backspace nextMessage]);
end

%--------------------------------------------------------------------------
function nextMessage = iPrintProgress(printer, progressMsg, prevMessage, progressPercent)

    % Formulate next message based on inputs.
    nextMessage = sprintf('%s %.2f %% %s', getString(message(progressMsg)),...
        progressPercent*100, getString(message('vision:trainOCR:completed')));

    % Update the printed message on the commandline.
    iUpdateMessage(printer, prevMessage, nextMessage);
end