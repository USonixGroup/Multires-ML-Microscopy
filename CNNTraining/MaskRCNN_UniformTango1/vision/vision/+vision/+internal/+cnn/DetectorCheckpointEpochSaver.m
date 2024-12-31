classdef DetectorCheckpointEpochSaver < nnet.internal.cnn.trainNetwork.reporter.checkpoint.EpochReporter
    % DetectorCheckpointEpochSaver   This class is used to save detector
    % checkpoints at specific epochs during network training. Use this
    % class if you have a detector that contains a network that you'd like
    % to checkpoint.

    %   Copyright 2016-2021 The MathWorks, Inc.
    
    properties                       
        % Copy of the partially trained detector. Detector have a one or
        % more networks. The DetectorFcn assigns networks into the detector
        % and then saves the detector.
        Detector
        
        % DetectorFcn Detector function called for assigning network
        % checkpoint into this.Detector.                
        DetectorFcn
        
        % CheckpointPrefix String prefix to use for the checkpoint mat
        % filename, e.g. fast_rcnn or faster_rcnn.
        CheckpointPrefix
    end
    
    methods
        function this = DetectorCheckpointEpochSaver( checkpointSaver )            
             this@nnet.internal.cnn.trainNetwork.reporter.checkpoint.EpochReporter(checkpointSaver);                                  
        end
        
        function setup( ~, ~ )
        end
        
        function start( ~ )
        end
        
        function reportIteration( ~, ~, ~, ~, ~, ~, ~ )
        end
        
        function reportEpoch( this, epoch, iteration, network )
            if this.Saver.canSaveThisInstance(epoch)
                this.saveCheckpoint( network, iteration );
            end
        end
        
        function finish( ~, ~, ~ )
        end
    end
    
    methods(Access = private)
        function saveCheckpoint(this, net, iteration)
            assert(~isempty(this.CheckpointPrefix));
            assert(~isempty(this.Detector));
            
            checkpointPath = this.Saver.CheckpointPath;
            
            name = this.generateCheckpointName(iteration);
            
            fullPath = fullfile(checkpointPath, name);
            
            % convert internal network to external network.
            network = this.Saver.ConvertorFcn(net);
            
            % store external network into detector checkpoint.
            detector = this.DetectorFcn(network, this.Detector);
            
            iSaveDetector(fullPath, detector);
        end
        
        function name = generateCheckpointName(this, iteration)
            basename = [this.CheckpointPrefix '_checkpoint'];
            timestamp = char(datetime('now', 'Format', 'yyyy_MM_dd__HH_mm_ss'));
            name = [ basename '__' int2str(iteration) '__' timestamp '.mat' ];
        end
    end
end

function iSaveDetector(fullPath,detector)
try
    iSave(fullPath, 'detector', detector);
catch e
    warning( message('nnet_cnn:internal:cnn:Trainer:SaveCheckpointFailed', fullPath, e.message ) )
end
end

function iSave(fullPath, name, value)
S.(name) = value; %#ok<STRNU>
save(fullPath, '-struct', 'S', name);
end


