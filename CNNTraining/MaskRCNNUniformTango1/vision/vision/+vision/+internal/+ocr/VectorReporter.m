classdef VectorReporter < vision.internal.ocr.Reporter
    % VectorReporter   Container to hold a series of reporters.
    %
    % Note: This is a CVT equivalent of DLT's nnet.internal.cnn.util.VectorReporter.

    %   Copyright 2022 The MathWorks, Inc.
    
    properties

        Reporters
    end
    
    methods
        %------------------------------------------------------------------
        function start( this )

            computeAndReport( this, 'start' );
        end
        
        %------------------------------------------------------------------
        function reportIteration( this, summary )

            computeAndReport( this, 'reportIteration', summary );
        end
        
        %------------------------------------------------------------------
        function finish( this, summary)

            computeAndReport( this, 'finish', summary);
        end
        
        %------------------------------------------------------------------
        function add( this, reporter )
            
            this.Reporters = cat(1, this.Reporters, { reporter });
        end
    end
    
    methods( Access = private )
        %------------------------------------------------------------------
        function computeAndReport( this, method, varargin )

            errors = {};
            for i = 1:length(this.Reporters)
                % Use try..catch to ensure all reporters get a chance to
                % update.
                try
                    feval( method, this.Reporters{i}, varargin{:} );
                catch me
                    errors{end+1} = me; %#ok<AGROW>
                end
            end
            % Throw the first of any errors.
            if ~isempty(errors)
                rethrow(errors{1});
            end
        end
    end
end
