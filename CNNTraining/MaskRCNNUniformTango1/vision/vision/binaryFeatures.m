classdef binaryFeatures < vision.internal.EnforceScalarValue

    % Copyright 2012-2024 The MathWorks, Inc.

    %#codegen

    properties (SetAccess='private', GetAccess='public')
        Features;
        NumBits;
        NumFeatures;
    end
    
    methods
        %------------------------------------------------------------------
        % Constructor
        %------------------------------------------------------------------
        function this = binaryFeatures(in)
            validateattributes(in, {'uint8'}, {'2d', 'real'}, ...
                'binaryFeatures', 'FEATURE_VECTORS'); 
            
            numBitsInUint8 = 8;
            
            this.NumBits     = size(in, 2) * numBitsInUint8;
            this.NumFeatures = size(in, 1);           
            this.Features    = in;
        end             
    end
end
