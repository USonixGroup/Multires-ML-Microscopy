classdef PropertyCache < handle
    % Handle class for sharing numeric values by reference. Used to hold
    % layer property values that need to be updated during inference.
    
    % Copyright 2018 The MathWorks, Inc.
    properties        
        Value = []
    end
    
    methods       
        function this = PropertyCache(value)
            if nargin == 1
                this.Value = value;
            end
        end
        
        function value = get.Value(this)
            value = this.Value;
        end         
    end
    
    methods(Static)
        function val = fetchValue(aValue)
            if isa(aValue,'vision.internal.cnn.layer.util.PropertyCache')
                val = aValue.Value;
            else
                val = aValue;
            end                
        end
    end
end

    