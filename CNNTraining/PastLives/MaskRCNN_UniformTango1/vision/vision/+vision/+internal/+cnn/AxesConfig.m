classdef (Abstract) AxesConfig
    % AxesConfig   Class that acts as Vision wrapper for nnet.internal.cnn.ui.AxesConfiguration.

    %   Copyright 2020 The MathWorks, Inc.
    
    properties (Abstract)
        % AxesConfiguration(nnet.internal.cnn.ui.AxesConfiguration) Holds 
        % a cell array of AxesProperties for each
        % axis and a MetricRowDataFactory. Also calculates number of Axes.

        AxesConfiguration
    end
 
end
