function useParallel = validateUseParallel(useParallel,varargin)
% validate UseParallel and check if there is a pool to use. If a pool is
% not available return false.
%
% Optionally specify inputs for GCP. For example, to prevent this function
% from opening a pool specify 'nocreate'.

%   Copyright 2014-2020 The MathWorks, Inc.

vision.internal.inputValidation.validateLogical(useParallel, 'UseParallel');

if useParallel
    try
        % gcp() will error if the Parallel Computing Toolbox is not
        % installed, or if it is unable to check out a license
        currPool = gcp(varargin{:});
        if isempty(currPool)           
            useParallel = false;                  
        end        
    catch        
        useParallel = false;
    end
end

