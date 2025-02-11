% This class wraps the RANDPERM function to provide a mechanism to simplify
% testing.
classdef RandomSelector

%   Copyright 2016-2020 The MathWorks, Inc.

    methods
        function i = randperm(~, N, K)
            if nargin > 2
                i = randperm(N, K);
            else
                i = randperm(N);
            end
        end
    end
end
