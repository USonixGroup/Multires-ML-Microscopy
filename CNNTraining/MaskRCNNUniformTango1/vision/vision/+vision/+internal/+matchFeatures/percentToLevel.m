function matchThreshold = percentToLevel(matchPercentage, ...
    vector_length, metric, outputClass)
%percentToLevel Convert the match threshold from a percentage value to an
%   absolute value based on the type of metric 

%   Copyright 2020 The MathWorks, Inc.

%#codegen
matchPercentage = cast(matchPercentage, outputClass);
vector_length = cast(vector_length, outputClass);

if (strcmp(metric, 'normxcorr'))
    matchThreshold = cast(0.01, outputClass)*(cast(100, outputClass) ...
        - matchPercentage);
else
    if (strcmp(metric, 'sad'))
        max_val = cast(2, outputClass)*sqrt(vector_length);
    elseif (strcmp(metric, 'ssd'))
        max_val = cast(4, outputClass);
    else % 'hamming'
        % the below value assumes that binary features are stored
        % in 8-bit buckets, which is correct for binaryFeatures class
        max_val = cast(8*vector_length, outputClass);
    end
    
    matchThreshold = (matchPercentage*cast(0.01, outputClass))*max_val;
    
    if strcmp(metric, 'hamming')
        % Round up since we are dealing with whole bits
        matchThreshold = round(matchThreshold);
    end
end