function avg = perChannelMean(I, networkInputSize)
%PERCHANNELMEAN Compute per channel mean for an input image, I.

% Copyright 2019 The MathWorks, Inc.

    avg           = mean(mean(I));
    netInputIsRGB = networkInputSize(3) > 1;

    if netInputIsRGB
        if isscalar(avg)
            % I is grayscale, duplicate to form RGB channel mean.
            avg = repelem(avg, 1,1,3);
        end
    else
        % net input is grayscale, I is RGB. Use RGB2GRAY to convert.
        if ~isscalar(avg)
            avg = rgb2gray(avg);
        end
    end
end
