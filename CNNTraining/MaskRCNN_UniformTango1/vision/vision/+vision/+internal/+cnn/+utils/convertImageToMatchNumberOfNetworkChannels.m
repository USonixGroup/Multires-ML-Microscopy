function Icell = convertImageToMatchNumberOfNetworkChannels(Icell, colorPreprocessing)
%CONVERTIMAGETOMATCHNUMBEROFNETWORKCHANNELS Based on colorPreprocessing
% value convert images to rgb2gray or gray2rgb.

% Copyright 2019 The MathWorks, Inc.

    for ii = 1:numel(Icell)
        isImageRGB = ~ismatrix(Icell{ii});

        checkChannelSize = false;
        switch colorPreprocessing.Preprocessing
            case 'rgb2gray'
                if isImageRGB
                    Icell{ii} = rgb2gray(Icell{ii});
                else
                    checkChannelSize = true;
                end
            case 'gray2rgb'
                if ~isImageRGB
                    Icell{ii} = repmat(Icell{ii},1,1,3);
                else
                    checkChannelSize = true;
                end
            case 'multi-channel'
                % do nothing, just check if channel sizes are equal,
                % after the switch case
                checkChannelSize = true;
        end
        if checkChannelSize
            vision.internal.cnn.validation.checkImageAndNetworkChannelSizes(...
                Icell{ii}, colorPreprocessing.NetworkInputChannelSize);
        end
    end
end
