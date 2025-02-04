function [imSz,isBatchOfImages] = checkDetectionInputImage(networkInputSize,sampleImage,validateChannelSize,validateImageSize)
%checkDetectionInputImage - Validate the detection input image with network inputsize.
%   networkInputSize    - Input size of the network.
%   sampleImage         - Input Image:
%                            H-by-W for a grayscale image.
%                            H-by-W-by-3 for a truecolor image.
%                            H-by-W-by-1-by-B for grayscale batch of images.
%                            H-by-W-by-3-by-B for truecolor batch of images.
%                            H-by-W-by-C-by-B for multi-channel batch of images.
%   validateChannelSize - Logical to indicate whether to validate channel size
%                         of the network with that of the input image.
%   validateImageSize   - Logical to indicate whether to validate image size
%                         must be greater than or equal to the network input size.
    
%   Copyright 2019 The MathWorks, Inc.

    [imSz(1),imSz(2),imSz(3),imSz(4)] = size(sampleImage);

    networkChannelSize = networkInputSize(3);
    imageChannelSize = imSz(3);

    isBatchOfImages =  imSz(4) > 1;

    if isBatchOfImages
        % Pass the first image in the batch for internal validation.
        sampleImage = sampleImage(:,:,:,1);
    end

    if networkChannelSize > 3 || networkChannelSize == 2
        args = {sampleImage, 'I', 'multi-channel'};
    else
        args = {sampleImage, 'I'};
    end
    % multi-channel or grayscale or RGB images allowed
    vision.internal.inputValidation.validateImage(args{:});

    if validateImageSize && any(imSz(1:2) < networkInputSize(1:2))
        error(message('vision:ObjectDetector:imageSmallerThanNetwork',mat2str(networkInputSize(1:2))));
    end

    % Validate number of channels for input image and network input
    if validateChannelSize && imageChannelSize ~= networkChannelSize
        error(message('vision:ObjectDetector:invalidInputImageChannelSize',imageChannelSize,networkChannelSize));
    end
end
