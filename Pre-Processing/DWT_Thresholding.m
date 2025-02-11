clear all
close all
clc

function thresh_reconstructed_img = coefficients(img, params)

arguments
    params.Wavelet char = 'db5'; %wavelet type
    params.Levels mustBeInteger = 4; %level of decomposition
    params.Threshold {mustBeReal, mustBeGreaterThanOrEqual(params.Threshold, 0), mustBeLessThanOrEqual(params.Threshold, 1)} = 0.2; 
    %threhsold to be taken at each level: singe value in range [0,1] or
    %array of values of size (1,Levels) listing threshold for each level of
    %decomposition
    params.Display logical = false; %option to display output as it processes
end
    %% Discrete Wavelet Transform    
    % Loops through each decomposition level calculating coefficients from
    % previous level approximation coefficients
    for level = 1:params.Levels
        if level == 1
            [cA, cH, cV, cD] = dwt2(img, wavelet);
        else
            [cA, cH, cV, cD] = dwt2(cA, wavelet);
        end

        % Calculating thresholding parameters
        cH_max = threshold * max(abs([cH(:)]));
        cV_max = threshold * max(abs([cV(:)]));
        cD_max = threshold * max(abs([cD(:)]));

        coeffs{level, 1} = cA;
        coeffs{level, 2} = wthresh(cH,'s',cH_max);
        coeffs{level, 3} = wthresh(cV,'s',cV_max);
        coeffs{level, 4} = wthresh(cD,'s',cD_max);

    end
    
    %% Reconstruction
    % Obtaining highest decomposition level approximation coefficients
    cA_rec = coeffs{level, 1};

    % Loops through each decomposition level (highest level of 
    % decomposition to lowest) and calculates the reconstructed
    % approximation coefficients for the next level
    for level = params.Levels:-1:2
        cH = coeffs{level, 2};
        cV = coeffs{level, 3};
        cD = coeffs{level, 4};
        cA_rec = idwt2(cA_rec, cH, cV, cD, wavelet);
        
        % Bug fix - resolving issue with extra column
        if size(cA_rec,2) == 96
            cA_rec(:,96) = [];
        end
        
        coeffs{level-1, 1} = cA_rec;
    end

    % Reconstruct the image using the thresholded coefficients (level 1
    % decomposition coefficients)
    cA = coeffs{1, 1};
    cH = coeffs{1, 2};
    cV = coeffs{1, 3};
    cD = coeffs{1, 4};
    thresh_reconstructed_img = idwt2(cA, cH, cV, cD, wavelet);

    if params.Display == 1;
    % Display the original and reconstructed images
    figure;
    subplot(1,2,1);
    imshow(img, []);
    title('Original Image');

    subplot(1,2,2);
    imshow(thresh_reconstructed_img, []);
    title('Thresholded Reconstructed Image');
    end
    
end

% 
% % MAKE SURE THE FOLDER CONTAINING IMAGES IS ADDED TO PATH AND YOU ARE IN
% % THE SAME FOLDER HIERACHY AS THE IMAGE FOLDER
% 
% % Define folder containing test images
% folder_path = 'test_images';
% imageFiles = dir(fullfile(folder_path, '*.png'));
% 
% % Initialisation of parameters
% imageNumber = 1;
% 
% % Get the full image path
% imagePath = fullfile(folder_path, imageFiles(imageNumber).name);
% 
% % Importing image
% im=imread(imagePath);
% 
% % Coverts image to grayscale by taking mean of the 3 colour channels
% img = mean(im,3);
% img = rescale(img);
% 
% % Applies the DWT thresholding function and returns the denoised image
% denoised_img = coefficients(img);