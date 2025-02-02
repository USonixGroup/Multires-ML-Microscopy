clear all
close all
clc

function thresh_reconstructed_img = coefficients(img)

    % Initialise Parameters
    wavelet = 'db5';
    n = 4;
    t = 0.5; % Threshold (decimal)
    
    %% Discrete Wavelet Transform
    % Extract detail and approximation coefficients for first decomposition
    % level
    [cA, cH1, cV1, cD1] = dwt2(img, wavelet);

    % Loops through each decomposition level calculating coefficients from
    % previous level approximation coefficients
    for level = 2:n
        [cA, cH, cV, cD] = dwt2(cA, wavelet);
        
        % Calculating thresholding parameters
        cA_max = t * max(abs([cA(:)]));
        cH_max = t * max(abs([cH(:)]));
        cV_max = t * max(abs([cV(:)]));
        cD_max = t * max(abs([cD(:)]));

        coeffs{level, 1} = wthresh(cA, 's',cA_max);
        coeffs{level, 2} = wthresh(cH, 's',cH_max);
        coeffs{level, 3} = wthresh(cV, 's',cV_max);
        coeffs{level, 4} = wthresh(cD, 's',cD_max);

    end
    
    %% Reconstruction
    % Obtaining highest decomposition level approximation coefficients
    cA_rec = coeffs{n, 1};

    % Loops through each decomposition level (highest level of 
    % decomposition to lowest) and calculates the reconstructed
    % approximation coefficients for the next level
    for level = n:-1:2
        cH = coeffs{level, 2};
        cV = coeffs{level, 3};
        cD = coeffs{level, 4};
        cA_rec = idwt2(cA_rec, cH, cV, cD, wavelet);
        
        % Resolving issue with extra column
        if size(cA_rec,2) == 96
            cA_rec(:,96) = [];
        end
        
        coeffs{level-1, 1} = cA_rec;
    end

    % Reconstruct the image using the thresholded coefficients (level 1
    % decomposition coefficients)
    thresh_reconstructed_img = idwt2(coeffs{1, 1}, cH1, cV1, cD1, wavelet);

    % Display the original and reconstructed images
    figure;
    subplot(1,2,1);
    imshow(img, []); title('Original Image');

    subplot(1,2,2);
    imshow(thresh_reconstructed_img, []); title('Thresholded Reconstructed Image');
    
end


% MAKE SURE THE FOLDER CONTAINING IMAGES IS ADDED TO PATH AND YOU ARE IN
% THE SAME FOLDER HIERACHY AS THE IMAGE FOLDER

% Define folder containing test images
folder_path = 'test_images';
imageFiles = dir(fullfile(folder_path, '*.png'));

% Initialisation of parameters
imageNumber = 1;

% Get the full image path
imagePath = fullfile(folder_path, imageFiles(imageNumber).name);

% Importing image
im=imread(imagePath);

% Coverts image to grayscale by taking mean of the 3 colour channels
img = mean(im,3);
img = rescale(img);

% Applies the DWT thresholding function and returns the denoised image
denoised_img = coefficients(img);