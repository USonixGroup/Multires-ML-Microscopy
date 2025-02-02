clear all
close all
clc

function coeffs = coefficients(img)

    % Initialise Parameters
    wname = 'db5';
    n = 4;
    t = 0.5;

    % Extract detail and approximation coefficients
    [cA, cH1, cV1, cD1] = dwt2(img, wname);

    for level = 2:n
        [cA, cH, cV, cD] = dwt2(cA, wname);
        
        % Thresholding
        cA_max = t * max(abs([cA(:)]));
        cH_max = t * max(abs([cH(:)]));
        cV_max = t * max(abs([cV(:)]));
        cD_max = t * max(abs([cD(:)]));

        coeffs{level, 1} = wthresh(cA, 's',cA_max);
        coeffs{level, 2} = wthresh(cH, 's',cH_max);
        coeffs{level, 3} = wthresh(cV, 's',cV_max);
        coeffs{level, 4} = wthresh(cD, 's',cD_max);

    end
    
    cA_rec = coeffs{n, 1};
    for level = n:-1:2
        cH = coeffs{level, 2};
        cV = coeffs{level, 3};
        cD = coeffs{level, 4};
        cA_rec = idwt2(cA_rec, cH, cV, cD, wname);
        
        if size(cA_rec,2) == 96
            cA_rec(:,96) = [];
        end

        coeffs{level-1, 1} = cA_rec;
    end

    % Reconstruct the image using the thresholded coefficients
    thresh_reconstructed_img = idwt2(coeffs{1, 1}, cH1, cV1, cD1, wname);

    % Display the original and reconstructed images
    figure;
    subplot(1,2,1); imshow(img, []); title('Original Image');
    size(img)
    subplot(1,2,2); imshow(thresh_reconstructed_img, []); title('Thresholded Reconstructed Image');
    size(thresh_reconstructed_img)
   
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

% % Displaying the original image
% figure
% imshow(img)
% title('Original')

% % Displaying the image after noise added
% figure
% imshow(noisy_image)
% title('Additional Noise')

% % Displaying the image after DWT denoising is applied
% figure
% imshow(reconstructed_image)
% title('Denoised')

coeffs = coefficients(img); % Calculate the DWT coefficients (For CNN input)