clear all
close all
clc

% DWT_function
% This functions objective is to apply the DWT onto an image, retaining
% only the approximation coefficients, and reconstructing the image.
% Thus by removing the 'high-frequency noise' the image will be denoised.


% Define folder containing test images
folder_path = 'test_images';
imageFiles = dir(fullfile(folder_path, '*.png'));

% Initialisation of parameters
wavelet = 'db5'; % Wavelet function
NumberImages = 1; % Number of images being tested (if only testing subset)
n = 3; % Levels of decomposition



for imageNumber = 1:NumberImages
    % Get the full image path
    imagePath = fullfile(folder_path, imageFiles(imageNumber).name);
    
    % Importing image
    im=imread(imagePath);

    % Coverts image to grayscale by taking mean of the 3 colour channels
    imgs = mean(im,3);

    % Perform wavelet decomposition up to n levels
    % C = wavelet decomposition vector
    % S = bookkeeping matrix
    [C, S] = wavedec2(imgs, n, wavelet);

    % Get detail coefficients for the current level
    [H, V, D] = detcoef2('all', C, S, n);

    % Get approximation coefficients for the current level
    A = appcoef2(C, S, wavelet, n);
    
    reconstructed_img = wrcoef2("a",C,S,wavelet,n);
    
    % Calculate SSIM and PSNR between the original and reconstructed image
    ssim_val = ssim(double(reconstructed_img), imgs);
    psnr_val = psnr(double(reconstructed_img)/ 255, double(imgs)/ 255);

end
