clear all
close all
clc

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


function reconstructed_img = denoise(img)
    % INPUT: img = Original Image
    % OUTPUT: reconstructed_img = Denoised Image
    % Function takes the original image, performs DWT, reconstructs image
    % using only approximation coefficients, thus removes high-frequency
    % noise.

    % Initialisation of parameters
    wavelet = 'db5'; % Wavelet function
    n = 1; % Levels of decomposition

    % Perform wavelet decomposition up to n levels
    % C = wavelet decomposition vector
    % S = bookkeeping matrix
    [C, S] = wavedec2(img, n, wavelet);
    
    % Get detail coefficients for the current level
    [H, V, D] = detcoef2('all', C, S, n);
    
    % Get approximation coefficients for the current level
    A = appcoef2(C, S, wavelet, n);
    
    reconstructed_img = wrcoef2("a",C,S,wavelet,n);
    
end


reconstructed_image = denoise(img); % Apply the function
