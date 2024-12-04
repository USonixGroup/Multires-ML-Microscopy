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
    
    % Get approximation coefficients for the current level
    A = appcoef2(C, S, wavelet, n);
    
    reconstructed_img = wrcoef2("a",C,S,wavelet,n);
    
end


function [approx_coeff, vert_coeff, horiz_coeff, diag_coeff] = coefficients(input_image)
    % DWT_coefficients
    % This function takes an input image, performs the DWT at a specified
    % decomposition level, and returns the approximation, vertical,
    % horizontal, and diagonal coefficients.
    
    % Input arguments:
    % input_image: the reconstructed image
    
    % Outputs:
    % approx_coeff: Approximation coefficients
    % vert_coeff: Vertical detail coefficients
    % horiz_coeff: Horizontal detail coefficients
    % diag_coeff: Diagonal detail coefficients
    
    % Optionally: Display a plot containing the coefficients

    % Initialisation of parameters
    wavelet = 'db5'; % Wavelet function
    n = 2; % Levels of decomposition
    
    % Perform wavelet decomposition up to the specified level
    [C, S] = wavedec2(input_image, n, wavelet);
    
    % Extract approximation coefficients
    approx_coeff = appcoef2(C, S, wavelet, n);
    
    % Extract detail coefficients: vertical, horizontal, and diagonal
    [vert_coeff, horiz_coeff, diag_coeff] = detcoef2('all', C, S, n);
    
    % OPTIONAL: Display the coefficients for the current level
    figure
    subplot(2, 2, 1)
    imagesc(approx_coeff)
    colormap(gray)
    title(['Approximation Coef. of Level ' num2str(n)])

    subplot(2, 2, 2)
    imagesc(horiz_coeff)
    title(['Horizontal Detail Coef. of Level ' num2str(n)])

    subplot(2, 2, 3)
    imagesc(vert_coeff)
    title(['Vertical Detail Coef. of Level ' num2str(n)])

    subplot(2, 2, 4)
    imagesc(diag_coeff)
    title(['Diagonal Detail Coef. of Level ' num2str(n)])
end

reconstructed_image = denoise(img); % Apply denoising function 
[approx_coeff, vert_coeff, horiz_coeff, diag_coeff] = coefficients(reconstructed_image); % Calculate the DWT coefficients (For CNN input)