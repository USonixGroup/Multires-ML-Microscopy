clear all
close all
clc





% db_wavelets = {'db1', 'db2', 'db3', 'db4', 'db5', 'db6', 'db7', 'db8', ...
%                'db9', 'db10', 'db11', 'db12', 'db13', 'db14', 'db15', ...
%                'db16', 'db17', 'db18', 'db19', 'db20', 'db21', 'db22', ...
%                'db23', 'db24', 'db25', 'db26', 'db27', 'db28', 'db29', ...
%                'db30', 'db31', 'db32', 'db33', 'db34', 'db35', 'db36', ...
%                'db37', 'db38', 'db39', 'db40', 'db41', 'db42', 'db43', ...
%                'db44', 'db45'};
% 
% sym_wavelets = {'sym2', 'sym3', 'sym4', 'sym5', 'sym6', 'sym7', 'sym8', ...
%                 'sym9', 'sym10', 'sym11', 'sym12', 'sym13', 'sym14', ...
%                 'sym15', 'sym16', 'sym17', 'sym18', 'sym19', 'sym20', ...
%                 'sym21', 'sym22', 'sym23', 'sym24', 'sym25', 'sym26', ...
%                 'sym27', 'sym28', 'sym29', 'sym30', 'sym31', 'sym32', ...
%                 'sym33', 'sym34', 'sym35', 'sym36', 'sym37', 'sym38', ...
%                 'sym39', 'sym40', 'sym41', 'sym42', 'sym43', 'sym44', ...
%                 'sym45'};
% 
% coif_wavelets = {'coif1', 'coif2', 'coif3', 'coif4', 'coif5'};
% 
% bior_wavelets = {'bior1.1', 'bior1.3', 'bior1.5', ...
%                  'bior2.2', 'bior2.4', 'bior2.6', 'bior2.8', ...
%                  'bior3.1', 'bior3.3', 'bior3.5', 'bior3.7', ...
%                  'bior4.4', 'bior5.5', 'bior6.8'};
% 
% rbio_wavelets = {'rbio1.1', 'rbio1.3', 'rbio1.5', ...
%                  'rbio2.2', 'rbio2.4', 'rbio2.6', 'rbio2.8', ...
%                  'rbio3.1', 'rbio3.3', 'rbio3.5', 'rbio3.7', ...
%                  'rbio4.4', 'rbio5.5', 'rbio6.8'};

db_wavelets = {'db1', 'db2', 'db3', 'db4', 'db5', 'db6', 'db7', 'db8', ...
               'db9', 'db10', 'db11', 'db12', 'db13', 'db14', 'db15', ...
               'db16', 'db17', 'db18', 'db19', 'db20'};

sym_wavelets = {'sym2', 'sym3', 'sym4', 'sym5', 'sym6', 'sym7', 'sym8', ...
                'sym9', 'sym10', 'sym11', 'sym12', 'sym13', 'sym14', ...
                'sym15', 'sym16', 'sym17', 'sym18', 'sym19', 'sym20'};

coif_wavelets = {'coif1', 'coif2', 'coif3', 'coif4', 'coif5'};

bior_wavelets = {'bior1.1', 'bior1.3', 'bior1.5', ...
                 'bior2.2', 'bior2.4', 'bior2.6', 'bior2.8', ...
                 'bior3.1', 'bior3.3', 'bior3.5', 'bior3.7', ...
                 'bior4.4', 'bior5.5', 'bior6.8'};

rbio_wavelets = {'rbio1.1', 'rbio1.3', 'rbio1.5', ...
                 'rbio2.2', 'rbio2.4', 'rbio2.6', 'rbio2.8', ...
                 'rbio3.1', 'rbio3.3', 'rbio3.5', 'rbio3.7', ...
                 'rbio4.4', 'rbio5.5', 'rbio6.8'};

wavelets = [db_wavelets, sym_wavelets, coif_wavelets, bior_wavelets, rbio_wavelets];


% Define folder containing test images
folder_path = 'test_images';
imageFiles = dir(fullfile(folder_path, '*.png'));


NumberImages = 5;
% wavelets = db_wavelets;
n = 5;

ssim_values = zeros(NumberImages, length(wavelets), n);
psnr_values = zeros(NumberImages, length(wavelets), n);
elapsed_times = zeros(NumberImages, length(wavelets));


for imageNumber = 1:NumberImages
    imageNumber
    % Get the full image path
    imagePath = fullfile(folder_path, imageFiles(imageNumber).name);

    % Importing image
    im=imread(imagePath);
   
    % Coverts image to grayscale by taking mean of the 3 colour channels
    imgs = mean(im,3);
    
      
    % Loop through each wavelet function
    for w = 1:length(wavelets)
        wavelet_name = wavelets{w};
        fprintf('\nUsing wavelet: %s\n', wavelet_name);
            
        tic;

        % Perform wavelet decomposition up to n levels with the current wavelet
        [C, S] = wavedec2(imgs, n, wavelet_name);        
        
        elapsed_times(imageNumber, w) = toc;
        fprintf('The wavelet transform with %s took %.4f seconds.\n', wavelet_name, elapsed_times(w));


        for level = 1:n
            % Get detail coefficients for the current level
            [H, V, D] = detcoef2('all', C, S, level);
    
            % Get approximation coefficients for the current level
            A = appcoef2(C, S, wavelet_name, level);
            
            C_approx = C;
            
            % Zero out the detail coefficients for each level up to the specified level
            for j = 1:level
                % Determine the starting indices for the details at this level
                % Size of the current level's detail coefficients
                level_size = S(end-j+1, :);
                num_coeffs = prod(level_size);
    
                % Calculate starting indices for H, V, and D in the coefficient array C
                startIdxH = sum(prod(S(1:end-j, :), 2)) + 1;
                startIdxV = startIdxH + num_coeffs;
                startIdxD = startIdxV + num_coeffs;
    
                % Zero out the coefficients in the C array for horizontal, vertical, and diagonal details
                C_approx(startIdxH:startIdxH + num_coeffs - 1) = 0; % Horizontal detail
                C_approx(startIdxV:startIdxV + num_coeffs - 1) = 0; % Vertical detail
                C_approx(startIdxD:startIdxD + num_coeffs - 1) = 0; % Diagonal detail
            end
            
            % Reconstruct the image using only the approximation coefficients
            reconstructed_img = waverec2(C_approx, S, wavelet_name);

            % Resize the reconstructed image to match the original dimensions, if necessary
            if size(reconstructed_img) ~= size(imgs)
                reconstructed_img = imresize(reconstructed_img, size(imgs));
            end

            % Calculate SSIM between the original and reconstructed image
            ssim_val = ssim(double(reconstructed_img), imgs);
    
            % Calculate PSNR between the original and reconstructed image
            peaksnr_val = psnr(double(reconstructed_img)/ 255, double(imgs)/ 255);

            % Display the metric values
            disp(['Level ' num2str(level) ': SSIM = ' num2str(ssim_val)]);
            disp(['Level ' num2str(level) ': PSNR = ' num2str(peaksnr_val)]);

            ssim_values(imageNumber, w, level) = ssim_val; % Store the SSIM value
            psnr_values(imageNumber, w, level) = peaksnr_val; % Store the PSNR value

            % Display the reconstructed image
            % figure
            % imagesc(uint8(reconstructed_img))
            % colormap(gray)
    
            % title(['Reconstructed Image using Approximation Coefficients of Level ' num2str(level)])
        end
        
    end

end

% Calculating average metrics across all images
average_ssim = mean(ssim_values, 1);
average_ssim = squeeze(average_ssim);
average_psnr = mean(psnr_values, 1);
average_psnr = squeeze(average_psnr);
average_elapsed_time = mean(elapsed_times, 1);

figure;

% Plot SSIM values for each decomposition level as separate lines
plot(average_ssim(:,1), '-o', 'LineWidth', 2); % Level 1
hold on;
plot(average_ssim(:,2), '-o', 'LineWidth', 2); % Level 2
plot(average_ssim(:,3), '-o', 'LineWidth', 2); % Level 3
plot(average_ssim(:,4), '-o', 'LineWidth', 2); % Level 4
plot(average_ssim(:,5), '-o', 'LineWidth', 2); % Level 5

% Set x-ticks to match the number of wavelet functions
num_wavelets = size(average_ssim, 1); % Number of wavelets
set(gca, 'XTick', 1:num_wavelets); % Set x-tick positions to each wavelet index
set(gca, 'XTickLabel', wavelets); % Set x-tick labels to the wavelet names
xtickangle(90); % Rotate x-axis labels for readability

% Set font properties for labels, title, and legend
set(gca, 'FontName', 'Times New Roman', 'FontSize', 14); % Axis font

% Label axes with specified font and size
xlabel('Wavelet Function', 'FontName', 'Times New Roman', 'FontSize', 14);
ylabel('SSIM', 'FontName', 'Times New Roman', 'FontSize', 14);

% Title with specific font and size
title('SSIM Values against Wavelet Functions', ...
    'FontName', 'Times New Roman', 'FontSize', 16);

% Legend with specified font size
legend({'Level 1', 'Level 2', 'Level 3', 'Level 4', 'Level 5'}, ...
    'FontSize', 12, 'FontName', 'Times New Roman');

% Enable grid and minor grid with specified font
grid on;
grid minor;

hold off; % Release the hold on the current figure



figure;

% Plot PSNR values for each decomposition level as separate lines
plot(average_psnr(:,1), '-x', 'LineWidth', 2); % Level 1
hold on;
plot(average_psnr(:,2), '-x', 'LineWidth', 2); % Level 2
plot(average_psnr(:,3), '-x', 'LineWidth', 2); % Level 3
plot(average_psnr(:,4), '-x', 'LineWidth', 2); % Level 4
plot(average_psnr(:,5), '-x', 'LineWidth', 2); % Level 5

% Set x-ticks to match the number of wavelet functions
num_wavelets = size(average_psnr, 1); % Number of wavelets
set(gca, 'XTick', 1:num_wavelets); % Set x-tick positions to each wavelet index
set(gca, 'XTickLabel', wavelets); % Set x-tick labels to the wavelet names
xtickangle(90); % Rotate x-axis labels for readability

% Set font properties for labels, title, and legend
set(gca, 'FontName', 'Times New Roman', 'FontSize', 14); % Axis font

% Label axes with specified font and size
xlabel('Wavelet Function', 'FontName', 'Times New Roman', 'FontSize', 14);
ylabel('PSNR (dB)', 'FontName', 'Times New Roman', 'FontSize', 14);

% Title with specific font and size
title('PSNR Values against Wavelet Functions', ...
    'FontName', 'Times New Roman', 'FontSize', 16);

% Legend with specified font size
legend({'Level 1', 'Level 2', 'Level 3', 'Level 4', 'Level 5'}, ...
    'FontSize', 12, 'FontName', 'Times New Roman');

% Enable grid and minor grid with specified font
grid on;
grid minor;

hold off; % Release the hold on the current figure



figure;

% Plot PSNR values for each decomposition level as separate lines
plot(average_elapsed_time, '-o', 'LineWidth', 2);

% Set x-ticks to match the number of wavelet functions
num_wavelets = size(average_psnr, 1); % Number of wavelets
set(gca, 'XTick', 1:num_wavelets); % Set x-tick positions to each wavelet index
set(gca, 'XTickLabel', wavelets); % Set x-tick labels to the wavelet names
xtickangle(90); % Rotate x-axis labels for readability

% Set font properties for labels, title, and legend
set(gca, 'FontName', 'Times New Roman', 'FontSize', 14); % Axis font

% Label axes with specified font and size
xlabel('Wavelet Function', 'FontName', 'Times New Roman', 'FontSize', 14);
ylabel('Time (s)', 'FontName', 'Times New Roman', 'FontSize', 14);

% Title with specific font and size
title('Time for Discrete Wavelet Transform Computations', ...
    'FontName', 'Times New Roman', 'FontSize', 16);

% Enable grid and minor grid with specified font
grid on;
grid minor;

hold off; % Release the hold on the current figure


