function [Masks, Labels, Scores, Boxes] = segmentSobelWatershed(I_gray, sigma, DiskSize, polarity)
% 
%   Sobel + marker-controlled Watershed cell segmenter.
%
%   Wraps the Sobel/watershed pipeline so that its outputs are compatible
%   with the Multires ML Microscopy app. To avoid allocating an M×N×K
%   logical mask stack (which is prohibitive for images with many cells),
%   Masks are returned as a compact M×N uint32 watershed label matrix.
%   The app's Sobel+Watershed branch reconstructs per-cell binary masks
%   from this label matrix one at a time as needed.
%
%   Outputs
%   -------
%   Masks  – M x N uint32 label matrix; pixel value = basin index,
%            0 = watershed boundary or background basin
%   Labels – K x 1 categorical array  (all entries = 'cell')
%   Scores – K x 1 double array       (all entries = 1.0; the algorithm
%                                      has no confidence score)
%   Boxes  – K x 4 double array       ([x y width height] per cell,
%                                      regionprops BoundingBox convention)
%
%   Inputs
%   ------
%   I_gray   : 2-D uint8 or double grayscale image (converted by caller)
%   sigma    : Gaussian blur sigma (positive scalar); larger values smooth
%              more aggressively before edge detection
%   DiskSize : Morphological disk radius (positive integer); correlates
%              with single-cell size in pixels — see Sobel + Watershed
%              report
%   polarity : 'bright' or 'dark' (cell brightness relative to background)
%
%   Pipeline
%   --------
%   1. Gaussian smoothing       (frequency domain)
%   2. Sobel edge detection     (frequency domain)
%   3. Iterative thresholding   (binarise gradient magnitude)
%   4. Adaptive foreground mask (imbinarize + morphological reconstruction)
%   5. Marker extraction        (distance transform + open-close by
%                                reconstruction)
%   6. Marker-controlled watershed
%   7. Background basin removal (pixel-fraction threshold)
%   8. Pack outputs             (label matrix + bounding boxes)
%
%   Dependencies: Image Processing Toolbox (v25.2)

    [M, N] = size(I_gray);
    I_gray = im2double(I_gray);   % work in double throughout

    % ------------------------------------------------------------------ %
    %  1. Gaussian smoothing in the frequency domain                       %
    % ------------------------------------------------------------------ %
    gausksize    = 2*ceil(3*sigma) + 1; % Using 3-sigma rule
    gaussKernel  = fspecial('gaussian', [gausksize gausksize], sigma);

    % Zero-pad Gaussian kernel to image size
    Pad_g = zeros(M, N);
    Pad_g(1:gausksize, 1:gausksize) = gaussKernel;
    % Shift kernel so its centre lands at (1,1) before taking the FFT
    Gkernel_final = circshift(Pad_g, [-(floor(gausksize/2)) -(floor(gausksize/2))]);

    % ------------------------------------------------------------------ %
    %  2. Sobel edge detection in the frequency domain                     %
    % ------------------------------------------------------------------ %
    % Sobel kernels
    sobelX = [-1 0 1; -2 0 2; -1 0 1];
    sobelY = [ 1 2 1;  0 0 0; -1 -2 -1];

    % Zero-pad Sobel kernels and insert Sobel operators
    Pad_x = zeros(M, N); Pad_x(1:3, 1:3) = sobelX;
    Pad_y = zeros(M, N); Pad_y(1:3, 1:3) = sobelY;

    % Shift kernels to center around (1, 1)
    Xkernel_final = circshift(Pad_x, [-1 -1]);
    Ykernel_final = circshift(Pad_y, [-1 -1]);

    % Fourier transform into frequency domain
    F              = fft2(I_gray);
    Gkernel_F      = fft2(Gkernel_final);
    Xkernel_final_F = fft2(Xkernel_final);
    Ykernel_final_F = fft2(Ykernel_final);

    % Gradient calculations
    F_smooth    = F .* Gkernel_F;
    XGradient   = real(ifft2(F_smooth .* Xkernel_final_F));
    YGradient   = real(ifft2(F_smooth .* Ykernel_final_F));
    G           = sqrt(XGradient.^2 + YGradient.^2);

    % ------------------------------------------------------------------ %
    %  3. Iterative threshold on gradient magnitude                        %
    % ------------------------------------------------------------------ %
    Gvec  = G(:);
    T_old = mean(Gvec); % Starting threshold
    tolerance = 1e-5;

    while true
        % Separate into classes
        C_L   = Gvec(Gvec <  T_old);
        C_H   = Gvec(Gvec >= T_old);
        mu_L  = mean(C_L);
        mu_H  = mean(C_H);
        % Create new threshold
        T_new = (mu_L + mu_H) / 2;
        if abs(T_new - T_old) < tolerance, break; end
        T_old = T_new;
    end
    T = T_old; % Final threshold

    % ------------------------------------------------------------------ %
    %  4. Adaptive foreground mask                                        %
    % ------------------------------------------------------------------ %
    edgeMask   = G >= T;
    foreground = imbinarize(I_gray, 'adaptive', ...
                            'ForegroundPolarity', polarity, ...
                            'Sensitivity', 0.3);

    % Opening-by-reconstruction (remove speckle)
    se_open    = strel('disk', 1);
    foreground = imreconstruct(imerode(foreground, se_open), foreground);

    % Closing-by-reconstruction (fill small holes)
    ForeDil    = imdilate(foreground, strel('disk', 1));
    foreground = imcomplement(imreconstruct(imcomplement(ForeDil), imcomplement(foreground)));

    % ------------------------------------------------------------------ %
    %  5. Marker creation via distance transform                           %
    % ------------------------------------------------------------------ %
    % Confine watershed basins to the foreground
    edgeMask = edgeMask | ~foreground;

    D     = bwdist(edgeMask);
    se    = strel('disk', DiskSize);

    D_E     = imreconstruct(imerode(D, se), D);
    D_Dil   = imreconstruct(imcomplement(imdilate(D_E, se)), imcomplement(D_E));
    D_final = imcomplement(D_Dil);

    % Extract markers
    markers = imregionalmax(D_final);

    % Drop the smallest 5% by area of marker components (likely noise)
    stats   = regionprops(markers, 'Area');
    if ~isempty(stats)
        minArea = prctile([stats.Area], 5);
        markers = bwareaopen(markers, round(minArea));
    end

    markers = imfill(imclose(markers, strel('disk', 2)), 'holes');

    % ------------------------------------------------------------------ %
    %  6. Marker-controlled watershed                                      %
    % ------------------------------------------------------------------ %
    G_mod = imimposemin(G, markers);
    W     = watershed(G_mod);

    % ------------------------------------------------------------------ %
    %  7. Background basin classification                                  %
    %     Basins where >89 % of pixels fall outside foreground -> background
    % ------------------------------------------------------------------ %
    basinLabels = unique(W(W > 0));

    % Vectorised background classification — avoids a per-basin logical
    % array loop that would be slow for images with many basins.
    % totalPx  : number of pixels in each basin (indexed by label value)
    % outsidePx: number of those pixels that lie outside the foreground
    % Basins where more than 89% of pixels are outside the foreground
    % are classified as background and excluded from the output.
    W_flat     = W(:);
    fg_flat    = foreground(:);
    totalPx    = accumarray(W_flat(W_flat>0), 1,                  [max(basinLabels) 1]);
    outsidePx  = accumarray(W_flat(W_flat>0), ~fg_flat(W_flat>0), [max(basinLabels) 1]);
    bgFraction = outsidePx ./ max(totalPx, 1);
    fgLabels   = basinLabels(bgFraction(basinLabels) <= 0.89);

    % ------------------------------------------------------------------ %
    %  8. Convert labelled regions -> Masks / Labels / Scores / Boxes     %
    % ------------------------------------------------------------------ %
    K = numel(fgLabels);
    
    if K == 0
        Masks  = false(M, N, 0);
        Labels = categorical([], [], {'cell'});
        Scores = zeros(0, 1);
        Boxes  = zeros(0, 4);
        return;
    end
    
    allProps = regionprops(W, 'BoundingBox');
    Scores   = ones(K, 1);
    Boxes    = zeros(K, 4);
    for k = 1:K
        lbl        = fgLabels(k);
        Boxes(k,:) = allProps(lbl).BoundingBox;
    end
    
    % Masks are built per-cell in the app Segment callback to avoid
    % allocating M×N×K. Return the watershed label matrix as a uint32
    % matrix — the app's SW branch reads fgLabels from it via unique().
    Masks  = uint32(W) .* uint32(ismember(W, fgLabels));
    Labels = repmat(categorical({'cell'}), K, 1);
end