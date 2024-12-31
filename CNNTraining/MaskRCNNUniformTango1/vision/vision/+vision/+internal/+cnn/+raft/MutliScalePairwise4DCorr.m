classdef MutliScalePairwise4DCorr
% Constructs the 4D "cost volume" between two images - a multi-scale
% similarity or correlation between the features at every pixel location
% in the two input images.

% Copyright 2021-2024 The MathWorks, Inc.

    % Correlation properties
    properties (SetAccess = private)
        CorrelationLevels  % number of levels in the scale pyramid
        CorrelationPyramid % correlation values at each pyramid level
    end

    % Feature map size properties
    properties (Access = private)
        H % feature map height
        W % feature map width
        C % num channels
        B % batch size
    end

    % Sampling properties
    properties (Access = private)
        Neighbors % neighbor set for update search 
        FeatureSize % total number of extracted features = H*W for fmap
        Interpolants % interpolants for correlation sampling
        ZCoordinates % dummy sampling coordinates on c*b dim
    end

    methods
        % Constructor
        function obj = MutliScalePairwise4DCorr(fmap1, fmap2, correlationLevels, correlationRadius)
            % This constructs the correlation pyramid
            arguments
                fmap1 % feature map for img1
                fmap2 % feature map for img2
                correlationLevels = 4 % number of levels
                correlationRadius = 4 % correlation radius
            end

            obj.CorrelationLevels = correlationLevels;
            [obj.H, obj.W, obj.C, obj.B] = size(fmap1);
            obj.FeatureSize = obj.H * obj.W;

            % Create the searching neighbor set for flow update
            R = 2 * correlationRadius + 1; % total length R
            dxy = -correlationRadius:correlationRadius;
            [X,Y] = meshgrid(dxy,dxy);
            delta = cat(3,Y,X);
            obj.Neighbors = permute(delta,[1 2 4 3]); % size [RR12] to match sampling dimension

            % Compute correlation pyramids
            obj.CorrelationPyramid = cell(1,obj.CorrelationLevels);

            corr = obj.baseCorrelation(fmap1, fmap2);

            % Padding the image to match the boundary difference between
            % Python and MATLAB sampling (g3184771).
            obj.CorrelationPyramid{1} = padarray(extractdata(corr),[1 1]);

            % Get 2x lower resolution levels
            for i = 2:obj.CorrelationLevels
                corr = avgpool(corr,2,'stride',2,'DataFormat','SSCB');
                obj.CorrelationPyramid{i} = padarray(extractdata(corr),[1 1]);
            end

            % Store reused dummy coordinates for w1*h1*batch dimensions
            obj.ZCoordinates = repmat(permute(single(1:obj.FeatureSize*obj.B),[1 3 2]),[R,R,1]);

            % Store interpolants (g3166695)
            % obj = createInterpolants(obj);
        end

        function obj = createInterpolants(obj)
            obj.Interpolants = cell(1,obj.CorrelationLevels);
            for i = 1:obj.CorrelationLevels
                img = obj.CorrelationPyramid{i};

                [hMap, wMap] = size(img,[1 2]); % H, W is +2 for the padding

                % the Value in Interpolants can be changes for new maps
                obj.Interpolants{i} = griddedInterpolant({-1:hMap-2, -1:wMap-2, 1:obj.FeatureSize*obj.B}, img,'linear','none');
            end
        end

        function out = sample(obj, coords)
            % This computes the correlation values on the sample
            % coordinates.
            % - coords is [h1,w1,2,b], new pixel locations in fmap2 for each
            % pixel in fmap1
            
            % 1st coords is not dlarray, the ones after are.
            if isdlarray(coords)
                coords = extractdata(coords);
            end
            
            % Prepare for sampling shapes
            coords = permute(coords, [2, 1, 4, 3]);% [w,h,b,2]

            outPyramid = cell(1, obj.CorrelationLevels);

            for i = 1:obj.CorrelationLevels
                % Add neighbor set to the centroid pixels
                centroidPixels = reshape(coords, [1, 1, obj.W * obj.H * obj.B, 2]) / 2^(i-1);
                allPixels = centroidPixels + obj.Neighbors; % [RRC2]

                % Sample correlation values
                corr = obj.bilinearSamplerInterp3(obj.CorrelationPyramid{i}, ...
                    allPixels); % [h1,w1,R*R,b]
                % corr = obj.bilinearSampler(i, allPixels); (g3166695)

                outPyramid{i} = corr;
            end

            % Cat on the C dimension
            out = cat(3, outPyramid{:});
        end
    end

    methods (Access = private)
        function corrOut = baseCorrelation(obj, fmap1, fmap2)
            % This computes the base correlation map
            % - fmap is [HWCB], c = # of features

            % Flatten along the feature dimensions HW.
            % - need permute to match python order
            fmap1 = permute(fmap1,[2 1 3 4]);
            fmap2 = permute(fmap2,[2 1 3 4]);
            fmap1 = reshape(fmap1, [obj.W * obj.H, obj.C, obj.B]);
            fmap2 = reshape(fmap2, [obj.W * obj.H, obj.C, obj.B]);

            % Compute correlation for the whole batch
            corrOut = pagemtimes(fmap2,'none',fmap1,'transpose');% [w2*h2,w1*h1,b]

            % Separate fmap2 spatial dimensions and combine fmap1 spatial
            % dimensions with batch dimension.
            % - The reason is that the query points will be from fmap2
            corrOut = reshape(corrOut, [obj.W, obj.H, obj.W*obj.H*obj.B]);% [w2, h2, w1*h1*b]

            % Normalize 
            corrOut = corrOut / sqrt(obj.C);
        end

        function mapOut = bilinearSamplerInterp3(obj, map, coords)
            % This samples from map using interp3
            % - map is [w2,h2,w1*h1*b]
            % - coords is [R,R,w1*h1*b,2]
            [hMap, wMap] = size(map,[2 1]); % [H, W] is [h2, w2] +2 for the padding
        
            % Separate coordinates for x and y
            x = coords(:,:,:,1);
            y = coords(:,:,:,2);
        
            % Sample
            mapOut = interp3(-1:hMap-2, -1:wMap-2, 1:obj.FeatureSize*obj.B, map, y,x,obj.ZCoordinates,'linear',0); % [R,R,w1*h1*b]
            
            % Reshape it back to HWCB format
            mapOut = permute(mapOut,[3 2 1]);% [w1*h1*b,R,R]
            mapOut = reshape(mapOut, obj.W, obj.H, obj.B, []);% [w1,h1,b,R*R]
            mapOut = permute(mapOut, [2, 1, 4, 3]);% [h1,w1,R*R,b]
        end

        function mapOut = bilinearSampler(obj, i, coords)
            F = obj.Interpolants{i};
        
            % Separate coordinates for x and y
            x = coords(:,:,:,1);
            y = coords(:,:,:,2);
        
            mapOut = F(x,y,obj.ZCoordinates);

            % set all extrapolated nan to zero
            mapOut(isnan(mapOut)) = 0;
            
            % Reshape it back to HWCB format
            mapOut = permute(mapOut,[3 2 1]);% [w1*h1*b,R,R]
            mapOut = reshape(mapOut, obj.W, obj.H, obj.B, []);% [w1,h1,b,R*R]
            mapOut = permute(mapOut, [2, 1, 4, 3]);% [h1,w1,R*R,b]
        end
    end
end
