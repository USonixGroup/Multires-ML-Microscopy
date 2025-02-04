classdef acfObjectDetectorBuildable < coder.ExternalDependency %#codegen
    %  acfObjectDetectorBuildable - encapsulate acfObjectDetector implementation library
    %  This function is used by detect function of acfObjectDetector class

%   Copyright 2018-2022 The MathWorks, Inc.

    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'acfObjectDetectorBuildable';
        end
        
        function b = isSupportedContext(~)
            b = true; % supports non-host target
        end
        
        %--------------------------------------------------------------------------
        % Generate Hash table used to convert RGB to LUV color Space
        % to avoid cube root computation
        %--------------------------------------------------------------------------
        function yTable = initHashTable()
            
            yTable = zeros(1,1064,'single');
            y0 = single((6.0/29)*(6.0/29)*(6.0/29));
            a =  single((29.0/3)*(29.0/3)*(29.0/3));
            maxi = single(1.0/270);
            
            for i = 0:1024
                y = single(i/1024.0);
                if y > y0
                    l = 116 * y^(1/3) - 16;
                else
                    l =  y * a;
                end
                yTable(i+1) = l*maxi;
            end
            
            for i = 1025:1063
                yTable(i+1) = yTable(i);
            end
            
        end
        
        function updateBuildInfo(buildInfo, context)
            vision.internal.buildable.cvstBuildInfo(buildInfo, context, ...
                'acfObjectDetector', ...
                {'use_tbb'});
        end
        %------------------------------------------------------------------
        % write all supported data-type specific function calls
        %------------------------------------------------------------------
        function [luv] = rgb2luv(I)
            
            coder.inline('always');
            coder.cinclude('cvstCG_acfObjectDetector.h');
            
            % create uninitialized memory
            luv = coder.nullcopy(I);
            
            % Generating the yTable
            persistent yTable;
            if isempty(yTable)
                yTable = vision.internal.buildable.acfObjectDetectorBuildable.initHashTable;
            end
            
            nPixelsPerChannel = uint64((size(I,1))*(size(I,2)));
            nYTable = uint64(size(yTable, 2));
            
            if coder.isColumnMajor
                % calling core c++ function to convert rgb to luv image
                coder.ceval('-col', 'rgb2luv', coder.ref(I), coder.ref(luv),...
                    (yTable), nPixelsPerChannel, nYTable);
            else
                
                I_temp = I(:);
                luv_temp = coder.nullcopy(zeros(size(I_temp,1),1,'like',I_temp));
                coder.ceval('-row', 'rgb2luv', I_temp, coder.ref(luv_temp),...
                    (yTable'), nPixelsPerChannel, nYTable);
                if (~isempty(luv_temp))
                    luv = reshape(luv_temp', size(I));
                else
                    luv = luv_temp;
                end
                
            end
        end
        
        %------------------------------------------------------------------
        % write all supported data-type specific function calls
        %------------------------------------------------------------------
        function [J] = convTri(I, r, s)
            
            coder.inline('always');
            coder.cinclude('cvstCG_acfObjectDetector.h');
            
            % create uninitialized memory using coder.nullcopy
            J = coder.nullcopy(zeros(size(I), 'single'));
            
            % Initialsing the variables for core c++ function
            h = size(I,1);
            w = size(I,2);
            d = size(I,3);
            p = r;
            
            if (nargin < 2 || isempty(r))
                r = 3;
            end
            if (nargin < 3 || isempty(s))
                s=1;
            end
            
            if (isempty(I) || (r == 0 && s == 1))
                J = I;
                return;
            end
            
            % calling core c++ function to convolve with traingular
            % waveform
            isConvTri = true;
            if (r > 0 && r <= 1 && s <= 2)
                isConvTri = false;
            end
            
            if isConvTri
                if(mod(h,4) == 0)
                    h0 = h;
                    h1 = h;
                else
                    h0 = h - (mod(h,4));
                    h1 = h0 + 4;
                end
                if coder.isColumnMajor
                    T = coder.nullcopy(zeros(1, h1*2, 'single'));
                    coder.ceval('-col','convTri',coder.ref(I),  coder.ref(J),...
                        int16(h), int16(w), int16(d), int16(r), int16(s), coder.ref(T), int16(h0), int16(h1));
                else
                    T_t = coder.nullcopy(zeros( h1*2,1, 'single'));
                    I_temp = I(:);
                    J_temp = coder.nullcopy(zeros(size(I_temp,1),1,'like',I_temp));
                    coder.ceval('-row','convTri',I_temp,  coder.ref(J_temp),...
                        int16(h), int16(w), int16(d), int16(r), int16(s), coder.ref(T_t), int16(h0), int16(h1));
                    J = reshape(J_temp', size(I));
                end
            else
                p = 12/p/(p+2)-2;
                if coder.isColumnMajor
                    T = coder.nullcopy(zeros(1,h, 'single'));
                    coder.ceval('-col','convTri1', coder.ref(I),  coder.ref(J),...
                        int16(h), int16(w), int16(d), single(p), int16(s), coder.ref(T));
                else
                    T_t = coder.nullcopy(zeros(h,1, 'single'));
                    I_temp = I(:);
                    J_temp = coder.nullcopy(zeros(size(I_temp,1),1,'like',I_temp));
                    coder.ceval('-row','convTri1', I_temp,  coder.ref(J_temp),...
                        int16(h), int16(w), int16(d), single(p), int16(s), coder.ref(T_t));
                    J = reshape(J_temp', size(I));
                end
            end
            
        end
        
        %------------------------------------------------------------------
        % write all supported data-type specific function calls
        %------------------------------------------------------------------
        function [grM, grO] = gradient(I, params)
            
            coder.inline('always');
            coder.cinclude('cvstCG_acfObjectDetector.h');
            
            normRad   = params.NormalizationRadius;
            normConst = params.NormalizationConstant;
            fullOrient = params.FullOrientation;
            useSSE = true;
            
            % Check inputs
            if(nargin < 1 || isempty(I))
                grM = single(zeros(size(I,1), size(I,2), 'single'));
                grO = grM;
                return;
            end
            
            if max(max(I)) > 1
                I = im2single(I);
            end
            
            % Initialising the variables for core c++ funtion
            h = size(I,1);
            w = size(I,2);
            d = size(I,3);
            
            % create uninitialized memory using coder.nullcopy
            grM = zeros(h, w, 'single');
            grO = zeros(h, w, 'single');
            
            % calling the core c++ function to calcule gradient
            if useSSE
                if(mod(h,(4)) == 0)
                    hsse = h;
                else
                    hsse = h - (mod(h,(4))) + (4);
                end
                if coder.isColumnMajor
                    imgSize = hsse * d * w;
                    M = coder.nullcopy(zeros(1, imgSize, 'single'));
                    Gx = coder.nullcopy(zeros(1, imgSize, 'single'));
                    Gy = coder.nullcopy(zeros(1, imgSize, 'single'));
                    
                    coder.ceval('-col','gradientSSE',...
                        coder.ref(I), coder.ref(grM), coder.ref(grO),...
                        uint64(h), uint64(w), uint64(d), uint64(hsse), coder.ref(M), coder.ref(Gx), coder.ref(Gy));
                else
                    imgSize = hsse * d * w;
                    M = coder.nullcopy(zeros(imgSize, 1, 'single'));
                    Gx = coder.nullcopy(zeros(imgSize, 1, 'single'));
                    Gy = coder.nullcopy(zeros(imgSize, 1, 'single'));
                    I_temp = I(:);
                    grM_temp = coder.nullcopy(zeros(size(I_temp,1)/3,1,'like',I_temp));
                    grO_temp = coder.nullcopy(zeros(size(I_temp,1)/3,1,'like',I_temp));
                    
                    coder.ceval('-row','gradientSSE',...
                        I_temp, coder.ref(grM_temp), coder.ref(grO_temp),...
                        uint64(h), uint64(w), uint64(d), uint64(hsse), coder.ref(M), coder.ref(Gx), coder.ref(Gy));
                    grM = reshape(grM_temp', [h w]);
                    grO = reshape(grO_temp', [h w]);
                end
            else
                if coder.isColumnMajor
                    imgSize = h * d;
                    M = coder.nullcopy(zeros(1, imgSize, 'single'));
                    Gx = coder.nullcopy(zeros(1, imgSize, 'single'));
                    Gy = coder.nullcopy(zeros(1, imgSize, 'single'));
                    
                    coder.ceval('-col','gradient',...
                        coder.ref(I), coder.ref(grM), coder.ref(grO),...
                        uint64(h), uint64(w), uint64(d), coder.ref(M), coder.ref(Gx), coder.ref(Gy));
                else
                    imgSize = h * d;
                    M = coder.nullcopy(zeros(imgSize, 1, 'single'));
                    Gx = coder.nullcopy(zeros(imgSize, 1, 'single'));
                    Gy = coder.nullcopy(zeros(imgSize, 1, 'single'));
                    I_temp = I(:);
                    grM_temp = coder.nullcopy(zeros(size(I_temp,1)/3, 1, 'like', I_temp));
                    grO_temp = coder.nullcopy(zeros(size(I_temp,1)/3, 1, 'like', I_temp));
                    
                    coder.ceval('-row','gradient',...
                        I_temp, coder.ref(grM_temp), coder.ref(grO_temp),...
                        uint64(h), uint64(w), uint64(d), coder.ref(M), coder.ref(Gx), coder.ref(Gy));
                    grM = reshape(grM_temp', [h w]);
                    grO = reshape(grO_temp', [h w]);
                end
            end
            
            if (fullOrient == 0)
                for  n = 1: h * w
                    if (grO(n) < 0)
                        grO(n) = grO(n) + pi;
                    end
                end
                
            else
                % orientation between [0 2*pi]
                twoPi = pi * 2;
                for  n = 1: h * w
                    if (grO(n) < 0)
                        grO(n) = grO(n) + twoPi;
                    end
                end
            end
            
            if normRad > 0
                S = vision.internal.buildable.acfObjectDetectorBuildable.convTri(grM, normRad, 1);
                grM = grM ./ (S + normConst);
            end
            
        end
        
        %------------------------------------------------------------------
        % write all supported data-type specific function calls
        %------------------------------------------------------------------
        function [data_out] = resize(data, h_out, w_out, norm)
            
            coder.inline('always');
            coder.cinclude('cvstCG_acfObjectDetector.h');
            
            % Initialising the variables
            h =  int32(size(data, 1));
            w = int32(size(data, 2));
            d = int32(size(data, 3));
            
            % Calling the core c++ function to do resize of the data
            if isa(data,'single')
                % create uninitialized memory using coder.nullcopy to hold
                % the output of core c++ function
                if coder.isColumnMajor
                    data_res = coder.nullcopy(zeros(1, h_out * w_out * d, 'single'));
                    coder.ceval('-col','resample_float',...
                        coder.ref(data), coder.ref(data_res), h,...
                        h_out, w, w_out, d, norm);
                    data_out = reshape(data_res, [h_out w_out d]);
                else
                    data_res = coder.nullcopy(zeros(h_out * w_out * d, 1, 'single'));
                    data_temp = data(:);
                    coder.ceval('-row','resample_float',...
                        data_temp, coder.ref(data_res), h,...
                        h_out, w, w_out, d, norm);
                    data_out = reshape(data_res', [h_out w_out d]);
                    
                end
                
            elseif isa(data,'double')
                % create uninitialized memory using coder.nullcopy to hold
                % the output of core c++ function
                if coder.isColumnMajor
                    data_res = coder.nullcopy(zeros(1, h_out * w_out * d, 'double'));
                    coder.ceval('-col','resample_double',...
                        coder.ref(data), coder.ref(data_res), h,...
                        h_out, w, w_out, d, norm);
                    data_out = reshape(data_res, [h_out w_out d]);
                else
                    data_res = coder.nullcopy(zeros(h_out * w_out * d, 1, 'double'));
                    data_temp = data(:);
                    coder.ceval('-row','resample_double',...
                        data_temp, coder.ref(data_res), h,...
                        h_out, w, w_out, d, norm);
                    data_out = reshape(data_res', [h_out w_out d]);
                end
            else % uint8
                % create uninitialized memory using coder.nullcopy to hold
                % the output of core c++ function
                if coder.isColumnMajor
                    data_res = coder.nullcopy(zeros(1, h_out * w_out * d, 'single'));
                    input_data = single(data);
                    coder.ceval('-col','resample_float',...
                        coder.ref(input_data), coder.ref(data_res), h,...
                        h_out, w, w_out, d, norm);
                    data_out = reshape(uint8(data_res), [h_out w_out d]);
                else
                    data_res = coder.nullcopy(zeros(h_out * w_out * d, 1, 'single'));
                    input_data = single(data(:));
                    coder.ceval('-row','resample_float',...
                        (input_data), coder.ref(data_res), h,...
                        h_out, w, w_out, d, norm);
                    data_out = reshape(uint8(data_res'), [h_out w_out d]);
                end
                
            end
        end
        
        %------------------------------------------------------------------
        % write all supported data-type specific function calls
        %------------------------------------------------------------------
        function [ gradHist ] = gradientHist(gMag, gDir, params)
            
            coder.inline('always');
            coder.cinclude('cvstCG_acfObjectDetector.h');
            
            % Initialising the Variables
            cellSize    = params.CellSize;
            numBins     = params.NumBins;
            interpolate = params.Interpolation;
            full        = params.FullOrientation;
            
            % Check inputs
            if (nargin < 2 || isempty(gMag))
                gradHist = gMag;
                return;
            end
            
            % Check inputs Size
            [m, n] = size(gMag);
            m = single(m);
            n = single(n);
            cr = single(mod([m n], cellSize));
            
            if(any(cr))
                m   = m - cr(1);
                n   = n - cr(2);
                gMag = gMag(1:m, 1:n);
                gDir = gDir(1:m, 1:n);
            end
            
            % Initialsing the varaibles for core c++ functions
            h = int32(size(gMag, 1));
            w = int32(size(gMag, 2));
            
            numberOfRowCells = int32(h / cellSize);
            numberOfColumnCells = int32(w / cellSize);
            numberOfCellsPerBin = int32(numberOfRowCells * numberOfColumnCells);
            useSignedOrientation = int32(full) ;
            
            % create uninitialized memory using coder.nullcopy
            grHist = zeros(1, numberOfRowCells * numberOfColumnCells * numBins, 'single');
            gradHist = zeros([numberOfRowCells, numberOfColumnCells, numBins], 'single');
            
            % calling the core c++ function to calculate gradient histogram
            % with diffrent methods
            if(strcmp(interpolate,'None'))
                if coder.isColumnMajor
                    iO = coder.nullcopy(zeros(1, h, 'int32'));
                    iM = coder.nullcopy(zeros(1, h, 'single'));
                    coder.ceval('-col','noInterpolation',...
                        coder.ref(grHist), coder.ref(gMag), coder.ref(gDir),...
                        cellSize, h, w, numberOfRowCells, numberOfColumnCells,...
                        numberOfCellsPerBin, int32(numBins), useSignedOrientation, coder.ref(iO), coder.ref(iM));
                else
                    iO = coder.nullcopy(zeros(h, 1, 'int32'));
                    iM = coder.nullcopy(zeros(h, 1, 'single'));
                    grHist_temp = zeros(numberOfRowCells * numberOfColumnCells * numBins, 1, 'single');
                    gMag_temp = gMag';
                    gDir_temp = gDir';
                    coder.ceval('-row','noInterpolation',...
                        coder.ref(grHist_temp), gMag_temp, gDir_temp,...
                        int32(cellSize), h, w, numberOfRowCells, numberOfColumnCells,...
                        numberOfCellsPerBin, int32(numBins), useSignedOrientation, coder.ref(iO), coder.ref(iM));
                    grHist = grHist_temp';
                end
                
            elseif(strcmp(interpolate,'Orientation'))
                if coder.isColumnMajor
                    iO = coder.nullcopy(zeros(1, h, 'int32'));
                    iO2 = coder.nullcopy(zeros(1, h, 'int32'));
                    iM = coder.nullcopy(zeros(1, h, 'single'));
                    iM2 = coder.nullcopy(zeros(1, h, 'single'));
                    
                    coder.ceval('-col','orientationInterpolation',...
                        coder.ref(grHist), coder.ref(gMag), coder.ref(gDir),...
                        int32(cellSize), h, w, numberOfRowCells, numberOfColumnCells,...
                        numberOfCellsPerBin, int32(numBins), useSignedOrientation, coder.ref(iO), coder.ref(iM),...
                        coder.ref(iO2), coder.ref(iM2));
                else
                    iO = coder.nullcopy(zeros(h, 1, 'int32'));
                    iO2 = coder.nullcopy(zeros(h, 1, 'int32'));
                    iM = coder.nullcopy(zeros(h, 1, 'single'));
                    iM2 = coder.nullcopy(zeros(h, 1, 'single'));
                    grHist_temp = zeros(numberOfRowCells * numberOfColumnCells * numBins, 1, 'single');
                    gMag_temp = gMag';
                    gDir_temp = gDir';
                    
                    coder.ceval('-row','orientationInterpolation',...
                        coder.ref(grHist_temp), gMag_temp, gDir_temp,...
                        int32(cellSize), h, w, numberOfRowCells, numberOfColumnCells,...
                        numberOfCellsPerBin, int32(numBins), useSignedOrientation, coder.ref(iO), coder.ref(iM),...
                        coder.ref(iO2), coder.ref(iM2));
                    grHist = grHist_temp';
                    
                end
                
            elseif(strcmp(interpolate,'Spatial'))
                if coder.isColumnMajor
                    iO = (zeros(1, h, 'int32'));
                    iM = (zeros(1, h, 'single'));
                    
                    coder.ceval('-col','spatialInterpolation',...
                        coder.ref(grHist), coder.ref(gMag), coder.ref(gDir),...
                        int32(cellSize), h, w, numberOfRowCells, numberOfColumnCells,...
                        numberOfCellsPerBin, int32(numBins), useSignedOrientation, coder.ref(iO), coder.ref(iM));
                else
                    iO = coder.nullcopy(zeros(h, 1, 'int32'));
                    iM = coder.nullcopy(zeros(h, 1, 'single'));
                    grHist_temp = zeros(numberOfRowCells * numberOfColumnCells * numBins, 1, 'single');
                    gMag_temp = gMag';
                    gDir_temp = gDir';
                    
                    coder.ceval('-row','spatialInterpolation',...
                        coder.ref(grHist_temp), gMag_temp, gDir_temp,...
                        int32(cellSize), h, w, numberOfRowCells, numberOfColumnCells,...
                        numberOfCellsPerBin, int32(numBins), useSignedOrientation, coder.ref(iO), coder.ref(iM));
                    grHist = grHist_temp';
                end
                
            elseif(strcmp(interpolate,'Both'))
                if coder.isColumnMajor
                    iO = coder.nullcopy(zeros(1, h, 'int32'));
                    iO2 = coder.nullcopy(zeros(1, h, 'int32'));
                    iM = coder.nullcopy(zeros(1, h, 'single'));
                    iM2 = coder.nullcopy(zeros(1, h, 'single'));
                    
                    coder.ceval('-col','spatialOrientationInterpolation',...
                        coder.ref(grHist), coder.ref(gMag), coder.ref(gDir),...
                        int32(cellSize), h, w, numberOfRowCells, numberOfColumnCells,...
                        numberOfCellsPerBin, int32(numBins), useSignedOrientation, coder.ref(iO), coder.ref(iM),...
                        coder.ref(iO2), coder.ref(iM2));
                else
                    iO = coder.nullcopy(zeros(h, 1, 'int32'));
                    iO2 = coder.nullcopy(zeros(h, 1, 'int32'));
                    iM = coder.nullcopy(zeros(h, 1, 'single'));
                    iM2 = coder.nullcopy(zeros(h, 1, 'single'));
                    grHist_temp = zeros(numberOfRowCells * numberOfColumnCells * numBins, 1, 'single');
                    gMag_temp = gMag';
                    gDir_temp = gDir';
                    
                    coder.ceval('-row','spatialOrientationInterpolation',...
                        coder.ref(grHist_temp), gMag_temp, gDir_temp,...
                        int32(cellSize), h, w, numberOfRowCells, numberOfColumnCells,...
                        numberOfCellsPerBin, int32(numBins), useSignedOrientation, coder.ref(iO), coder.ref(iM),...
                        coder.ref(iO2), coder.ref(iM2));
                    grHist = grHist_temp';
                end
            else
                disp('Invalid string ');
                return;
            end
            gradHist = reshape(grHist,[numberOfRowCells numberOfColumnCells numBins]);
        end
        
        %------------------------------------------------------------------
        % write all supported data-type specific function calls
        %------------------------------------------------------------------
        function [bbox_out] = detector(P, model, params)
            
            coder.inline('always');
            coder.cinclude('cvstCG_acfObjectDetector.h');
            
            % Initialising the variables for core c++ functions
            chns = P;
            shrink = params.Shrink;
            modelHt = params.ModelSizePadded(1);
            modelWd = params.ModelSizePadded(2);
            stride = params.WindowStride;
            cascThr = params.Threshold;
            
            thrs = model.thrs;
            hs = model.hs;
            fids = model.fids;
            child = (model.child);
            pm = model.treeDepth;
            
            if isempty(pm)
                treeDepth = 0;
            else
                treeDepth = pm;
            end
            
            chnsSize = size(P);
            height = chnsSize(1);
            width = chnsSize(2);
            
            if chnsSize <= 2
                nChns = 1;
            else
                nChns = chnsSize(3);
            end
            
            fidsSize = size(model.fids);
            nTreeNodes = fidsSize(1);
            nTrees = fidsSize(2);
            height1 = ceil(fix(height * shrink - modelHt + 1) / stride);
            width1 = ceil(fix(width * shrink - modelWd + 1) / stride);
            
            % Initialsing the variable to hold the number of bounding boxes
            % as the output of core c++ functions
            m = int32(0);
            
            % Initialising the variables for the core c++ functions
            cs = coder.opaquePtr('void', coder.internal.null);
            rs = coder.opaquePtr('void', coder.internal.null);
            hs1 = coder.opaquePtr('void', coder.internal.null);
            flag = coder.opaquePtr('unsigned char', coder.internal.null);
            
            % Calling the core  c++ function to get the number of bounding
            % boxes
            if coder.isColumnMajor
                m = coder.ceval('-col','getNumberOfBoundingBoxes',...
                    coder.ref(chns), coder.ref(thrs),...
                    coder.ref(hs), coder.ref(fids), coder.ref(child), int32(shrink), int32(modelHt),...
                    int32(modelWd),int32(stride), single(cascThr), int32(height), int32(width), int32(nChns), int32(nTreeNodes),...
                    int32(nTrees), int32(height1), int32(width1), int32(treeDepth), coder.ref(rs), coder.ref(cs), coder.ref(hs1), (flag));
            else
                chns_temp = chns(:);
                thrs_temp = thrs(:);
                hs_temp = hs(:);
                fids_temp = fids(:);
                child_temp = child(:);
                
                m = coder.ceval('-row','getNumberOfBoundingBoxes',...
                    chns_temp, thrs_temp,...
                    hs_temp, fids_temp, child_temp, int32(shrink), int32(modelHt),...
                    int32(modelWd),int32(stride), single(cascThr), int32(height), int32(width), int32(nChns), int32(nTreeNodes),...
                    int32(nTrees), int32(height1), int32(width1), int32(treeDepth), coder.ref(rs), coder.ref(cs), coder.ref(hs1), (flag));
            end
            
            % Calling the c++ function to copy the bounding boxes
            if (m>0)
                % Initialsing the variable to hold the bounding boxes
                % as the output of core c++ functions
                
                if coder.isColumnMajor
                    bbox = coder.nullcopy(zeros(1,m * 5, 'double'));
                    coder.ceval('-col', 'copyBoundingBox', rs, cs, hs1, coder.ref(bbox), int32(stride), int32(modelHt) ,...
                        int32(modelWd), m);
                    bbox_out = reshape(bbox,[m 5]);
                    
                else
                    bbox = coder.nullcopy(zeros(m * 5, 1, 'double'));
                    coder.ceval('-row', 'copyBoundingBox', rs', cs', hs1', coder.ref(bbox), int32(stride), int32(modelHt) ,...
                        int32(modelWd), m);
                    bbox_out = reshape(bbox',[m 5]);
                end
            else
                bbox_out = (zeros(1,5,'double'));
            end
        end
        %------------------------------------------------------------------
        % write all supported data-type specific function calls
        %------------------------------------------------------------------
        function [bbox_out] = detector_flag(P, model, params, flag)
            
            coder.inline('always');
            coder.cinclude('cvstCG_acfObjectDetector.h');
            
            % Initialising the variables for core c++ functions
            chns = P;
            shrink = params.Shrink;
            modelHt = params.ModelSizePadded(1);
            modelWd = params.ModelSizePadded(2);
            stride = params.WindowStride;
            cascThr = params.Threshold;
            
            thrs = model.thrs;
            hs = model.hs;
            fids = model.fids;
            child = (model.child);
            pm = model.treeDepth;
            
            if isempty(pm)
                treeDepth = 0;
            else
                treeDepth = pm;
            end
            
            chnsSize = size(P);
            height = chnsSize(1);
            width = chnsSize(2);
            
            if chnsSize <= 2
                nChns = 1;
            else
                nChns = chnsSize(3);
            end
            
            fidsSize = size(model.fids);
            nTreeNodes = fidsSize(1);
            nTrees = fidsSize(2);
            height1 = ceil(fix(height * shrink - modelHt + 1) / stride);
            width1 = ceil(fix(width * shrink - modelWd + 1) / stride);
            
            % Initialsing the variable to hold the number of bounding boxes
            % as the output of core c++ functions
            m = int32(0);
            
            % Initialising the variables for the core c++ functions
            cs = coder.opaquePtr('void', coder.internal.null);
            rs = coder.opaquePtr('void', coder.internal.null);
            hs1 = coder.opaquePtr('void', coder.internal.null);
            
            % Calling the core  c++ function to get the number of bounding
            % boxes
            if coder.isColumnMajor
                m = coder.ceval('-col','getNumberOfBoundingBoxes',...
                    coder.ref(chns), coder.ref(thrs),...
                    coder.ref(hs), coder.ref(fids), coder.ref(child), int32(shrink), int32(modelHt),...
                    int32(modelWd),int32(stride), single(cascThr), int32(height), int32(width), int32(nChns), int32(nTreeNodes),...
                    int32(nTrees), int32(height1), int32(width1), int32(treeDepth), coder.ref(rs), coder.ref(cs),...
                    coder.ref(hs1),coder.ref(flag));
            else
                chns_temp = chns(:);
                thrs_temp = thrs(:);
                hs_temp = hs(:);
                fids_temp = fids(:);
                child_temp = child(:);
                
                m = coder.ceval('-row','getNumberOfBoundingBoxes',...
                    chns_temp, thrs_temp,...
                    hs_temp, fids_temp, child_temp, int32(shrink), int32(modelHt),...
                    int32(modelWd),int32(stride), single(cascThr), int32(height), int32(width), int32(nChns), int32(nTreeNodes),...
                    int32(nTrees), int32(height1), int32(width1), int32(treeDepth), coder.ref(rs), coder.ref(cs), coder.ref(hs1), (flag'));
            end
            
            % Calling the c++ function to copy the bounding boxes
            if (m>0)
                % Initialsing the variable to hold the bounding boxes
                % as the output of core c++ functions
                if coder.isColumnMajor
                    bbox = coder.nullcopy(zeros(1,m * 5, 'double'));
                    coder.ceval('-col', 'copyBoundingBox', rs, cs, hs1, coder.ref(bbox), int32(stride), int32(modelHt) ,...
                        int32(modelWd), m);
                    bbox_out = reshape(bbox,[m 5]);
                else
                    bbox = coder.nullcopy(zeros(m * 5, 1, 'double'));
                    coder.ceval('-row', 'copyBoundingBox', rs', cs', hs1', coder.ref(bbox), int32(stride), int32(modelHt) ,...
                        int32(modelWd), m);
                    bbox_out = reshape(bbox',[m 5]);
                end
            else
                bbox_out = (zeros(1,5,'double'));
            end
        end
    end
end
