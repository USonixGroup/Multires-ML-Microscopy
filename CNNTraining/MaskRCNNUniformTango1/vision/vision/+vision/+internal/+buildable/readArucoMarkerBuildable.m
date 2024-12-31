classdef readArucoMarkerBuildable < coder.ExternalDependency %#codegen
    %

    % Copyright 2024 The MathWorks, Inc.

    methods (Static)

        function name = getDescriptiveName(~)
            name = 'readArucoMarker';
        end

        function b = isSupportedContext(~)
            b = true; % supports non-host target
        end

        function updateBuildInfo(buildInfo, context)

            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','ocv','include'), ...
                fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','ocv','include','aruco'),...
                fullfile(matlabroot,'toolbox', ...
                'images','opencv','opencvcg', 'include')} );

            srcPaths = repmat({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','ocv')}, [1 2]);
            buildInfo.addSourceFiles({'readArucoMarkerCore.cpp', ...
                'cgCommon.cpp'},srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');

            srcPaths1 = repmat({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','ocv','aruco')}, [1 2]);

            buildInfo.addSourceFiles({'readArucoImpl.cpp', ...
                'arucoDictionary.cpp'},srcPaths1);
            buildInfo.addSourcePaths(srcPaths1);

            buildInfo.addIncludeFiles({'vision_defines.h', ...
                'readArucoMarkerCore_api.hpp', ...
                'readArucoImpl.hpp', ...
                'cgCommon.hpp','arucoDictionary.hpp'});

            vision.internal.buildable.portableOpenCVBuildInfo(buildInfo, context, ...
                'readArucoMarker');
        end

        %------------------------------------------------------------------
        % write all supported data-type specific function calls
        function [ids, locs, detectedFamily, rejections, varargout] = readAruco(I, markerFamily, detectorParams,...
                varargin)

            coder.inline('always');
            coder.cinclude('readArucoMarkerCore_api.hpp');

            % Image width and height
            nRows = int32(size(I,1));
            nCols = int32(size(I,2));
            isRGB = size(I,3) == 3;
            
            if iscellstr(markerFamily)

                % Number of markers input by the user and their lengths
                numMarkers = int32(length(markerFamily));                
                markersLength = int32(strlength(markerFamily));
                if numMarkers == 1
                    markersLength = int32([strlength(markerFamily) 0]);
                end
                % Combine all the markers into a single character vector,
                % if more than one format is provided.
                % The cellstr does not null terminate strings,
                % so nullcharacter is explicitly added
                nullCharacter = 0;
                markersProc = [strjoin(markerFamily,'') nullCharacter];

            else
                % If markers is provided as a character vector (single
                % marker)
                numMarkers = int32(1);
                markersLength = int32([strlength(markerFamily) 0]);
                markersProc = [markerFamily 0];
            end           
            
            if nargin > 3
                doEstimatePose =  true;
                markerSize = varargin{1};
                camMatrix = varargin{2};
                distCoeffs = varargin{3};
            else
                doEstimatePose = false;
                markerSize = 0;
                camMatrix = zeros(3,3);
                distCoeffs = zeros(1,5);
            end

            params = struct('adaptiveThreshWinSizeMin',int32(detectorParams.AdaptiveThreshWinSizeMin),...
                            'adaptiveThreshWinSizeMax',int32(detectorParams.AdaptiveThreshWinSizeMax),...
                            'adaptiveThreshWinSizeStep', int32(detectorParams.AdaptiveThreshWinSizeStep),...
                            'adaptiveThreshConstant', detectorParams.AdaptiveThreshConstant,...                             
                            'minMarkerPerimeterRate', detectorParams.MinMarkerPerimeterRate,...
                            'maxMarkerPerimeterRate', detectorParams.MaxMarkerPerimeterRate,...
                            'polygonalApproxAccuracyRate', detectorParams.PolygonalApproxAccuracyRate,...
                            'minCornerDistanceRate', detectorParams.MinCornerDistanceRate,...
                            'minDistanceToBorder', int32(detectorParams.MinDistanceToBorder),...
                            'minMarkerDistanceRate', detectorParams.MinMarkerDistanceRate,...
                            'perspectiveRemovePixelPerCell', int32(detectorParams.PerspectiveRemovePixelPerCell),...
                            'perspectiveRemoveIgnoredMarginPerCell', detectorParams.PerspectiveRemoveIgnoredMarginPerCell,...                            
                            'markerBorderBits', int32(detectorParams.MarkerBorderBits),...
                            'minOtsuStdDev', detectorParams.MinOtsuStdDev,...  
                            'maxErroneousBitsInBorderRate', detectorParams.MaxErroneousBitsInBorderRate,...
                            'errorCorrectionRate', detectorParams.ErrorCorrectionRate,...   
                            'detectInvertedMarker', detectorParams.DetectInvertedMarker,...
                            'useAruco3Detection', detectorParams.UseAruco3Detection,...
                            'minSideLengthCanonicalImg', int32(detectorParams.MinSideLengthCanonicalImg),...
                            'minMarkerLengthRatioOriginalImg', single(detectorParams.MinMarkerLengthRatioOriginalImg),...
                            'cornerRefinementMethod', detectorParams.CornerRefinementMethod,...
                            'cornerRefinementWinSize', int32(detectorParams.CornerRefinementWinSize),...
                            'cornerRefinementMaxIterations', int32(detectorParams.CornerRefinementMaxIterations),...
                            'cornerRefinementMinAccuracy', detectorParams.CornerRefinementMinAccuracy);
            coder.cstructname(params,'ArucoDetectorParams',...
                'extern', 'HeaderFile', 'readArucoMarkerCore_api.hpp');

            % Call function
            numDetections = uint32(0);

            % Stores the reference to the result object
            detectionsObj = coder.opaquePtr('void', coder.internal.null);
            markerFamilyCombinedLen = int32(0);
            rejectionLength = int32(0);
            poseLength = int32(0);

            if coder.isColumnMajor
                numDetections = coder.ceval('-col', 'readArucoImplCore',...
                    coder.ref(I), isRGB,...
                    nRows, nCols, coder.ref(markersProc),...
                    numMarkers, markersLength, params,...
                    doEstimatePose, markerSize, ...
                    camMatrix, distCoeffs, coder.ref(detectionsObj), ...
                    coder.ref(markerFamilyCombinedLen), coder.ref(rejectionLength),...
                    coder.ref(poseLength));

            else
                % Row major order for computing detections
                IRow = I(:);
                camMatrixRow = camMatrix(:);
                distCoeffsRow = distCoeffs(:);

                numDetections = coder.ceval('-row', 'readArucoImplCore',...
                    coder.ref(IRow), isRGB,...
                    nRows, nCols, coder.ref(markersProc),...
                    numMarkers, markersLength, params,...
                    doEstimatePose, markerSize, ...
                    coder.ref(camMatrixRow), coder.ref(distCoeffsRow), coder.ref(detectionsObj), ...
                    coder.ref(markerFamilyCombinedLen), coder.ref(rejectionLength),...
                    coder.ref(poseLength));
            end
            
            coder.varsize('locs',[4 2 inf]);
            coder.varsize('rejectionLocs',[4 2 inf]);
            coder.varsize('ids',[1 inf]);
            coder.varsize('famLength',[1 inf]);
            coder.varsize('rotMatrices',[3 3 inf]);
            coder.varsize('transVectors',[3 inf]);

            locs = coder.nullcopy(zeros(4,2,numDetections));
            rejections = coder.nullcopy(zeros(4,2,rejectionLength));
            ids   = coder.nullcopy(zeros(1,numDetections));
            famLength  = coder.nullcopy(zeros(1, numDetections,'int32'));
            familyNames = blanks(markerFamilyCombinedLen);
            rotMatrices = coder.nullcopy(zeros(3, 3, poseLength));
            transVectors = coder.nullcopy(zeros(3, poseLength));

            if coder.isColumnMajor
                coder.ceval('assignOutputs', coder.ref(ids), coder.ref(locs), ...
                    coder.ref(famLength), coder.ref(familyNames),...
                    coder.ref(rejections), coder.ref(rotMatrices),...
                    coder.ref(transVectors), detectionsObj, doEstimatePose);
            else
                locsRow = coder.nullcopy(zeros(size(locs(:),1),1,'like',locs));
                rejectionsRow = coder.nullcopy(zeros(size(rejections(:),1),1,'like',rejections));
                rotMatricesRow = zeros(size(rotMatrices(:),1),1,'like',rotMatrices);
                transVectorsRow = zeros(size(transVectors(:),1),1,'like',transVectors);
                
                coder.ceval('-row', 'assignOutputs', coder.ref(ids), coder.ref(locsRow), ...
                    coder.ref(famLength), coder.ref(familyNames),...
                    coder.ref(rejectionsRow), coder.ref(rotMatricesRow),...
                    coder.ref(transVectorsRow), detectionsObj, doEstimatePose);
                if ~isempty(locsRow)
                    locs = reshape(locsRow', size(locs));
                end
                if ~isempty(rejectionsRow)
                    rejections = reshape(rejectionsRow', size(rejections));
                end
                if doEstimatePose
                    rotMatrices = reshape(rotMatricesRow', size(rotMatrices));
                    transVectors = reshape(transVectorsRow', size(transVectors));
                end

            end

            varargout{1} = rotMatrices;
            varargout{2} = transVectors;
            coder.varsize('detectedFamily');
            detectedFamily = coder.nullcopy(cell(1,numDetections));

            % extract individual tag families from combined character
            % vector of the result
            tmpLen = int32(0);
            for i = 1:numDetections
                start = tmpLen + 1;
                tagi = familyNames(start : start + famLength(i) - 1);
                detectedFamily{i} = tagi;
                % detectedFamily1(i) = tagi;
                tmpLen = tmpLen + famLength(i);
            end
            
            %Deallocating the memory
            coder.ceval('deleteResultPtr', detectionsObj);

        end
    end
end

