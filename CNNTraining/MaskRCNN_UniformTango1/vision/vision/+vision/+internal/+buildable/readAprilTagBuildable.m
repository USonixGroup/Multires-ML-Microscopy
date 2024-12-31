classdef readAprilTagBuildable < coder.ExternalDependency %#codegen
    % readAprilTagBuildable - encapsulate readAprilTag implementation library

    % Copyright 2021-2024 The MathWorks, Inc.
    methods (Static)

        function name = getDescriptiveName(~)
            name = 'readAprilTagBuilder';
        end

        function b = isSupportedContext(context)
            b = context.isMatlabHostTarget();
        end

        function updateBuildInfo(buildInfo, context)
            vision.internal.buildable.cvstBuildInfo(buildInfo, context, ...
                'readAprilTag',{});
            vision.internal.buildable.portableAprilTagBuildInfo(buildInfo, context);
        end
        
        %--------------------------------------------------------------------------
        % Detect id, image locations and tag families detected based on
        % Image, tag family and quad decimate. 
        %--------------------------------------------------------------------------
        function [id, loc, detectedFamily, varargout] = readAprilTagID(I, tagFamily, estimatePose, detectorParams, varargin)
            
            coder.inline('always');
            coder.cinclude('cvstCG_readAprilTag.h');

            if estimatePose == true
                
                % extracting focal length, principal Point and tag Size
                % from varargin 
                focalLength = varargin{1};
                principalPoint = varargin{2};
                tagSize = double(varargin{3});
            else
               
                % initializing focal length, principal point and tag size
                % with default values to allocate memory
                focalLength = ones(1,2);
                principalPoint = ones(1,2);
                tagSize = 1;
            end

            if iscellstr(tagFamily)

                % extract number of tag families, length of all tag
                % families and merge tag families into a single character
                % vector

                numTags = int32(size(tagFamily,2));
                tagLength = int32(strlength(tagFamily));

                if numTags == 1
                    
                    % appending zero to mark end of family lengths in case
                    % of single tag family
                    tagLength = int32([strlength(tagFamily) 0]);
                end

                %merges all tag families into a single character vector
                tagFamilyProc = strjoin(tagFamily,'');

            else
                
                % this condition executes if a single tag family is passed
                % as a character vector
                numTags = int32(1);
                tagLength = int32([strlength(tagFamily) 0]);
                tagFamilyProc = [tagFamily 0];
            end


            nRows = int32(size(I,1));
            nCols = int32(size(I,2));
            
            % allocate memory for output variables
            detectionsObj = coder.opaquePtr('void',coder.internal.null);
            detectionSize = int32(0);
            tagFamilyCombinedLen = int32(0);
            fcName = 'getAprilTagIdPoseDetections';

            params = struct('nthreads',int32(detectorParams.nthreads),...
                            'quadDecimate',single(detectorParams.quadDecimate),...
                            'quadSigma',single(detectorParams.quadSigma),...
                            'refineEdges',logical(detectorParams.refineEdges),...
                            'decodeSharpening',double(detectorParams.decodeSharpening),...
                            'numBitsCorrected',int32(detectorParams.numBitsCorrected),...
                            'autoBitCorrection',logical(detectorParams.autoBitCorrection));
            coder.cstructname(params,'AprilTagDetectorParams',...
                'extern', 'HeaderFile', 'cvstCG_readAprilTag.h');

            % Row major order for computing detections
            I_row = I(:);

            posesObj = coder.opaquePtr('void',coder.internal.null);
            numOfDetections = int32(1);
            detectedPosesSize = int32(0);

            if estimatePose == false
                if coder.isColumnMajor
                    % call internal function to compute detections
                    detectionSize = coder.ceval(fcName, coder.ref(I),...
                        coder.ref(tagFamilyProc), estimatePose, params,...
                        nRows, nCols, coder.ref(detectionsObj), ...
                        coder.ref(tagFamilyCombinedLen), numTags, ...
                        tagLength, coder.ref(focalLength), coder.ref(principalPoint), ...
                        tagSize, coder.ref(posesObj), coder.ref(numOfDetections));
                else
                    detectionSize = coder.ceval('-row',fcName, coder.ref(I_row),...
                        coder.ref(tagFamilyProc), estimatePose, params,...
                        nRows, nCols, coder.ref(detectionsObj), ...
                        coder.ref(tagFamilyCombinedLen), numTags, ...
                        tagLength,coder.ref(focalLength), coder.ref(principalPoint), ...
                        tagSize, coder.ref(posesObj), coder.ref(numOfDetections));
                end
            else
                if coder.isColumnMajor
                    detectedPosesSize = coder.ceval(fcName, coder.ref(I), coder.ref(tagFamilyProc), ...
                        estimatePose, params, nRows, ...
                        nCols, coder.ref(detectionsObj), ...
                        coder.ref(tagFamilyCombinedLen), numTags, tagLength,...
                        coder.ref(focalLength), coder.ref(principalPoint), ...
                        tagSize, coder.ref(posesObj), coder.ref(detectionSize));
                else
                    detectedPosesSize = coder.ceval('-row',fcName, coder.ref(I_row),...
                        coder.ref(tagFamilyProc), estimatePose, params,...
                        nRows, nCols, coder.ref(detectionsObj), ...
                        coder.ref(tagFamilyCombinedLen), numTags, ...
                        tagLength,coder.ref(focalLength), coder.ref(principalPoint), ...
                        tagSize, coder.ref(posesObj), coder.ref(detectionSize));
                end
            end

            % allocate memory for id/loc/rotMatrices and translation vectors
            % based on detected poses size
            id = ones(1,detectionSize);
            loc = zeros(4,2,detectionSize);
            familyDetectionsLen = zeros(1,detectionSize);
            rotMatrices = zeros(3,3,detectedPosesSize);
            transVectors = zeros(3,1,detectedPosesSize);
            familyNames = blanks(tagFamilyCombinedLen);

            locsRow = coder.nullcopy(zeros(size(loc(:),1),1,'like',loc));
            rotMatricesRow = coder.nullcopy(zeros(size(rotMatrices(:),1),1,'like',rotMatrices));
            transVectorsRow = coder.nullcopy(zeros(size(transVectors(:),1),1,'like',transVectors));

            if estimatePose == false
                if coder.isColumnMajor
                    % initialize id/loc/detectedfamilies after detections are
                    % computed
                    coder.ceval('-col', 'initializeOutput', coder.ref(id),...
                        coder.ref(loc), coder.ref(familyDetectionsLen), coder.ref(familyNames), detectionsObj, detectionSize);
                else
                    coder.ceval('-row', 'initializeOutput', coder.ref(id),...
                        coder.ref(locsRow),coder.ref(familyDetectionsLen), coder.ref(familyNames), detectionsObj, detectionSize);

                    if detectionSize ~= 0
                        loc = reshape(locsRow',size(loc));
                    end
                end
                varargout{1} = ones(3);
                varargout{2} = ones(1,3);
            else
                if coder.isColumnMajor
                    % call function to initialize id/loc/detected family based
                    % on detection object
                    coder.ceval('-col', 'initializeOutput', coder.ref(id),...
                        coder.ref(loc), coder.ref(familyDetectionsLen), ...
                        coder.ref(familyNames), detectionsObj, detectionSize);

                    % call function to initialize rot matrices and translation
                    % vectors based on detected poses object
                    coder.ceval('-col', 'initializePoses',coder.ref(rotMatrices),...
                        coder.ref(transVectors), detectionsObj, posesObj, detectedPosesSize);
                else
                    coder.ceval('-row','initializeOutput', coder.ref(id),...
                        coder.ref(locsRow),coder.ref(familyDetectionsLen), ...
                        coder.ref(familyNames), detectionsObj,detectionSize);
                    
                    coder.ceval('-row','initializePoses',coder.ref(rotMatricesRow),...
                        coder.ref(transVectorsRow), detectionsObj, posesObj, detectedPosesSize);

                    if detectionSize ~= 0
                        loc = reshape(locsRow',size(loc));
                    end

                    if detectedPosesSize ~= 0
                        rotMatrices = reshape(rotMatricesRow', size(rotMatrices));
                        transVectors = reshape(transVectorsRow', size(transVectors));
                    end
                end
                varargout{1} = rotMatrices;
                varargout{2} = transVectors;
            end

            detectedFamily = cell(1,detectionSize);
            coder.varsize('detectedFamily');

            % extract individual tag families from combined character
            % vector of the result
            tmpLen = 0;
            for i = 1 : detectionSize
                start = tmpLen + 1;
                tagi = familyNames(start : start + familyDetectionsLen(i) - 1);
                detectedFamily{i} = tagi;
                tmpLen = tmpLen + familyDetectionsLen(i);

            end
        end
    end
end
