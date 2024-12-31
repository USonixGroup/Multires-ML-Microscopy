classdef visionSBAAuxiliaryBuildable < coder.ExternalDependency %#codegen
% visionSBAAuxiliaryBuildable - encapsulate visionSBAAuxiliaryBuildable implementation library

% Copyright 2021 The MathWorks, Inc.
    methods (Static)

        function name = getDescriptiveName(~)
            name = 'visionSBAAuxiliaryBuildable';
        end

        function b = isSupportedContext(context)
            b = context.isMatlabHostTarget();
        end

        function updateBuildInfo(buildInfo, context)
            buildInfo.addIncludePaths({...
                fullfile(matlabroot,'toolbox','vision','builtins','src','vision','include'),...
                fullfile(matlabroot,'toolbox','vision','builtins','src','vision','include','calibration'),...
                fullfile(matlabroot,'toolbox','vision','builtins','src','vision','export','include','vision'),...
                fullfile(matlabroot,'toolbox','vision','builtins','src','vision','export','include','vision','calibration'),...
                });
            
            srcPaths = fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision');
            buildInfo.addSourceFiles({'visionSBAAuxiliaryVariableCore.cpp'}, srcPaths);
            buildInfo.addSourcePaths(srcPaths, 'CVT_GROUP');
            
            buildInfo.addIncludeFiles({'vision_defines.h', 'visionSBAAuxiliaryVariableCore_api.hpp', ...
                'rodriguesVectorToMatrix.hpp', 'reprojectPointToCamera.hpp', 'quaternion.hpp'});
        end

        function  [errors, varargout] = visionSBAAuxiliary(xyzPoints, ...
                measurements, cameraMatrices, quaternionBases, visibility, ...
                intrinsics, type, varargin)

            coder.inline('always');
            coder.cinclude('visionSBAAuxiliaryVariableCore_api.hpp');
            
            fcnName = 'visionSBAAuxiliaryVariable';
            
            % appending zero to mark end of type string
            typeN = [type 0];

            numPoints = int32(size(xyzPoints, 2));
            % cameraMatrices - cameraPoses
            numViews = int32(size(cameraMatrices, 2));

            bSingleCamera = isscalar(intrinsics);
            needJacobian = true;
            
            % computing irs and jcs array for sparse array input
            [irsd, ~] = find(visibility);
            irs = int32(irsd-1);
            jcsd = zeros(1, size(visibility, 2) + 1);
            jcsd(1) = 0;
            for i = 2:size(visibility, 2) + 1
                jcsd(i) = nnz(visibility(:, i - 1)) + jcsd(i - 1); 
            end

            jcs = int32(jcsd);
            nvis = int32(jcs(end));

            focal = zeros(2, numViews);
            center = zeros(2, numViews);
            tangDistort = zeros(2, numViews);
            skew = zeros(1, numViews);
            
            if strcmp(type, 'motion')
               
                fixedCameraIndex = -1;
                numFixedCameraIndex = int32(0);
                radDistortion = zeros([size(intrinsics.radialDistortion), numViews]);
                
            else
                fixedCameraIndex = varargin{1};
                numFixedCameraIndex = int32(numel(fixedCameraIndex));
                radDistort = zeros([size(intrinsics(1).radialDistortion), numViews]);
                radDistortion = squeeze(radDistort);
            end
            
            if strcmp(type, 'motion')
                if numViews > 1 && ~bSingleCamera
                    for i = 1:numViews
                        focal(:, i) = intrinsics(i).focalLength;
                        center(:, i) = intrinsics(i).principalPoint;
                        radDistortion(:, i) = intrinsics(i).radialDistortion;
                        tangDistort(:, i) = intrinsics(i).tangentialDistortion;
                        skew(:, i) = intrinsics(i).skew;
                    end
                else
                    focal = intrinsics.focalLength;
                    center = intrinsics.principalPoint;
                    radDistortion = intrinsics.radialDistortion;
                    tangDistort = intrinsics.tangentialDistortion;
                    skew = intrinsics.skew;
                end
            else
            
                if numViews > 1 && ~bSingleCamera
                    for i = 1:numViews
                        focal(:, i) = intrinsics(i).focalLength;
                        center(:, i) = intrinsics(i).principalPoint;
                        radDistortion(:, i) = intrinsics(i).radialDistortion;
                        tangDistort(:, i) = intrinsics(i).tangentialDistortion;
                        skew(:, i) = intrinsics(i).skew;
                    end
                else
                    focal(:, 1) = intrinsics(1).focalLength;
                    center(:, 1) = intrinsics(1).principalPoint;
                    radDistortion(:, 1) = intrinsics(1).radialDistortion;
                    tangDistort(:, 1) = intrinsics(1).tangentialDistortion;
                    skew(:, 1) = intrinsics(1).skew;
                end
            end
           
            numRad = int32(size(intrinsics(1).radialDistortion, 2));
            flagRadial = true;
            flagTang = true;
            
            % outputs declaration

            errors = zeros(2, nvis);
            Uj = zeros(6, 6*numViews);
            Vi = zeros(3, 3*numPoints);
            Wij = zeros(6, 3*nvis);
            eaj = zeros(6, numViews);
            ebi = zeros(3, numPoints);
            
            if coder.isColumnMajor
                % call internal functions to compute errors in column major
                % format
                coder.ceval(fcnName, coder.ref(xyzPoints), ...
                   coder.ref(measurements), numViews, cameraMatrices, ...
                   quaternionBases, bSingleCamera, coder.ref(typeN), needJacobian, ...
                   irs, jcs, coder.ref(fixedCameraIndex), numFixedCameraIndex, ...
                   coder.ref(focal), coder.ref(center), flagRadial, flagTang, coder.ref(radDistortion), ...
                   coder.ref(tangDistort), numRad, coder.ref(skew), coder.ref(errors), ...
                   coder.ref(Uj), coder.ref(Vi), coder.ref(Wij), coder.ref(eaj), ...
                   coder.ref(ebi));
               
            else
                % initialize errors in row major format
                errRow =zeros(nvis, 2);
                ujRow = zeros(6*numViews, 6);
                viRow = zeros(3*numPoints, 3);
                wijRow = zeros(3*nvis, 6);
                eajRow = zeros(numViews, 6);
                ebiRow = zeros(numPoints, 3);

                % internal call to compute outputs in row major format
                coder.ceval('-row', fcnName, xyzPoints', ...
                   measurements', numViews, cameraMatrices', ...
                   quaternionBases', bSingleCamera, coder.ref(typeN), needJacobian, ...
                   irs', jcs', coder.ref(fixedCameraIndex), numFixedCameraIndex, ...
                   focal', center', flagRadial, flagTang, radDistortion, ...
                   tangDistort, numRad, coder.ref(skew), coder.ref(errRow), ...
                   coder.ref(ujRow), coder.ref(viRow), coder.ref(wijRow), coder.ref(eajRow), ...
                   coder.ref(ebiRow));

                errors = errRow';
                Uj = ujRow';
                Vi = viRow';
                Wij = wijRow';
                eaj = eajRow';
                ebi = ebiRow';

            end

            if strcmp(type, 'full')
                varargout{1} = Uj;
                varargout{2} = Vi;
                varargout{3} = Wij;
                varargout{4} = eaj;
                varargout{5} = ebi;
            elseif strcmp(type, 'motion')
                varargout{1} = Uj;
                varargout{2} = eaj;
                varargout{3} = Wij;
                varargout{4} = eaj;
                varargout{5} = ebi;
            else
                varargout{1} = Vi;
                varargout{2} = ebi;
                varargout{3} = Wij;
                varargout{4} = eaj;
                varargout{5} = ebi;
            end
        end
    end
end
