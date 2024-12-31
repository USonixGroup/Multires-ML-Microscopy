classdef registerICPBuildablePortable < coder.ExternalDependency %#codegen
    % registerICPBuildablePortable - encapsulate pcregistericp core implementataion

    % Copyright 2024 The MathWorks, Inc.

    methods (Static)

        function name = getDescriptiveName(~)
            name = 'registerICPBuildablePortable';
        end

        function b = isSupportedContext(~)
            b = true; % Supports non-host target
        end

        function updateBuildInfo(buildInfo,buildConfig)
            % Set to 1 to disable use of multithreading in Kdtree indexing
            buildInfo.addDefines('DISABLE_MULTITHREADED_INDEXING=1');

            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','vision','include'), ...
                fullfile(matlabroot,'toolbox','vision','builtins','src',...
                'visionopen3d','include')});
            
             % Add these files to make pcregistericp fully portable
            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','thirdparty','open3dICP','include')});

            srcPaths = repmat({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','visionopen3d')}, [4 1]);
            buildInfo.addSourceFiles({'registerICPCore.cpp','registration.cpp',...
                'generalizedICP.cpp', 'coloredICP.cpp'},srcPaths);
            buildInfo.addIncludeFiles({'vision_defines.h', ...
                'registerICPUtils.hpp','registration.h','generalizedICP.h', 'coloredICP.h'});

            registrationHeaderPath = fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','thirdparty','open3dICP','include','open3d','pipelines','registration');
            % Add open3d registration includes to buildInfo
            registrationHeaderFiles = [dir(registrationHeaderPath)];

            registrationHeaderFiles = registrationHeaderFiles(~[registrationHeaderFiles.isdir]);

            arrayfun(@(s)buildInfo.addIncludeFiles(fullfile(s.folder,s.name)), ...
                registrationHeaderFiles, 'UniformOutput', false);

            utilityHeaderPath = fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','thirdparty','open3dICP','include','open3d','utility');
            % Add open3d utility includes to buildInfo
            utilityHeaderFiles = [dir(utilityHeaderPath)];

            utilityHeaderFiles = utilityHeaderFiles(~[utilityHeaderFiles.isdir]);

            arrayfun(@(s)buildInfo.addIncludeFiles(fullfile(s.folder,s.name)), ...
                utilityHeaderFiles, 'UniformOutput', false);

            geometryHeaderPath = fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','thirdparty','open3dICP','include','open3d','geometry');
            % Add open3d geometry includes to buildInfo
            geometryHeaderFiles = [dir(geometryHeaderPath)];

            geometryHeaderFiles = geometryHeaderFiles(~[geometryHeaderFiles.isdir]);

            arrayfun(@(s)buildInfo.addIncludeFiles(fullfile(s.folder,s.name)), ...
                geometryHeaderFiles, 'UniformOutput', false);

            buildInfo.addIncludeFiles('registerICPCore_api.hpp');

            srcPaths = repmat({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','thirdparty','open3dICP','src')}, [15 1]);
            buildInfo.addSourceFiles({'Eigen.cpp','Helper.cpp',...
                'Parallel.cpp','Random.cpp','Logging.cpp','ProgressBar.cpp', ...
                'CPUInfo.cpp','EstimateNormals.cpp','Geometry3D.cpp','PointCloud.cpp', ...
                'KDTreeFlann.cpp','CorrespondenceChecker.cpp','Feature.cpp', ...
                'RobustKernel.cpp','TransformationEstimation.cpp'},srcPaths);
            buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
                'vision','builtins','src','o3dcg', 'include', 'open3d','3rdparty')});

            % Add eigen includes to buildInfo
            externalSourcePath = fullfile(matlabroot, 'toolbox', 'shared',...
                'robotics', 'externalDependency');
            buildInfo.addIncludePaths({fullfile(externalSourcePath, 'eigen', 'include','eigen3')});
            
            % Ignore including eigen header files for toolchain cmake in
            % windows
            if ~((computer('arch')=="win64") && contains(buildConfig.ToolchainInfo.Name,'CMake'))
                eigenIncludePath = fullfile(externalSourcePath, 'eigen', 'include');
                eigenHeaderFiles = [dir(fullfile(eigenIncludePath, 'eigen3','Eigen', '**', '*'));...
                    dir(fullfile(eigenIncludePath, 'eigen3','unsupported', '**', '*'))];

                eigenHeaderFiles = eigenHeaderFiles(~[eigenHeaderFiles.isdir]);

                arrayfun(@(s)buildInfo.addIncludeFiles(fullfile(s.folder,s.name)), ...
                    eigenHeaderFiles, 'UniformOutput', false);
            end

            if isMatlabHostTarget(buildConfig)
                [GNUC_FLAG, MSCVER_FLAG] = vision.internal.buildable.compilerBuildInfo();
                % gnuCompilerOnWin is true while using MinGW64 compiler on windows
                gnuCompilerOnWin = ispc && contains(buildConfig.ToolchainInfo.Name, ...
                    {'gcc','g++','gnu', 'mingw'},'IgnoreCase',true);

                intelCompilerOnWin = ispc && contains(buildConfig.ToolchainInfo.Name, ...
                    {'intel'},'IgnoreCase',true);

                if gnuCompilerOnWin || intelCompilerOnWin
                    % only visual studio compilers are supported on windows
                    error(message('vision:internal:UnsupportedWindowsCompiler'));
                end

                % Shared robotics 3rdparty source folder
                externalSourcePath = fullfile(matlabroot, 'toolbox', 'shared',...
                    'robotics', 'externalDependency');
                buildInfo.addIncludePaths({fullfile(externalSourcePath, 'eigen', 'include', ...
                    'eigen3')});

                arch            = computer('arch');
                pathBinArch     = fullfile(matlabroot, 'bin', arch, filesep);
                group = 'BlockModules';
                switch arch
                    case {'win64'}
                        % Force addition of compiler defines. See g2960359.
                        % remove this after the above geck is resolved.
                        buildInfo.addDefines(MSCVER_FLAG);
                        [~, linkLibExt, ~] = buildConfig.getStdLibInfo();
                        if buildConfig.ConfigData.EnableOpenMP
                            % Adding OpenMP compiler flags and linking library
                            buildInfo.addCompileFlags('/openmp');
                            buildInfo.addLinkFlags('-nodefaultlib:vcomp');
                            openMPLinkFiles = strcat('libiomp5md', linkLibExt);
                            openMPlinkLibPath = pathBinArch;
                        end

                    case {'glnxa64'}
                        % Force addition of compiler defines. See g2960359.
                        % remove this after the above geck is resolved.
                        buildInfo.addDefines(GNUC_FLAG);
                        if buildConfig.ConfigData.EnableOpenMP
                            % Adding OpenMP compiler flags and linking library
                            buildInfo.addCompileFlags('-fopenmp');
                            openMPLinkFiles = strcat('libiomp5', '.so');
                            openMPlinkLibPath = fullfile(matlabroot,'sys','os','glnxa64');
                        end

                    case {'maci64','maca64'}

                        ext = '.dylib';
                        % Force addition of compiler defines. See g2960359.
                        % remove this after the above geck is resolved.
                        buildInfo.addCompileFlags('-Wno-builtin-macro-redefined');
                        buildInfo.addDefines('__GNUC__=4');

                        if buildConfig.ConfigData.EnableOpenMP
                            % Adding OpenMP compiler flags and linking library
                            buildInfo.addCompileFlags('-Xclang -fopenmp');
                            externalSourcePathOpenMP = fullfile(matlabroot, 'toolbox', 'eml', 'externalDependency', 'omp', arch);
                            buildInfo.addIncludePaths(fullfile(externalSourcePathOpenMP, 'include'));
                            buildInfo.addIncludeFiles('omp.h');

                            if isequal(arch, 'maci64')
                                openMPLinkFiles = strcat('libiomp5', ext);
                                openMPlinkLibPath = fullfile(matlabroot,'sys','os',arch);
                            else
                                openMPLinkFiles = strcat('libomp', ext);
                                openMPlinkLibPath = fullfile(matlabroot, 'bin', arch);
                            end
                        end
                end
                ismac = isequal(arch,'maci64')|| isequal(arch,'maca64');
                if ~ispc && ~( ismac && isequal(buildConfig.CodeGenTarget,'rtw'))
                    cppStandardFlag = '-std=c++14';
                    buildInfo.addCompileFlags(cppStandardFlag,'CPP_OPTS');
                end

                linkPriority    = '';
                linkPrecompiled = true;
                linkLinkonly    = true;

                if buildConfig.ConfigData.EnableOpenMP
                    buildInfo.addLinkObjects(openMPLinkFiles, openMPlinkLibPath, linkPriority, ...
                        linkPrecompiled, linkLinkonly, group);
                end
            end
        end

        %------------------------------------------------------------------
        % Write all supported data-type specific function calls
        function [outTransform, rmse] = registerICP(obj)

            coder.inline('always');
            coder.cinclude('registerICPCore_api.hpp');

            locationA = obj.moving;
            locationB = obj.fixed;
            numPointsA = int32(size(obj.moving,1));
            numPointsB = int32(size(obj.fixed,1));

            dataType = class(locationA);

            hasNormalsA = obj.HasNormalsA;
            hasNormalsB = obj.HasNormalsB;

            if hasNormalsA
                normalsA = obj.movingNormals;
            else
                normalsA = zeros(coder.ignoreConst(0),3,dataType);
            end

            if hasNormalsB
                normalsB = obj.fixedNormals;
            else
                normalsB = zeros(coder.ignoreConst(0),3,dataType);
            end

            useColors          = obj.UseColors;
            voxelSizes         = obj.VoxelSizes;
            maxIterationVector = obj.MaxIterationsVector;
            if useColors
                movingColors  = obj.movingColors;
                fixedColors  = obj.fixedColors;
            else
                movingColors  = zeros(coder.ignoreConst(0),3);
                fixedColors  = zeros(coder.ignoreConst(0),3);
            end

            initTform = obj.InitialTransform;
            dataTypeTform = [class(initTform) 0];
            maxIteration = int32(obj.MaxIteration);
            relativeRotation = obj.RelativeRotation;
            relativeTranslation = obj.RelativeTranslation;

            useInlierRatio = obj.UseInlierRatio;

            if(useInlierRatio)
                metric = obj.InlierRatio;
            else
                if ~isempty(obj.MaxInlierDistance)
                    metric = obj.MaxInlierDistance;
                else
                    metric = 0;
                end
            end
            method = [obj.Method 0];

            outTransform = zeros(4);
            % Getting the function name w.r.t dataType
            fcnName = ['registerICP_', dataType];

            % Calling the core C++ function
            rmse = 0;
            if coder.isColumnMajor
                rmse = coder.ceval('-col', fcnName, coder.ref(locationA), coder.ref(locationB),...
                    hasNormalsA, coder.ref(normalsA), hasNormalsB, coder.ref(normalsB),...
                    useColors, coder.ref(movingColors), coder.ref(fixedColors),...
                    coder.ref(voxelSizes), coder.ref(maxIterationVector), ...
                    coder.ref(initTform), coder.ref(dataTypeTform),...
                    maxIteration, ...
                    method, ...
                    relativeRotation, ...
                    relativeTranslation, ...
                    metric, ...
                    numPointsA,...
                    numPointsB, ...
                    useInlierRatio,...
                    coder.ref(outTransform));
            else
                outTransformTemp = zeros(4);
                rmse = coder.ceval('-row', fcnName, locationA(:), locationB(:),...
                    hasNormalsA, normalsA(:), hasNormalsB, normalsB(:),...
                    useColors, movingColors(:), fixedColors(:), voxelSizes(:),...
                    maxIterationVector(:), initTform(:), coder.ref(dataTypeTform),...
                    maxIteration, ...
                    method, ...
                    relativeRotation, ...
                    relativeTranslation, ...
                    metric, ...
                    numPointsA,...
                    numPointsB, ...
                    useInlierRatio,...
                    coder.ref(outTransformTemp));
                outTransform = outTransformTemp';

            end
        end
    end
end
