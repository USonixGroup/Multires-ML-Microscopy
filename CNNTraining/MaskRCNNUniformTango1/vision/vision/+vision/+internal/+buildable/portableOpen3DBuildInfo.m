function portableOpen3DBuildInfo(buildInfo, buildConfig)
% portableOpen3DBuildInfo:
% This functions performs following operations:
%   All platforms:
%       (1) headers: includes ALL open3d header files.
%       (2) includes open3d libraries.
%                        (win: dll, linux: so, mac: dylib) as nonBuildFiles

%   Copyright 2023-2024 The MathWorks, Inc.

errorIfNotHierachicalPackType(buildInfo);

buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
    'vision','builtins','src','o3dcg', 'include')});
buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox', ...
    'vision','builtins','src','o3dcg', 'include', 'open3d','3rdparty')});
externalSourcePath = fullfile(matlabroot, 'toolbox', 'shared',...
    'robotics', 'externalDependency');

% Add eigen includes to buildInfo
eigenIncludePath = fullfile(externalSourcePath, 'eigen', 'include');
buildInfo.addIncludePaths({fullfile(externalSourcePath, 'eigen', 'include','eigen3')});
eigenHeaderFiles = [dir(fullfile(eigenIncludePath, 'eigen3','Eigen', '**', '*'));...
    dir(fullfile(eigenIncludePath, 'eigen3','unsupported', '**', '*'))];

eigenHeaderFiles = eigenHeaderFiles(~[eigenHeaderFiles.isdir]);

arrayfun(@(s)buildInfo.addIncludeFiles(fullfile(s.folder,s.name)), ...
    eigenHeaderFiles, 'UniformOutput', false);

% No need to include libraries in the buildInfo. These libraries are for
% MATLAB host only.
if ~isMatlabHostTarget(buildConfig)
    buildInfo.addSysLibs('Open3D');
else
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
            linkLibPath = fullfile(matlabroot, 'toolbox', 'vision', ...
                'builtins', 'src', 'o3dcg', ...
                arch, 'lib');
            [~, linkLibExt, execLibExt] = buildConfig.getStdLibInfo();

            % Adding Open3D linking library
            linkFiles =  strcat('Open3D', linkLibExt);
            nonBuildPath = pathBinArch;
            nonBuildFiles = strcat(nonBuildPath,filesep,'Open3D',execLibExt);
            buildInfo.addNonBuildFiles(nonBuildFiles, '', group);

            % Force addition of compiler defines. See g2960359.
            % remove this after the above geck is resolved.
            buildInfo.addDefines(MSCVER_FLAG);

            if isOpenmpEnabled(buildConfig)
                % Adding OpenMP compiler flags and linking library
                buildInfo.addCompileFlags('/openmp');
                buildInfo.addLinkFlags('/nodefaultlib:vcomp');
                openMPLinkFiles = strcat('libiomp5md', linkLibExt);
                openMPlinkLibPath = pathBinArch;
            end

        case {'glnxa64'}
            linkLibPath     = pathBinArch;
            ext = '.so';
            linkFiles = strcat('libOpen3D', ext);

            % Force addition of compiler defines. See g2960359.
            % remove this after the above geck is resolved.
            buildInfo.addDefines(GNUC_FLAG);
            if isOpenmpEnabled(buildConfig)
                % Adding OpenMP compiler flags and linking library
                buildInfo.addCompileFlags('-fopenmp');
                openMPLinkFiles = strcat('libiomp5', ext);
                openMPlinkLibPath = fullfile(matlabroot,'sys','os','glnxa64');
            end

        case {'maci64','maca64'}
            linkLibPath     = pathBinArch;
            ext = '.dylib';
            linkFiles = strcat('libOpen3D', ext);

            % Force addition of compiler defines. See g2960359.
            % remove this after the above geck is resolved.
            buildInfo.addCompileFlags('-Wno-builtin-macro-redefined');
            buildInfo.addDefines('__GNUC__=4');
            
            if isOpenmpEnabled(buildConfig)
                % Adding OpenMP compiler flags and linking library
                buildInfo.addCompileFlags('-Xclang -fopenmp');
                externalSourcePathOpenMP = fullfile(matlabroot, 'toolbox', 'eml', 'externalDependency', 'omp', arch);
                buildInfo.addIncludePaths(fullfile(externalSourcePathOpenMP, 'include'));
                if isequal(arch, 'maci64')
                    openMPLinkFiles = strcat('libiomp5', ext);
                    openMPlinkLibPath = fullfile(matlabroot,'sys','os',arch);
                else
                    openMPLinkFiles = strcat('libomp', ext);
                    openMPlinkLibPath = fullfile(matlabroot, 'bin', arch);
                end
            end
    end

    if ~ispc
        cppStandardFlag = '-std=c++14';
        buildInfo.addCompileFlags(cppStandardFlag,'CPP_OPTS');
    end

    linkPriority    = '';
    linkPrecompiled = true;
    linkLinkonly    = true;

    buildInfo.addLinkObjects(linkFiles, linkLibPath, linkPriority, ...
        linkPrecompiled, linkLinkonly, group);

    if isOpenmpEnabled(buildConfig)
        buildInfo.addLinkObjects(openMPLinkFiles, openMPlinkLibPath, linkPriority, ...
            linkPrecompiled, linkLinkonly, group);
    end

end
end

% The following two helper functions are used for error checking and
% hardware specific behavior.
%--------------------------------------------------------------------------
function errorIfNotHierachicalPackType(buildInfo)

if buildInfo.Settings.DisablePackNGo
    return;
else
    matDir = getLocalBuildDir(buildInfo);
    if isfolder(matDir)
        matFilePath = fullfile(matDir, 'codeInfo.mat');
        if exist(matFilePath, 'file')
            x = load(matFilePath);
            y = x.configInfo;
            if isprop(y,'PostCodeGenCommand')
                str = y.PostCodeGenCommand;
                if ~isempty(str)
                    if ~contains(lower(str), 'packngo')
                        if contains(lower(str), 'hierarchical')
                            error(message('images:validate:useHierarchical'));
                        end
                    end
                end
            end
        end
    end
end
end
function out =  isOpenmpEnabled(ctx)
    out = ctx.ConfigData.EnableOpenMP;
end
