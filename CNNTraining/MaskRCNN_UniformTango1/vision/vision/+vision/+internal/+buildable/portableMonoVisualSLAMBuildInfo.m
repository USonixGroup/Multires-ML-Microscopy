function portableMonoVisualSLAMBuildInfo(buildInfo, context)

% Copyright 2023-2024 The MathWorks, Inc.

% Add vocabulary file
vocabularyPath =fullfile(matlabroot,'toolbox', ...
    'vision','builtins','src','shared','vslamcore');
buildInfo.addNonBuildFiles('bagOfFeatures.bin.gz', vocabularyPath);

% Link libraries on host computer
if context.isMatlabHostTarget()
    % Add binary files
    vision.internal.buildable.cvstBuildInfo(buildInfo, context, 'vslamcore', {});
    buildInfo.addIncludePaths(fullfile(matlabroot,'extern','include','vslamcore'));
    buildInfo.addIncludePaths(fullfile(matlabroot,'extern','include','ceres'));

    arch            = computer('arch');
    pathBinArch     = fullfile(matlabroot, 'bin', arch, filesep);

    switch arch
        case {'win64'}
            nonBuildPath = fullfile(matlabroot, 'bin', arch);
            nonBuildFiles = strcat(nonBuildPath, filesep, {'DBoW2.lib', ...
                'ceres.dll', 'libmwvslamcore.dll', 'libmwcerescodegen.dll'});

        case {'glnxa64'}
            nonBuildPath = pathBinArch;
            nonBuildFiles = strcat(nonBuildPath, filesep, {'libDBoW2.so', ...
                'libceres.so.2', 'libmwvslamcore.so', 'libmwcerescodegen.so'});

        otherwise
            nonBuildPath = pathBinArch;
            nonBuildFiles = strcat(nonBuildPath, filesep, {'libDBoW2.dylib', ...
                'libceres.dylib', 'libmwvslamcore.dylib', 'libmwcerescodegen.dylib'});

    end
    group = 'BlockModules';
    buildInfo.addNonBuildFiles(nonBuildFiles,'', group);
else

    % Add source files
    vslamSourcePath = fullfile(matlabroot,'toolbox', ...
        'vision','builtins','src','shared','vslamcore');
    vslamSourceFiles = dir(fullfile(vslamSourcePath,'*.cpp'));
    buildInfo.addSourceFiles({vslamSourceFiles.name});
    buildInfo.addSourcePaths(vslamSourcePath);

    dbow2SourcePath = fullfile(matlabroot,'toolbox','vision',...
        'builtins','src','thirdparty','DBoW2','src');
    dbow2SourceFiles = dir(fullfile(dbow2SourcePath,'*.cpp'));
    buildInfo.addSourceFiles({dbow2SourceFiles.name});
    buildInfo.addSourcePaths(dbow2SourcePath);

    ocvSourcePath = fullfile(matlabroot,'toolbox','vision', ...
        'builtins','src','ocv');
    ocvSourceFiles = dir(fullfile(ocvSourcePath,'cgCommon.cpp'));
    buildInfo.addSourceFiles({ocvSourceFiles.name});
    buildInfo.addSourcePaths(ocvSourcePath);

    % Ceres solver
    buildInfo.addSourcePaths({fullfile(matlabroot,'toolbox', ...
        'shared','ceres','builtins','libsrc','cerescodegen')});
    buildInfo.addSourceFiles({'cerescodegen_api.cpp','factor_graph.cpp',...
        'common_factors_2.cpp', 'imu_factor.cpp', 'utilities.cpp'});

    % Add header files
    dbow2Path = fullfile(matlabroot,'toolbox','vision',...
        'builtins','src','thirdparty','DBoW2');
    dbow2HeaderPath = fullfile(dbow2Path,'include','DBoW2');
    dbow2HeaderFiles = dir(fullfile(dbow2HeaderPath,'*.h'));
    buildInfo.addIncludePaths(dbow2HeaderPath);
    buildInfo.addIncludeFiles({dbow2HeaderFiles.name});

    vslamHeaderPath = fullfile(matlabroot,'toolbox','vision', ...
        'builtins','src','shared','vslamcore','export','include','vslamcore');
    vslamHppHeaderFiles = dir(fullfile(vslamHeaderPath,'*.hpp'));
    buildInfo.addIncludePaths(vslamHeaderPath);
    buildInfo.addIncludeFiles({vslamHppHeaderFiles.name});
    
    % OpenCV utility
    buildInfo.addIncludePaths(fullfile(matlabroot,'toolbox', ...
        'vision','builtins','src','ocv','include') );
    buildInfo.addIncludeFiles({'vision_defines.h', 'cgCommon.hpp'});
    
    % Ceres solver
    ceresHeaderPath = fullfile(matlabroot,'extern','include','ceres');
    ceresHeaderFiles = dir(fullfile(ceresHeaderPath,'*.hpp'));
    buildInfo.addIncludePaths(ceresHeaderPath);
    buildInfo.addIncludeFiles({ceresHeaderFiles.name});

    % Add Eigen
    eigen_link_flags = '`pkg-config --cflags --libs eigen3`';
    buildInfo.addLinkFlags(eigen_link_flags);

    % Add Ceres library
    buildInfo.addLinkFlags('-l${CMAKE_SHARED_LIBRARY_PREFIX}glog${CMAKE_SHARED_LIBRARY_SUFFIX}');
    buildInfo.addLinkFlags('-l${CMAKE_SHARED_LIBRARY_PREFIX}ceres${CMAKE_SHARED_LIBRARY_SUFFIX}');

    % Add DBoW2 license file
    buildInfo.addNonBuildFiles('dbow2.rights', dbow2Path);
    
    if ~isempty(context.ConfigData.Hardware) && contains(context.ConfigData.Hardware.Name, 'Robot Operating System') 
        buildInfo.addDefines("VSLAMROSCODEGEN"); % Define a macro to control the path of vocabulary file
    end
end
end