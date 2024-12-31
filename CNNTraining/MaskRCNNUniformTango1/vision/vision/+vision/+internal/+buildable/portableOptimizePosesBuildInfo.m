function portableOptimizePosesBuildInfo(buildInfo, context)
% portableAprilTagBuildInfo:
% This functions includes all link files (win: lib, linux: so, mac: dylib)
% as linkObjects :

%   Copyright 2021-2023 The MathWorks, Inc.

% Platform specific link and non-build files
if isMatlabHostTarget(context)
    arch            = computer('arch');
    pathBinArch     = fullfile(matlabroot, 'bin', arch, filesep);

    switch arch
        case {'win64'}
            nonBuildPath = fullfile(matlabroot, 'bin', arch);
            nonBuildFiles = strcat(nonBuildPath, filesep, {'g2o_solve.dll', 'g2o_types_sim3.dll', 'g2o_types_sclam2d.dll' ...
                'g2o_types_slam2d.dll', 'g2o_types_slam3d.dll', 'g2o_core.dll', 'g2o_stuff.dll', 'g2o_types_sba.dll'});

        case {'glnxa64'}
            nonBuildPath = pathBinArch;
            nonBuildFiles = strcat(nonBuildPath, filesep, {'libg2o_solve.so','libg2o_types_sim3.so', ...
                'libg2o_types_slam2d.so', 'libg2o_types_slam3d.so', 'libg2o_core.so', 'libg2o_stuff.so', 'libg2o_types_sba.so'});

        case {'maci64','maca64'}
            nonBuildPath = pathBinArch;
            nonBuildFiles = strcat(nonBuildPath, filesep, {'libg2o_solve.dylib', 'libg2o_types_sim3.dylib', ...
                'libg2o_types_slam2d.dylib', 'libg2o_types_slam3d.dylib', 'libg2o_core.dylib', 'libg2o_stuff.dylib', 'libg2o_types_sba.dylib'});

    end
    group = 'BlockModules';
    buildInfo.addNonBuildFiles(nonBuildFiles,'', group);

else
    srcPaths = fullfile(matlabroot,'toolbox', ...
        'vision','builtins','src', 'shared', 'optimizePoses');
    buildInfo.addIncludePaths({fullfile(matlabroot,'toolbox','vision','builtins','src',...
        'shared', 'optimizePoses', 'export', 'include', 'optimizePoses')});
    buildInfo.addSourceFiles({'optimizePosesCore.cpp'}, srcPaths);
    buildInfo.addIncludeFiles('optimizePoses_published_c_api.hpp');

    buildInfo.addDefines('-D__portable__');
    externalSourcePath = fullfile(matlabroot, 'toolbox', 'shared',...
        'robotics', 'externalDependency');
    buildInfo.addIncludePaths({fullfile(externalSourcePath, 'eigen', 'include', ...
        'eigen3')});
    % Force addition of compiler defines. See g2960359.
    % remove this after the above geck is resolved.
    buildInfo.addDefines('__GNUC__=10');
    % Add g2o libraries -- REMOVE
    buildInfo.addLinkFlags('-l${CMAKE_SHARED_LIBRARY_PREFIX}g2o_types_sim3${CMAKE_SHARED_LIBRARY_SUFFIX}');
    buildInfo.addLinkFlags('-l${CMAKE_SHARED_LIBRARY_PREFIX}g2o_types_sclam2d${CMAKE_SHARED_LIBRARY_SUFFIX}');
    buildInfo.addLinkFlags('-l${CMAKE_SHARED_LIBRARY_PREFIX}g2o_types_slam2d${CMAKE_SHARED_LIBRARY_SUFFIX}');
    buildInfo.addLinkFlags('-l${CMAKE_SHARED_LIBRARY_PREFIX}g2o_types_slam3d${CMAKE_SHARED_LIBRARY_SUFFIX}');
    buildInfo.addLinkFlags('-l${CMAKE_SHARED_LIBRARY_PREFIX}g2o_core${CMAKE_SHARED_LIBRARY_SUFFIX}');
    buildInfo.addLinkFlags('-l${CMAKE_SHARED_LIBRARY_PREFIX}g2o_stuff${CMAKE_SHARED_LIBRARY_SUFFIX}');
    buildInfo.addLinkFlags('-l${CMAKE_SHARED_LIBRARY_PREFIX}g2o_types_sba${CMAKE_SHARED_LIBRARY_SUFFIX}');

    % Add Eigen
    eigen_link_flags = '`pkg-config --cflags --libs eigen3`';
    buildInfo.addLinkFlags(eigen_link_flags);
end
