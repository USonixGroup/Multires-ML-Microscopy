function portableAprilTagBuildInfo(buildInfo, context)
% portableAprilTagBuildInfo:
% This functions includes all link files (win: lib, linux: so, mac: dylib)
% as linkObjects :

%   Copyright 2021-2022 The MathWorks, Inc.

% Platform specific link and non-build files
arch            = computer('arch');
pathBinArch     = fullfile(matlabroot, 'bin', arch, filesep);

switch arch
    case {'win64'}
        nonBuildPath = fullfile(matlabroot, 'bin', arch);
        nonBuildFiles = strcat(nonBuildPath, filesep, 'libAprilTag.dll');
        
    case {'glnxa64'}
        nonBuildPath = pathBinArch;
        nonBuildFiles = strcat(nonBuildPath, filesep, 'libAprilTag.so.3');
        
    case {'maci64','maca64'}
        nonBuildPath = pathBinArch;
        nonBuildFiles = strcat(nonBuildPath, filesep, 'libAprilTag.dylib');
        
end
group = 'BlockModules';
buildInfo.addNonBuildFiles(nonBuildFiles,'', group);

end
