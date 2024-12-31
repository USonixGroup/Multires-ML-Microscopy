function cvstBuildInfo(buildInfo, context, fcnName, stringCell, varargin)
% opencvBuildInfo: Lists all the libraries required for shared library
% based code generation
%
% stringCell is a list of strings specifying which libraries are required
% stringCell = {'use_tbb', 'use_boost', 'use_opencv_modulename', ...}
% Strings in stringCell can be in any order

%   Copyright 2013-2024 The MathWorks, Inc.

% Allow modules that use default names.
if numel(varargin) == 1
    usesDefaultModname = varargin{1};
else
    usesDefaultModname = false;
end

% Basic info
arch        = computer('arch');
pathBinArch = fullfile(matlabroot,'bin',arch,filesep);
[~, linkLibExt, execLibExt] = context.getStdLibInfo();
%        linkLibExt: one of {'.lib', '.dylib', '.so'}
%        execLibExt: one of {'.dll', '.dylib', '.so'}
group = 'BlockModules';

% =========================================================================
% Step-1: Add include paths for header files
% =========================================================================
buildInfo.addIncludePaths(fullfile(matlabroot,'extern','include'));

% =========================================================================
% Step-2: Add link objects
% =========================================================================
if usesDefaultModname && ispc    
    libName = fcnName;
else
    % if the module overrides MODNAME or not PC, add libmw
    libName = ['libmw' fcnName];
end
libNameCellNoExt = {libName}; %#ok<*EMCA>
linkFiles        = strcat(libNameCellNoExt, linkLibExt);
linkLibPath      = get_linkLibPath(context, arch, pathBinArch);        
linkPriority     = '';
linkPrecompiled  = true;
linkLinkonly     = true;
buildInfo.addLinkObjects(linkFiles,linkLibPath,linkPriority,...
    linkPrecompiled,linkLinkonly,group);

% =========================================================================
% Step-3: Add non-build files
% =========================================================================
% add main library
nonBuildFiles = strcat(pathBinArch, libNameCellNoExt, execLibExt); 

% add other dependent libraries
if hasString('use_tbb', stringCell)
    nonBuildFiles = AddTbbLibs(nonBuildFiles, arch, pathBinArch, execLibExt);
end

if hasString('use_tbbmalloc', stringCell)
    nonBuildFiles = AddTbbMallocLibs(nonBuildFiles, arch, pathBinArch, execLibExt);
end

if hasString('use_tesseract', stringCell)
    nonBuildFiles = AddTesseractLibs(nonBuildFiles, arch, pathBinArch, execLibExt);
end

if hasString('use_leptonica', stringCell)
    nonBuildFiles = AddLeptonicaLibs(nonBuildFiles, arch, pathBinArch, execLibExt);
end

if hasString('use_cpp11compat', stringCell)
     nonBuildFiles = AddCPP11CompatLibs(nonBuildFiles, pathBinArch, execLibExt);
end

if strcmpi(arch, 'glnxa64') || strcmpi(arch, 'maci64')
    if hasString('use_boost', stringCell)
        nonBuildFiles = AddBoostCompatLibs(nonBuildFiles, arch, pathBinArch);
    end
end
% add glnxa64 specific runtime libs
if strcmpi(arch,'glnxa64')    
    nonBuildFiles = AddGLNXRTlibs(nonBuildFiles);    
end

buildInfo.addNonBuildFiles(nonBuildFiles,'',group);
%==========================================================================
function libName = getBoostLibName(pathBinArch, libName)
dirInfo = dir(fullfile(pathBinArch, libName));
libName = dirInfo(1).name;

%==========================================================================
function flag = hasString(str, stringCell)
for ii=1:length(stringCell)
    if strcmpi(stringCell{ii},str)
        flag = true;
        return;
    end
end

flag = false;

%==========================================================================
function linkLibPath = get_linkLibPath(context, arch, pathBinArch)
% finds the path of [libmwFcnName.{lib/so/dylib}]
switch arch
    case {'win32','win64'}
        % associate bridge lib file: (matabroot)\extern\lib\win64\microsoft\libmwfcnName.lib
                
        libDir = images.internal.getImportLibDirName(context); 
        
        linkLibPath = fullfile(matlabroot,'extern','lib',arch,libDir);
        
    case {'glnxa64','maci64','maca64'}
        linkLibPath     = pathBinArch;
    otherwise
        % unsupported
        assert(false,[ arch ' operating system not supported']);
end
  
%==========================================================================
function nonBuildFiles = AddTbbLibs(nonBuildFiles, arch, pathBinArch, execLibExt)
% tbb: used by all
switch arch
    case {'win32','win64'}
        libNameNoExt = 'tbb12';
        libExt = execLibExt; % '.dll' => tbb.dll
    case {'glnxa64'}
        libNameNoExt = 'libtbb';
        libExt = [execLibExt '.12']; % '.so.12' => libtbb.so.12
    case {'maci64'}
        libNameNoExt = 'libtbb';
        libExt = execLibExt; % '.dylib' => libtbb.dylib
    case {'maca64'}
        libNameNoExt = 'libtbb.12';
        libExt = execLibExt; % '.dylib' => libtbb.12.dylib
    otherwise
        % unsupported
        assert(false,[ arch ' operating system not supported']);
end
nonBuildFiles{end+1} = strcat(pathBinArch,libNameNoExt, libExt);

%==========================================================================
function nonBuildFiles = AddTbbMallocLibs(nonBuildFiles, arch, pathBinArch, execLibExt)
% tbb: used by all
switch arch
    case {'win32','win64'}
        libNameNoExt = 'tbbmalloc';
        libExt = execLibExt; % '.dll' => tbbmalloc.dll
    case {'glnxa64'}
        libNameNoExt = 'libtbbmalloc';
        libExt = [execLibExt '.2']; % '.so.2' => libtbbmalloc.so.2
    case {'maci64'}
        libNameNoExt = 'libtbbmalloc';
        libExt = execLibExt; % '.dylib' => libtbbmalloc.dylib
    case {'maca64'}
        libNameNoExt = 'libtbbmalloc.2';
        libExt = execLibExt; % '.dylib' => libtbbmalloc.2.dylib
    otherwise
        % unsupported
        assert(false,[ arch ' operating system not supported']);
end
nonBuildFiles{end+1} = strcat(pathBinArch,libNameNoExt, libExt);

%==========================================================================
function nonBuildFiles = AddTesseractLibs(nonBuildFiles, arch, pathBinArch, execLibExt)

switch arch
    case {'win32','win64'}
        libNameNoExt = 'tesseract50';
        libExt = execLibExt;
    case {'glnxa64'}
        libNameNoExt = 'libtesseract';
        libExt = [execLibExt '.5.0.1'];
    case {'maci64','maca64'}
        libNameNoExt = 'libtesseract';
        libExt = execLibExt;
    otherwise
        % unsupported
        assert(false,[ arch ' operating system not supported']);
end
nonBuildFiles{end+1} = strcat(pathBinArch,libNameNoExt, libExt);

%==========================================================================
function nonBuildFiles = AddLeptonicaLibs(nonBuildFiles, arch, pathBinArch, execLibExt)

switch arch
    case {'win32','win64'}
        libNameNoExt = 'leptonica-1.83.0';
        libExt = execLibExt;
    case {'glnxa64'}
        libNameNoExt = 'libleptonica';
        libExt = [execLibExt '.5'];
    case {'maci64','maca64'}
        libNameNoExt = 'libleptonica';
        libExt = ['.5' execLibExt];
    otherwise
        % unsupported
        assert(false,[ arch ' operating system not supported']);
end
nonBuildFiles{end+1} = strcat(pathBinArch,libNameNoExt, libExt);

%==========================================================================
function nonBuildFiles = AddCPP11CompatLibs(nonBuildFiles, pathBinArch, execLibExt)
% for unix only
libNameNoExt = 'libmwcpp11compat';
libExt = execLibExt; 
nonBuildFiles{end+1} = strcat(pathBinArch,libNameNoExt, libExt);

%==========================================================================
function nonBuildFiles = AddGLNXRTlibs(nonBuildFiles)
% glnxa64 specific runtime libs      
arch = computer('arch');
sysosPath = fullfile(matlabroot,'sys','os',arch,filesep);
nonBuildFiles{end+1} = strcat(sysosPath,'libstdc++.so.6');
nonBuildFiles{end+1} = strcat(sysosPath,'libgcc_s.so.1');
%==========================================================================
function nonBuildFiles = AddBoostCompatLibs(nonBuildFiles, arch, pathBinArch)
if strcmpi(arch, 'glnxa64')
    boostFileSys = getBoostLibName(pathBinArch, 'libmwboost_filesystem.so.*');

    boostSystem  = getBoostLibName(pathBinArch, 'libmwboost_system.so.*');
    % boost_atomic library is needed by boost_filesystem.
    boostAtomic = getBoostLibName(pathBinArch, 'libmwboost_atomic.so.*');

else % must be maci6468
    boostFileSys = 'libmwboost_filesystem.dylib';
    boostSystem  = 'libmwboost_system.dylib';

    boostAtomic  = 'libmwboost_atomic.dylib';
end
nonBuildFiles{end+1} = strcat(pathBinArch, boostFileSys);
nonBuildFiles{end+1} = strcat(pathBinArch, boostSystem);

nonBuildFiles{end+1} = strcat(pathBinArch, boostAtomic);

%{
% test script
vision.internal.buildable.cvstBuildInfoXXX({}, [], ...
                'ComputeMetric', ...
                {'use_tbb', ...
                'use_a', ...
                'use_b', ...
                'use_c', ...
                 'use_tbbmalloc'}); 
%}
