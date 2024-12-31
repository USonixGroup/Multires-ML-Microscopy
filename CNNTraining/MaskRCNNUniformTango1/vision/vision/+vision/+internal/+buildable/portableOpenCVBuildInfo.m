function portableOpenCVBuildInfo(buildInfo, context, fcnName)
% portableOpenCVBuildInfo:
% This functions performs following operations:
%   All platforms:
%       (1) headers: includes ALL openCV header files.
%       (2) nonBuildFiles: includes ALL openCV libraries.
%                        (win: dll, linux: so, mac: dylib) as nonBuildFiles
%       (3) linkObjects: includes ALL openCV libraries.
%                        (win: lib, linux: so, mac: dylib) as linkObjects

%   Copyright 2015-2023 The MathWorks, Inc.
 
opencvbuildinfo.internal.portableOpenCVSharedBuildInfo(buildInfo, context, fcnName,'CVT');

end
