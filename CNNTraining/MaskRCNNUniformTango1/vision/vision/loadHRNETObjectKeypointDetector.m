function obj = loadHRNETObjectKeypointDetector(matFile)
%
  
% Copyright 2023 The MathWorks, Inc.


%#codegen
if coder.target("MATLAB")
    obj = loadHRNETromMatFile(matFile);
else
    obj = vision.internal.codegen.HRNETObjectKeypointDetector(matFile);
end
end


function obj = loadHRNETromMatFile(matFile)

coder.internal.errorIf(~isfile(matFile),'vision:hrnetObjectKeypoint:loadHRNETMATFileDoesNotExist');

s = load(matFile);
idx = structfun(@(x) isa(x,'hrnetObjectKeypointDetector'),s);

coder.internal.errorIf(~any(idx),'vision:hrnetObjectKeypoint:loadHRNETNoHRNetObjects');

tooManyObjects = sum(idx) > 1;
coder.internal.errorIf(tooManyObjects,'vision:hrnetObjectKeypoint:loadHRNETTooManyHRNetObjects')

fnames = fields(s);
obj = s.(fnames{idx});
end
