function offsetsZ = poseTranslationTargetsZ(lowerBoundZ, upperBoundZ, translationZ)
%POSETRANSLATIONTARGETSZ Calculate relative Z given upper and lower bounds

% Copyright 2023 The MathWorks, Inc.

% Handle degenerate cases gracefully - if Z bound estimation is incorrect
% and the actual centroid does not lie within these bounds.
lowerBoundZ(translationZ < lowerBoundZ) = translationZ(translationZ < lowerBoundZ);

upperBoundZ(translationZ > upperBoundZ) = translationZ(translationZ > upperBoundZ);

offsetsZ = (translationZ - lowerBoundZ) ./ (upperBoundZ - lowerBoundZ + 1e-6);

end