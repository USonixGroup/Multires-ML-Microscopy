%==========================================================================
% Determine if transformation matrix is rigid transformation

%  Copyright 2017-2020 The MathWorks, Inc.
%==========================================================================
function tf = isRigidTransform(T)

%#codegen
dimensionality = size(T,1) - 1;
singularValues = svd(T(1:dimensionality,1:dimensionality));
tf = max(singularValues)-min(singularValues) < 100*eps(max(singularValues(:)));
tf = tf && abs(det(T)-1) < 100*eps(class(T));

end