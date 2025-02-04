%vision.internal.ndt.validateTform Validate 'InitialTransform' parameter

% Copyright 2020-2022 The MathWorks, Inc.

%#codegen
function validateTform(in, emptyAllowed)

% By default tform is allowed to be [], unless emptyAllowed is false
if (nargin==1 || emptyAllowed) && isempty(in)
    return;
end

validateattributes(in, {'rigidtform3d','rigid3d','affine3d'}, {'scalar'}, '', ...
    'InitialTransform');

end