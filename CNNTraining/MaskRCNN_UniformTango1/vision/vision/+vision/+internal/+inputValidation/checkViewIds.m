function viewIds = checkViewIds(viewIds, isSingleView, fileName, varName)
%
%  Copyright 2020-2021 The MathWorks, Inc.
%#codegen
if isSingleView
    validateattributes(viewIds, {'numeric'}, {'nonsparse', 'scalar', ...
        'integer', 'positive', 'real'}, fileName, varName);
    
else
    validateattributes(viewIds, {'numeric'}, {'nonsparse', 'vector', ...
        'integer', 'positive', 'real'}, fileName, varName);
    coder.internal.errorIf((numel(unique(viewIds)) ~= numel(viewIds)), ...
        'vision:viewSet:duplicateViewIds');

end
viewIds = uint32(viewIds);
end