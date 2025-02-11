function tf = checkIfHasView(viewsOrViewIds, viewIds)
%
%  Copyright 2020-2021 The MathWorks, Inc.
%#codegen

if isempty(viewsOrViewIds)
    tf = false;
else
    if istable(viewsOrViewIds) % Views
        tf = ismember(viewIds, viewsOrViewIds.ViewId);
    else
        tf = ismember(viewIds, viewsOrViewIds);
    end
end
end