function viewTableConv = viewTableConvCheck(viewTable)
% viewTableConvCheck takes in a view table from either either a
%   pcviewset or imageviewset object. It takes a view table that is made
%   up of rigid3d objects and converts them to rigidtform3d.

% Copyright 2022 The MathWorks, Inc.

    viewTableConv = viewTable;
    if ~isempty(viewTable.AbsolutePose)
        viewTableConv.AbsolutePose = vision.internal.viewset.rigid3dConvert(viewTable.AbsolutePose);
    end
end