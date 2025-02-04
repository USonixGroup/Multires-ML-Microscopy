function connTableConv = connTableConvCheck(connTable)
% connTableConvCheck takes in a connection table from either either a
%   pcviewset or imageviewset object. It checks if the connection table is
%   made up of rigid3d or affine3d objects, and if so, converts them to
%   rigidtform3d (from rigid3d) or simtform3d (from affine3d).

% Copyright 2022 The MathWorks, Inc.

    connTableConv = connTable;
    if ~isempty(connTable.RelativePose)
        if iscell(connTable.RelativePose)   % imageviewset
            % Since cells can be varied in object type, each cell is
            % checked and converted individually here.
            for i = 1:height(connTable.RelativePose)
                if isa(connTable.RelativePose{i}, 'rigid3d')
                    connTableConv.RelativePose{i} = rigidtform3d(connTable.RelativePose{i}.T');
                else
                    connTableConv.RelativePose{i} = simtform3d(connTable.RelativePose{i}.T');
                end
            end
        else
            connTableConv.RelativePose = vision.internal.viewset.rigid3dConvert(connTable.RelativePose);
        end
    end
end