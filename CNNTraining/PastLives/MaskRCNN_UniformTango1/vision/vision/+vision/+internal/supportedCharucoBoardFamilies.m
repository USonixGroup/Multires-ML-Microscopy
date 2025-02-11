function families = supportedCharucoBoardFamilies()
% supportedCharucoBoardFamilies Get a list of supported ArUco marker families
% in ChArUco boards.
%
%   families = supportedCharucoBoardFamilies() returns a cell array containing 
%   the list of all supported ArUco marker families in ChArUco boards.

%   Copyright 2024 The MathWorks, Inc.

    families = {'DICT_4X4_1000'
                'DICT_5X5_1000'
                'DICT_6X6_1000'
                'DICT_7X7_1000'
                'DICT_ARUCO_ORIGINAL'};
end