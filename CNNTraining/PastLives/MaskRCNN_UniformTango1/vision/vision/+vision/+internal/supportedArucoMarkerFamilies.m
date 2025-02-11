function families = supportedArucoMarkerFamilies()
% supportedArucoMarkerFamilies Get a list of supported Aruco Marker families.
%   families = supportedArucoMarkerFamilies() returns a string vector containing the
%   list of all supported Aruco Marker families in families.

%   Copyright 2023-204 The MathWorks, Inc.
%#codegen

    families = {'DICT_4X4_50'
                'DICT_4X4_100'
                'DICT_4X4_250'
                'DICT_4X4_1000'
                'DICT_5X5_50'
                'DICT_5X5_100'
                'DICT_5X5_250'
                'DICT_5X5_1000'
                'DICT_6X6_50'
                'DICT_6X6_100'
                'DICT_6X6_250'
                'DICT_6X6_1000'
                'DICT_7X7_50'
                'DICT_7X7_100'
                'DICT_7X7_250'
                'DICT_7X7_1000'
                'DICT_ARUCO_ORIGINAL'};
end