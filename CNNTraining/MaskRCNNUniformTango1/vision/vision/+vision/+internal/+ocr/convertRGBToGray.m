function Igray = convertRGBToGray(I)
% Convert RGB to gray if codegen

%   Copyright 2015-2020 The MathWorks, Inc.

%#codegen

if ~ismatrix(I)    
    Igray = rgb2gray(I);    
else
    Igray = I;
end
