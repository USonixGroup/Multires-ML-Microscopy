function fontOut = checkFont(font, filename)
% Copyright 2022 The MathWorks, Inc.

% Validate 'Font'. Do a case insensitive match
fontOut = validatestring(font, vision.internal.getFontNamesInCell,...
    filename, 'Font');
