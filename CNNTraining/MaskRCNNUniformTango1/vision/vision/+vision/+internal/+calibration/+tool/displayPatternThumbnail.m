% displayPatternThumbnail Display pattern thumbnail image in pattern properties panel.

% Copyright 2021 The MathWorks, Inc.

function displayPatternThumbnail(panel, imageFileName, ~)
    uiimage(panel,'ImageSource',imageFileName);
end