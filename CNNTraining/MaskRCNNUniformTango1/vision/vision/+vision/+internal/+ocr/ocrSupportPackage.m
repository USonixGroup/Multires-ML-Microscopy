function ocrSupportPackage
    % This function launches the Add-On Explorer, and navigates to OCR
    % Support package page where you will be able to download and install 
    % the add-on.

%   Copyright 2021 The MathWorks, Inc.
    
    matlab.internal.addons.launchers.showExplorer('VP',"identifier",'CVST_OCR_LANG_DATA')
end
