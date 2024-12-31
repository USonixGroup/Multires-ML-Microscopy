function langList = ocrGetInstalledLanguageList()

    % Copyright 2018-2022 The MathWorks, Inc.

    %#codegen
    %List of languages installed in CVT
    langList = {'english','japanese','seven-segment'};

    %List of languages installed in spkg + CVT
    if vision.internal.ocr.ocrSpkgInstalled()
        spkgLangList = vision.internal.ocr.languagesInSupportPackage;
        langList = union(spkgLangList,langList,'sorted');
    end
end
