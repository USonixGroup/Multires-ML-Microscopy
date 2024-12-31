function str = formatLanguages(isCodegen, canUseFastModel)
% Format language strings for error message display.

%   Copyright 2014-2022 The MathWorks, Inc.

%#codegen
langs = vision.internal.ocr.ocrGetInstalledLanguageList();


if ~vision.internal.ocr.ocrSpkgInstalled()
    LangsInSpkg = vision.internal.ocr.languagesInSupportPackage();
    langs = union(langs,LangsInSpkg,'sorted');
end

if canUseFastModel
    % Remove unsupported fast models.
    idx = ismember(langs,{'tagalog','esparantoalternative','mathequation'});
    langs(idx) = [];

    % Append -fast so all valid language model names are listed.
    langs = [langs strcat(langs,'-fast')];
end

if isCodegen
    % comma separated display for MATLAB coder report.
    str = sprintf('%s, ',langs{:}); %#ok<EMCA>
    str = str(1:end-2);
else
    % newline separated display for command window.
    str = sprintf('%s\n',langs{:}); %#ok<EMCA>
end
