function tessdata = locateTessdataFolder(lang, useFastModel)
% Determine the location of the tessdata folder. When the support package
% is installed the support package's tessdata folder is used. Otherwise,
% the default tessdata folder is used.
%
% Input lang is 3-character code for language as returned by
% vision.internal.ocr.convertLanguageToAlias.

%   Copyright 2015-2023 The MathWorks, Inc.

%#codegen

coder.extrinsic('fullfile','eml_try_catch');

switch lang
    case {'equ','epo_alt','tgl'}
        % These languages are only supported in v3. The tessdata folder
        % is under tesseract-ocr.
        tessdataDir = fullfile('tesseract-ocr','tessdata');
    otherwise
        
        % v5 models are directly under either tessdata_best or
        % tessdata_fast.
        if useFastModel
            tessdataDir = 'tessdata_fast';
        else
            tessdataDir = 'tessdata_best';
        end
end

if isdeployed && ~vision.internal.ocr.isCodegen()
    tlbxroot = coder.internal.const(tbxprefix);
else
    tlbxroot = coder.internal.const(fullfile(matlabroot,'toolbox'));
end

if contains(lang,'eng') || contains(lang,'jpn') || contains(lang,'seven_segment')
    % Use the model data that ships in CVT.
    tessdata = coder.internal.const(...
        fullfile(tlbxroot,'vision','visionutilities',tessdataDir));

elseif vision.internal.ocr.ocrSpkgInstalled()
    
    if vision.internal.ocr.isCodegen() 
       [~,~,spkgLocation] = eml_const(eml_try_catch(... 
           'vision.internal.ocr.getTessdataSupportPackageLocation', lang));          
    else
        spkgLocation = vision.internal.ocr.getTessdataSupportPackageLocation(lang);        
    end

    tessdata = coder.internal.const(fullfile(spkgLocation,tessdataDir));
end

% Tesseract requires that the path must end with a filesep
tessdata = [tessdata localFilesep];

tessdata = addDoubleFilesepOnPC(tessdata);

% -------------------------------------------------------------------------
% Return a filesep based on the current platform. Code generation only
% supports filesep on MEX and SFUN targets. For other targets, the unix
% style / is returned.
% -------------------------------------------------------------------------
function fs = localFilesep

if isempty(coder.target)
    fs = filesep;
else
    if strcmp(coder.target,'sfun') || strcmp(coder.target,'mex')
        % ispc only support on host
        if ispc
            fs = '\';
        else
            fs = '/';
        end
    else
        % use unix style filesep for non-host
        fs = '/';
    end
end
    
% -------------------------------------------------------------------------
% Tesseract requires that language fileseps are doubled (\ -> \\) on PCs
% -------------------------------------------------------------------------
function out = addDoubleFilesepOnPC(p)

if isempty(coder.target) || ...
        strcmp(coder.target,'sfun') || ...
        strcmp(coder.target,'mex')
    if ispc
        fs  = localFilesep;
        isUNCPath = contains(p(1:2),'\\');       
        if isUNCPath
            out = strrep(p(3:end),fs,[fs fs]);
            out = ['\\' out];
        else
            out = strrep(p,fs,[fs fs]);
        end
    else
        out = p;
    end
else
    out = p;
end
