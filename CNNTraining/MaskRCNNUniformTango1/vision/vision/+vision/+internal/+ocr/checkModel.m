function [model, isCustomModel] = checkModel(userModel, parameterName, doAcceptMultipleModels, canUseFastModel)
% Check OCR Model name-value.

% Copyright 2022 The MathWorks, Inc.

%#codegen

if doAcceptMultipleModels && iscell(userModel)
    validateattributes(userModel,{'cell'},{'vector','row'}, ...
        mfilename, parameterName);
else
    validateattributes(userModel,{'string','char'},{'vector','row','scalartext'}, ...
        mfilename, parameterName);
end

coder.extrinsic('exist','filesep');
if ischar(userModel) || isstring(userModel)

    if vision.internal.ocr.isCodegen()
        % compile time error if  Model is not constant value.
        eml_invariant(eml_is_const(userModel), ...
            eml_message('vision:ocr:codegenModelMustBeConstant'));

        modifiedModel = userModel;
    else
        % fix up filesep for current platform. This converts PC style \ to
        % unix style / or vice versa. This prevents failures when loading
        % tesseract data files.
        modifiedModel = fixFilesep(userModel);
    end

    isCustomModel = checkIfCustomModel(modifiedModel);

    if ~isCustomModel

        model = vision.internal.ocr.validateLanguage(modifiedModel, vision.internal.ocr.ocrSpkgInstalled(), canUseFastModel);
    else

        model = modifiedModel;
        if vision.internal.ocr.isCodegen()
            coder.internal.errorIf(vision.internal.codegen.exist(model) ~= 2,...
                'vision:ocr:languageDataFileNotFound', model);
        else
            coder.internal.errorIf(exist(model,'file') ~= 2,...
                'vision:ocr:languageDataFileNotFound', model);
        end
    end
else % cell array of languages strings

    % check custom language strings. When multiple custom languages are
    % specified, they must all be custom languages (i.e. 'English' and a
    % custom language is not allowed).
    
    isCustomModel = cellfun(@(x)checkIfCustomModel(x),userModel);
       
    isMixedCustomAndNonCustomLanguages = any(isCustomModel) && ~all(isCustomModel);
    
    if isMixedCustomAndNonCustomLanguages        
        error(message('vision:ocr:customAndNonCustom'));
    end

    % 'seven-segment' language cannot be combined with any other language.
    if any(strcmp(userModel, 'seven-segment'))
        error(message('vision:ocr:sevenSegmentMustBeSpecifiedAlone'));
    end
            
    if all(isCustomModel)
        
        % all had at least tessdata/*.traineddata
        isCustomModel = true;
               
        model   = fixFilesep(userModel);                
        pathstr = cell(numel(model),1);
        for i = 1:numel(model)
           pathstr{i} = fileparts(model{i});
        end
        
        % all language data files must be in the same tessdata folder
        if numel(unique(pathstr)) > 1
            error(message('vision:ocr:notUniqueLanguagePath'));
            % this will fail if one is a relative path and the other is a
            % full path to the same folder.
        end
        
        % All language data files must be accessible
        for i = 1:numel(model)
            if ~exist(model{i},'file')
                error(message('vision:ocr:languageDataFileNotFound',model{i}));
            end
        end
    else        
        % check non-custom language strings
        isSupportPackageInstalled = vision.internal.ocr.ocrSpkgInstalled();
        for i = 1:numel(userModel)            
            model{i} = vision.internal.ocr.validateLanguage(userModel{i}, isSupportPackageInstalled, canUseFastModel);
        end        
        isCustomModel = false;
    end
end
end

% -------------------------------------------------------------------------
function isCustomLang = checkIfCustomModel(lang)

if ~contains(lang, '.traineddata')
    isCustomLang = false;
else
    isCustomLang = true;
end
end

% -------------------------------------------------------------------------
function modifiedLang = fixFilesep(userModel)
% Fix filesep for current platform. This converts PC style \ to unix style
% / or vice versa. This prevents failures when loading tesseract data
% files.

if vision.internal.ocr.isCodegen()
    % this function is not used in codegen, but codegen requires outputs to
    % be assigned on all execution paths.
    modifiedLang = userModel;
else
    modifiedLang = regexprep(userModel,'[\/\\]',filesep);
end
end