function lang = validateLanguage(userLang, isSupportPackageInstalled, canUseFastModel)
% Validates languages strings against the language strings supported by ocr
% and those available in the OCR support package.

%   Copyright 2014-2022 The MathWorks, Inc.

%#codegen

% Codegen needs this to be char.
userLang = char(userLang);

% Determine whether the OCR Language support package is required.
n = numel(userLang);

% To avoid ambiguity between english and other languages that start with an
% 'e', 'en' is required for a partial match to 'english'
isEnglish  = n > 1 && strncmpi(userLang, 'english',n);
isEnglishFast  = n > 1 && strncmpi(userLang, 'english-fast',n);

isJapanese = strncmpi(userLang, 'japanese', n);
isJapaneseFast = strncmpi(userLang, 'japanese-fast', n);

% To avoid ambiguity between seven-segment and other languages that start 
% with a 'se', 'sev' is required for a partial match to 'seven-segment'
isSevenSegment = n > 2 && strncmpi(userLang,'seven-segment',n);
isSevenSegmentFast = n > 2 && strncmpi(userLang,'seven-segment-fast',n);

coder.extrinsic('eml_try_catch');

% Determine whether the language string is valid. First, english, japanese
% and seven-segment are checked as these are the built-in ocr languages. If 
% none of these is selected, then check against content of the OCR Support 
% Package.
if isEnglish || isEnglishFast
    lang = 'english';
elseif isJapanese || isJapaneseFast
    lang = 'japanese';
elseif isSevenSegment || isSevenSegmentFast
    lang = 'seven-segment';
else % check if in support package
    
    if isempty(coder.target)
        try  %#ok<EMTC>
            lang = vision.internal.ocr.validateSupportPackageLanguages(userLang, canUseFastModel);
            inSupportPackage = true;
        catch
            lang = '';
            inSupportPackage = false;
        end
        
        % Error out if language is in the support package, but the support
        % package is not installed.
        if inSupportPackage && ~isSupportPackageInstalled
            msg = message('vision:ocr:requiresSupportPackage',userLang);
            str = getString(msg);
            error('vision:ocr:requiresSupportPackage',...
                '<a href="matlab:vision.internal.ocr.ocrSupportPackage">%s</a>',str);
        end
        % Error out if the language is not in the support package.
        if ~inSupportPackage
            %langs = getFormattedLanguageStrings();
            msg1  = message('vision:ocr:languagesInSupportPackage');
            msg2  = message('vision:ocr:languagesInSupportPackageDisp');
            if canUseFastModel
                error('vision:ocr:languagesInSupportPackage',...
                    '%s <a href="matlab:disp(vision.internal.ocr.formatLanguages(false,true))">%s</a>',...
                    getString(msg1),...
                    getString(msg2));
            else
                error('vision:ocr:languagesInSupportPackage',...
                    '%s <a href="matlab:disp(vision.internal.ocr.formatLanguages(false,false))">%s</a>',...
                    getString(msg1),...
                    getString(msg2));
            end
        end
    else        
        [~,~,langVal] = eml_const(eml_try_catch(...
            'vision.internal.ocr.validateSupportPackageLanguages',userLang, canUseFastModel));      
        
        if isempty(langVal)
            % An error was thrown evaluating
            % validateSupportPackageLanguages this means userLang was not
            % one of the support package languages.            
            lang = '';
            inSupportPackage = false;
        else            
            lang = langVal;
            inSupportPackage = true;           
        end        
               
        % Error out if language is in the support package, but the support
        % package is not installed.
        if inSupportPackage && ~isSupportPackageInstalled            
            % use eml_invariant to force compile time error message
            % (coder.internal.errorIf throws a runtime error), which is not
            % desirable here.            
            eml_invariant(0, ...
                eml_message('vision:ocr:requiresSupportPackage',userLang));
        end
        
        % Error out if the language is not in the support package.
        if ~inSupportPackage
            langs = getFormattedLanguageStrings(canUseFastModel);
            eml_invariant(0, ...
                eml_message('vision:ocr:languagesInSupportPackageCodegen',langs));
        end
    end                     
end

%--------------------------------------------------------------------------
function str = getFormattedLanguageStrings(canUseFastModel)
coder.extrinsic('eml_try_catch');

if isempty(coder.target)
    str = vision.internal.ocr.formatLanguages(false,canUseFastModel);
else    
    [~,~,str] = eml_const(eml_try_catch('vision.internal.ocr.formatLanguages',true, canUseFastModel));
end
