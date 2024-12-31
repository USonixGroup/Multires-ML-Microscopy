function [GNUC_FLAG, MSCVER_FLAG] = compilerBuildInfo()
% vision.internal.buildable.compilerBuildInfo() returns the
% GNUC version and MSC version of the compiler

% Copyright 2024 The MathWorks, Inc.

GNUC_Value = 0;
MSCVER_Value = 0;
GNUC_FLAG_FORMAT = '__GNUC__=%d';
MSCVER_FLAG_FORMAT = '_MSC_VER=%d';
GNUC_FLAG = '__GNUC__=12';
MSCVER_FLAG = '_MSC_VER=1936';

[~, currComp] = evalc('mex.getCompilerConfigurations(''c++'', ''Selected'')');
overrideCompilerFlags = regexprep(currComp.Details.CompilerFlags, '[-\/]std=[^\s]+', '');
frontEndOptions = internal.cxxfe.util.getMexFrontEndOptions('lang', 'c++', ...
    'addMWInc', true, ...
    'overrideCompilerFlags', overrideCompilerFlags);

% Convert cxxfe frontend options to compiler name and version.
compilerInfo = struct(...
    'language', 'c', ...
    'compiler', '',...
    'compilerVersion', 0);

if strncmp(frontEndOptions.Language.LanguageMode, 'cxx', 3)
    compilerInfo.language = 'c++';
else
    compilerInfo.language = 'c';
end

compVerOpt = '';
if ~isempty(find(strcmp(frontEndOptions.Language.LanguageExtra, '--gcc'), 1)) || ...
        ~isempty(find(strcmp(frontEndOptions.Language.LanguageExtra, '--g++'), 1))
    compilerInfo.compiler = 'gcc';
    compVerOpt = '--gnu_version';
elseif ~isempty(find(strcmp(frontEndOptions.Language.LanguageExtra, '--microsoft'), 1))
    compilerInfo.compiler = 'msvc';
    compVerOpt = '--microsoft_version';
end
if ~isempty(compVerOpt)
    idx = find(strncmp(frontEndOptions.Language.LanguageExtra, compVerOpt, numel(compVerOpt)), 1, 'last');
    if ~isempty(idx)
        compVer = regexp(frontEndOptions.Language.LanguageExtra{idx}, [compVerOpt, '=(\d+)'], 'tokens', 'once');
        if numel(compVer)==1
            compilerInfo.compilerVersion = sscanf(compVer{1}, '%d');
        elseif numel(frontEndOptions.Language.LanguageExtra) > idx
            compilerInfo.compilerVersion = sscanf(frontEndOptions.Language.LanguageExtra{idx+1}, '%d');
        end
    end
end

if(contains(compilerInfo.compiler,'msvc'))
    MSCVER_Value = compilerInfo.compilerVersion;
    MSCVER_FLAG = sprintf(MSCVER_FLAG_FORMAT, MSCVER_Value);
elseif(contains(compilerInfo.compiler,'gcc'))
    GNUC_Value = floor(compilerInfo.compilerVersion/10000);
    GNUC_FLAG = sprintf(GNUC_FLAG_FORMAT, GNUC_Value);
end

end