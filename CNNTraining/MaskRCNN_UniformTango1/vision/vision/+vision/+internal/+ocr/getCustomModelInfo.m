function [tessdata, model] = getCustomModelInfo(modelName)
% Return the path to the tessdata folder and the model alias.

% Copyright 2022-2023 The MathWorks, Inc.
%#codegen
    
% modelName contains a validated file path to the Tesseract
% model data file. The expected format is
%
%    'path/to/tessdata/foo.traineddata' for legacy engine.
%
%    'path/to/foo.traineddata' for latest engine.
%   
% where foo is the model name.

    if iscell(modelName)
        model    = combineModelNames(modelName);

        % Return the location of the tessdata folder from the path to a
        % custom model data file.
        tessdata = getFileParts(modelName{1});
    else      
        model    = getModelFromPath(modelName);

        % Return the location of the tessdata folder from the path to a
        % custom model data file.
        tessdata = getFileParts(modelName);
    end    
end

% -------------------------------------------------------------------------
function model = combineModelNames(modelNames)
    
    model = cell(size(modelNames));
    for i = 1:numel(modelNames)
        model{i} = getModelFromPath(modelNames{i});
    end

    if numel(model) > 1
        % multiple models take the form "model1+model2+..."
        model = strjoin(model,'+');
    else
        model = model{1};
    end
end

% -------------------------------------------------------------------------
% Return model alias from the path to a custom model data file.
%--------------------------------------------------------------------------
function model = getModelFromPath(datapath)

    [~, model, ext] = getFileParts(datapath);
    
    if ~strcmpi(ext, '.traineddata')
        model = '';
    end
end

% -------------------------------------------------------------------------
% A naive implementation of codegenable fileparts function from base MATLAB.
% -------------------------------------------------------------------------
function [pathstr, name, ext] = getFileParts(inputFile)
    
    file = char(inputFile);
    
    % Find file separator.
    if contains(file, '/')
        fileseparator = '/';
    else
        fileseparator = '\';
    end
    
    % Find path and filename.
    indices = strfind(file, fileseparator);
    if isempty(indices)
        pathstr = '.';
        filename = file;
    else
        index = indices(end);
        pathstr = file(1:index); % PATHSTR/
        filename = file(index+1:end); % FILENAME.EXT
    end

    % Find file extension.
    indices = strfind(filename, '.');
    if isempty(indices)
        ext = '';
        name = filename;
    else
        index = indices(end);
        name = filename(1:index-1); % FILENAME
        ext = filename(index:end); % .EXT
    end

    if isstring(inputFile)
        ext = string(ext);
        name = string(name);
        pathstr = string(pathstr);
    end
end