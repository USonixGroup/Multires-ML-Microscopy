classdef ObjectDetectorDatastoreException < MException
    % ObjectDetectorDatastoreException provides additional datastore
    % filename details on top of the actual error that occurred during
    % detection.
    
    % Copyright 2021 The MathWorks, Inc.

    methods 
        function this = ObjectDetectorDatastoreException(actualError, dsInfo, dsRecordIdx)
            % actualError - An exception or a message ID for the actual
            % error that occurred while processing a particular image.
            %
            % dsInfo - The info returned by the datastore read method.
            %
            % dsRecordIdx - The index of the datastore record being
            % processed when the actualError was caught.
            %
            % Example
            % -------
            %
            % [data, info] = read(ds);
            % for i = 1:size(data,1)
            %    try
            %       process(data(i,:));
            %    catch ME
            %       exp = ObjectDetectorDatastoreException(...
            %              ME, info, i);
            %       throw(exp);
            %     end
            % end

            filename = iFindFilenameWithinInfo(dsInfo, dsRecordIdx);

            if ~isempty(filename)
                % Create the base message using the filename.
                baseMsg =  message('vision:ObjectDetector:genericBaseMessageWithFilename', filename);

            else
                % No extra file information is available. Create a generic
                % base message.
                baseMsg =  message('vision:ObjectDetector:genericBaseMessageWithoutFilename');
            end

            % Replace \ with \\ to avoid printf warning about using invalid
            % format strings. 
            str = strrep(getString(baseMsg),'\','\\');
            
            this = this@MException(baseMsg.Identifier, str);

            if ischar(actualError)
                % Create an exception from the actual message ID. 
                actualError = MException(actualError, ...
                    getString(message(actualError)));
            end

            % Add the actual error as a cause to the base message. 
            this = addCause(this, actualError);
        end
    end

end

%--------------------------------------------------------------------------
function filename = iFindFilenameWithinInfo(dsInfo, dsRecordIdx)
if ~iscell(dsInfo)
    dsInfo = {dsInfo};
end

% dsInfo is a cell of struct arrays or a struct array. For imageDatastore
% and other MathWorks authored datastores, the Filename field contains a
% cell array of file names that were returned by read. We use dsRecordIdx
% to pull out the file being processed when the error occurred. 
filename = '';
try
    for i = 1:numel(dsInfo)
        s = dsInfo{i};
        for j = 1:numel(s)
            if isfield(s(j), 'Filename')
                filename = string(s(j).Filename);
                filename = filename(dsRecordIdx);
                break
            end
        end
    end
catch
    % Return an empty filename in case there was an error while trying to
    % look through the info data.
    filename = '';
end
end

% LocalWords:  datastores
