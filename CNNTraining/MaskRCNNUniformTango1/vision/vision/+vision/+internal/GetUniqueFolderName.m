function newFolder = GetUniqueFolderName(folderName)
% GetUniqueFolderName returns a new non-existant folder location given a folder
% path. If the folder-in exists, the next numeric value is appended to it to 
% make it unique.

    if(exist(folderName, 'dir'))
        [currPath, currFolderName] = fileparts(folderName);
        currPath = string(currPath);
        currFolderName = string(currFolderName);
        idx = 1;

        while(exist(fullfile(currPath, currFolderName+num2str(idx)), 'dir'))
            idx = idx+1;
        end

        newFolder = fullfile(currPath, currFolderName+num2str(idx));
    else
        newFolder = folderName;
    end

end

