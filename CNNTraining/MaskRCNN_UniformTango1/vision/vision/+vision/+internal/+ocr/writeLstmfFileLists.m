function numFiles = writeLstmfFileLists(files, shuffle, filename)
    % vision.internal.ocr.writeLstmfFileLists(image, boxFile, lstmfFileBasename) 
    % creates a text file, filename, that contains the names listed in
    % the cell array, files. It also optionally shuffles the order of names
    % written in filename. Valid values for shuffle are "once" and "never".
    
    % Copyright 2022 The MathWorks, Inc.

    % Create the file with filename. Tesseract requires using UTF-8 encoding.
    fid = fopen(filename, 'w', 'native','UTF-8');
    if (fid < 0)
        error(message('MATLAB:imagesci:imwrite:fileOpen', filename));
    end
    closeFile = onCleanup(@()fclose(fid));
    
    % Compute the number of files in the list.
    numFiles = size(files, 1);
    
    % Shuffle file order.
    if shuffle == "once"
        order = randperm(numFiles);
    elseif shuffle == "never"
        order = 1:numFiles;
    end
    
    % Write data into file.
    for i = order
        fprintf(fid,'%s\n', files{i});
    end
end