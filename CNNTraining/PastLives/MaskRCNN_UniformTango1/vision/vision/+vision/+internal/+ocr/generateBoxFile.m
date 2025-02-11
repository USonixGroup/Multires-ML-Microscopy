function [boxFilename, img] = generateBoxFile(boxFileBasename, bbox, gtruth, I)
    % vision.internal.ocr.generateBoxFile(boxFileBasename, bbox, gtruth, I)
    % creates a BOX file from the input bounding box and ground truth.
    % boxFileBasename is the full or relative path of the output box file
    % with base name without any extension. boxFilename is boxFileBasename
    % with .box extension. It additionally returns the image, I, cropped to
    % the bounding box, bbox in img.
    
    % Copyright 2022 The MathWorks, Inc.

    % Append the extension to the base name.
    boxFilename = [boxFileBasename '.box'];
    
    % Apply bounding box on the image.
    img = imcrop(I, bbox);

    % Write box file on the cropped image.
    writeBoxFile(boxFilename, gtruth, size(img));
end

%--------------------------------------------------------------------------
function writeBoxFile(boxFilename, chars, sz)

    % Create the file with filename. Tesseract requires using UTF-8 encoding.
    fid = fopen(boxFilename, 'w', 'native','UTF-8');
    if (fid < 0)
        error(message('MATLAB:imagesci:imwrite:fileOpen', filename));
    end
    closeFile = onCleanup(@()fclose(fid));
    
    % Define the bounding box.
    left   = 0;
    bottom = 0;
    right  = sz(2);
    top    = sz(1);
    page = 0;
    
    % Write data into box file.
    numChars = size(chars, 2);
    for i = 1:numChars
        fprintf(fid,'%s %d %d %d %d %d\n', ...
            chars(i), left, bottom, right, top, page);
    end

    % Insert tab character instead of newline as the end of line character.
    fprintf(fid,'\t %d %d %d %d %d\n', left, bottom, right, top, page);
end
