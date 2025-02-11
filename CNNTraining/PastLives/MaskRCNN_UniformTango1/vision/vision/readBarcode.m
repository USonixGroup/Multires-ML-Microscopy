function [msg, detectedFormat, loc] = readBarcode(I, varargin)

% Copyright 2019-2024 The MathWorks, Inc.

%#codegen

[roi, formats] = parseInputs(I, varargin{:});

% Convert to grayscale
if ~ismatrix(I)
    imgGray = rgb2gray(I);
else
    imgGray = I;
end

% Ensure image data is uint8
if ~isa(imgGray, 'uint8')
    imgUint8 = im2uint8(imgGray);
else
    imgUint8 = imgGray;
end

[msg, detectedFormat, loc] = zxingBarcodeReader(imgUint8, roi, formats);
end
%
%--------------------------------------------------------------------------
% parseInputs - Parse the input arguments
%--------------------------------------------------------------------------
function [roi, formats] = parseInputs(I, varargin)

vision.internal.inputValidation.validateImage(I);
roi = zeros(0,4);

if nargin == 1
    formats = 'all';
elseif nargin == 2
    if isnumeric(varargin{1}) % readBarcode(I, roi)
        roi = varargin{1};
        formats = 'all';
    else % readBarcode(I, format)
        formats = varargin{1};
    end
elseif nargin == 3
    roi = varargin{1};
    formats = varargin{2};
end

vision.internal.detector.checkROI(roi, size(I));

formats = checkBarcodeFormats(formats);
formats = formatFamilyToList(formats);

% non-zero row vector inputs cannot be handled with 'AZTEC' format
if isSimMode()
    if (ischar(formats) && (strcmp(formats,'all') || strcmp(formats,'AZTEC'))) ...
            || (iscellstr(formats) && ismember('AZTEC', formats))

        coder.internal.errorIf((size(I,1)==1 && size(I,2)>1 && all(any(I ~= 0))),...
            'vision:zxing:rowVectorInput');
    end
else

    if (ischar(formats) && (strcmp(formats,'all') || strcmp(formats,'AZTEC')))
        coder.internal.errorIf((size(I,1)==1 && size(I,2)>1 && all(any(I ~= 0))),...
            'vision:zxing:rowVectorInput');
    elseif iscellstr(formats)
        aztecFormat = 0;
        for i = 1:size(formats,2)
            if strcmp(formats{i},'AZTEC') || strcmp(formats{i}, 'all')
                aztecFormat = 1;
            end
        end
        coder.internal.errorIf((aztecFormat && size(I,1)==1 && size(I,2)>1 && all(any(I(:) ~= 0))),...
            'vision:zxing:rowVectorInput');
    end
end

end


%--------------------------------------------------------------------------
% zxingBarcodeReader - Call ZXing for barcode detection and decoding
%--------------------------------------------------------------------------
function [msg, format, loc] = zxingBarcodeReader(I, roi, formats)

tryHarderOneD = true;
tryHarderTwoD = true;


if isempty(roi)
    % Call to builtin with entire image. The image needs to be transposed
    % since the builtin expects the data to be row-major.
    if isSimMode()
        [msg, format, loc] = vision.internal.zxingMultiFormatReader(I.', formats, tryHarderOneD, tryHarderTwoD);
    elseif coder.internal.preferMATLABHostCompiledLibraries()        
        [msg, format, loc] = vision.internal.buildable.readBarcodeBuildable.readBarcodeMultiReader(I.', formats, tryHarderOneD, tryHarderTwoD);
    else        
        [msg, format, loc] = vision.internal.buildable.readBarcodeBuildablePortable.readBarcodeMultiReader(I.', formats, tryHarderOneD, tryHarderTwoD);
    end
    loc = loc + 1; % ZXing uses zero-indexing for images
else
    % Get ROI image
    Iroi = vision.internal.detector.cropImage(I, roi);

    % Call to builtin with ROI image. The image needs to be transposed
    % since the builtin expects the data to be row-major.
    if isSimMode()
        [msg, format, loc] = vision.internal.zxingMultiFormatReader(Iroi.', formats, tryHarderOneD, tryHarderTwoD);

        % Location of barcode in original image
        loc = loc + double(roi(1:2)) + 1; % ZXing uses zero-indexing for images
    else
        if coder.internal.preferMATLABHostCompiledLibraries() 
            [msg, format, loc] = vision.internal.buildable.readBarcodeBuildable.readBarcodeMultiReader(Iroi.', formats, tryHarderOneD, tryHarderTwoD);
        else
            [msg, format, loc] = vision.internal.buildable.readBarcodeBuildablePortable.readBarcodeMultiReader(Iroi.', formats, tryHarderOneD, tryHarderTwoD);
        end

        % if loc is non-empty
        if size(loc,1) ~= 0

            % if num of detected barcodes > 1
            if size(loc,1) > size(roi,1)
                roiExp = zeros(size(loc));

                % concatenate rows for size match before adding
                roiExp = repmat(roi(1:2), [size(loc,1),1]);

                % Location of barcode in original image
                loc = loc + double(roiExp) + 1; % ZXing uses zero-indexing for images
            else
                % Location of barcode in original image
                loc = loc + double(roi(1:2)) + 1; % ZXing uses zero-indexing for images
            end
        end
    end
end

msg = string(msg);
format = string(format);

end

%--------------------------------------------------------------------------
% checkBarcodeFormats - Validate barcode formats
%--------------------------------------------------------------------------
function formats = checkBarcodeFormats(formats)
validateattributes(formats, {'char', 'string', 'cell'}, {'nonempty', 'vector'}, 'readBarcode', 'format');

% all supported formats for barcode detection
oneDFormats = getOneDFormats();
twoDFormats = getTwoDFormats();

numOneDFormats = size(oneDFormats,2);
numTwoDFormats = size(twoDFormats,2);

if ischar(formats) || isStringScalar(formats)
    if ~isSimMode()

        numValidFormats = 3 + numOneDFormats + numTwoDFormats;
        validFormatsOptsChar = cell(1, numValidFormats);

        validFormatsOptsChar{1} = '1D';
        validFormatsOptsChar{2} = '2D';
        validFormatsOptsChar{3} = 'all';

        for i = 1:numOneDFormats
            validFormatsOptsChar{i+3} = oneDFormats{i};
        end

        for i = 1:numTwoDFormats
            validFormatsOptsChar{i+3+numOneDFormats} = twoDFormats{i};
        end

        isValidFamily = any(strcmp(validFormatsOptsChar,formats));

        formatMsg = strjoin(validFormatsOptsChar, ',');
        coder.internal.errorIf(~isValidFamily,'vision:readBarcode:unrecognizedStringChoice', formatMsg);
    else
        validFormatsOptsChar = [{'1D'}, {'2D'}, {'all'}, oneDFormats(:)', twoDFormats(:)'];
        formats = validatestring(formats, validFormatsOptsChar, 'readBarcode', 'format');
    end
else
    coder.internal.errorIf(~iscellstr(formats) && ...
        ~isstring(formats),'vision:readBarcode:invalidStringList');

    if isSimMode()
        validFormatsOptsCellStr = [oneDFormats(:)', twoDFormats(:)'];
        isValidFormat = all(ismember(formats, validFormatsOptsCellStr));

    else
        combinedLen = numOneDFormats + numTwoDFormats;
        validFormatsOptsCellStr = cell(1,combinedLen);

        for i = 1:numOneDFormats
            validFormatsOptsCellStr{i} = oneDFormats{i};
        end

        for i = 1:numTwoDFormats
            validFormatsOptsCellStr{i + numOneDFormats} = twoDFormats{i};
        end

        count = 0;
        if iscellstr(formats)
            for i = 1:size(formats,2)
                count = count + any(strcmp(validFormatsOptsCellStr, formats{i}));
            end
            isValidFormat = (count == size(formats,2));
        else
            isValidFormat = any(strcmp(validFormatsOptsCellStr, formats));
        end
    end

    formatMsg = strjoin(validFormatsOptsCellStr, ', ');
    coder.internal.errorIf(~isValidFormat,'vision:readBarcode:unrecognizedStringChoice', formatMsg);

    if isSimMode()
        formats = cellstr(formats);
    end

end

end

%--------------------------------------------------------------------------
% formatFamilyToList - Generate formats list from input
%--------------------------------------------------------------------------
function formatList = formatFamilyToList(formats)

if ischar(formats) || isStringScalar(formats)
    if strcmp(formats, '1D')
        formatList = getOneDFormats();
    elseif strcmp(formats, '2D')
        formatList = getTwoDFormats();
    else
        if ~isSimMode()
            formatList = {char(formats)};
        else
            formatList = formats;
        end
    end
else
    if isSimMode()
        formatList = unique(formats, 'stable');
    else
        formatList = getUnique(formats);
    end
end
end

%--------------------------------------------------------------------------
% getOneDFormats - Get supported 1-D formats
%--------------------------------------------------------------------------
function formats = getOneDFormats()
formats = {'UPC-A', 'UPC-E', 'EAN-8', 'EAN-13', 'CODE-39', 'CODE-93', ...
    'CODE-128', 'CODABAR', 'ITF', 'RSS-14', 'RSS-EXPANDED'};
end

%--------------------------------------------------------------------------
% getTwoDFormats - Get supported 2-D formats
%--------------------------------------------------------------------------
function formats = getTwoDFormats()
formats = {'QR-CODE', 'DATA-MATRIX', 'AZTEC', 'PDF-417'};
end

%--------------------------------------------------------------------------
% isSimMode - check if simulation mode or codegen mode
%--------------------------------------------------------------------------
function out = isSimMode()
out = isempty(coder.target);
end

%--------------------------------------------------------------------------
% getUnique - remove duplicate formats mentioned in input, result always
% has unique format names
%--------------------------------------------------------------------------

function formatList = getUnique(cellFormats)
% to extract only the unique formats mentioned in the cell array.
num = size(cellFormats,2);
uniqueFormats = cellFormats;
flagInd = zeros(1,num);
c = 1;

% count number of times each format appears and their indexes
for i = 1:num

    item = cellFormats{i};
    ind = NaN(1,num);
    cnt = 1;
    for k = 1:num
        if isequal(item, cellFormats{k})
            ind(cnt) = k;
            cnt = cnt + 1;
        end
    end
    validmask = ~isnan(ind);

    % sort the indices
    ind = sort(ind(validmask));

    if length(ind) > 1
        % if the format occurs more than once
        for j = 2:length(ind)
            if ~(any(flagInd(:) == ind(j)))
                flagInd(c) = ind(j);
                c = c+1;
            end
        end
    end
end
count = 0;
for i = 1:num
    if ~(any(flagInd(:) == i))
        count = count + 1;
        uniqueFormats{count} = cellFormats{i};
    end
end

formatList = cell(1,count);
for i = 1:count
    formatList{i} = uniqueFormats{i};
end
end
