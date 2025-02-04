function lstmfFile = generateLSTMFFile(image, boxFile, lstmfFileBasename)
    % vision.internal.ocrgenerateLSTMFFile(image, boxFile, lstmfFileBasename) 
    % creates an LSTMF file from the box data in boxFile. boxFile can be a 
    % full or relative path to a box file and  must include the .box extension. 
    % The name of the LSTMF file is specified in lstmfFileBasename and should 
    % not include the .lstmf extension.
    
    %   Copyright 2022-2023 The MathWorks, Inc.

    % Convert image to uint8 (including binary images to allow tesseract to
    % pre-process the image).
    Iu8 = im2uint8(image);    
    Igray = vision.internal.ocr.convertRGBToGray(Iu8);
    
    useFastModel = false;
    language = 'English'; % Tesseract needs the language set even though it does 
                          % not use it.
    tessOpts.lang          = vision.internal.ocr.convertLanguageToAlias(language);
    tessOpts.tessdata      = vision.internal.ocr.locateTessdataFolder(tessOpts.lang, useFastModel);
    tessOpts.setVariable   = getTesseractGlobalVariables();
    tessOpts.initVariable  = []; % No variable initialization needed.
    tessOpts.ocrEngineMode = 1; % Neural nets LSTM engine only.
    
    tesseractGenerateLSTMFFile(tessOpts, Igray, boxFile, lstmfFileBasename);

    lstmfFile = [lstmfFileBasename '.lstmf'];
end

%--------------------------------------------------------------------------
function variables = getTesseractGlobalVariables()
    % Define tesseract global variables to configure for training. These
    % parameters were taken from the lstm.train file in tesseract's main
    % repository tesseract/tessdata/configs/lstm.train.

    variables.tessedit_pageseg_mode          = '13'; 
    variables.file_type                      = '.bl';
    variables.textord_fast_pitch_test        = 'T';
    variables.tessedit_zero_rejection        = 'T';
    variables.tessedit_minimal_rejection     = 'F';
    variables.tessedit_write_rep_codes       = 'F';
    variables.edges_children_fix             = 'F';
    variables.edges_childarea                = '0.65';
    variables.edges_boxarea                  = '0.9';
    variables.tessedit_train_line_recognizer = 'T';
    variables.textord_no_rejects             = 'T';
    variables.tessedit_init_config_only      = 'T';
end