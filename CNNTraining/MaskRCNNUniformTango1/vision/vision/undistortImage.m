function [J, newIntrinsics] = undistortImage(I, intrinsics, varargin)

%   Copyright 2014-2023 The MathWorks, Inc.

%#codegen
%#ok<*EMCLS>
%#ok<*EMCA>

vision.internal.inputValidation.checkIntrinsicsAndParameters( ...
    intrinsics, true, mfilename, 'intrinsics');

[interp, outputView, fillValues] = parseInputs(I, varargin{:});

vision.internal.inputValidation.checkImageSize(I, intrinsics.ImageSize);

originalClass = class(I);
if ~(isa(I,'double') || isa(I,'single') || isa(I,'uint8'))
    I = single(I);
    fillValues = cast(fillValues, 'like', I);
end    

[J, newOrigin] = undistortImageImpl(intrinsics, I, interp, outputView, fillValues);
newPrincipalPoint = intrinsics.PrincipalPoint - newOrigin;
newImageSize = size(J,1:2);
J = cast(J, originalClass);

newIntrinsics = cameraIntrinsics(intrinsics.FocalLength, newPrincipalPoint, ...
    newImageSize, 'Skew', intrinsics.Skew);


%--------------------------------------------------------------------------
function [interp, outputView, fillValues] = parseInputs(I, varargin)
vision.internal.inputValidation.validateImage(I);
if isempty(coder.target)
    [interp, outputView, fillValues] = parseInputsMatlab(I, varargin{:});
else 
    [interp, outputView, fillValues] = ...
        vision.internal.inputValidation.parseUndistortRectifyInputsCodegen(...
        I, 'undistortImage', 'same', varargin{:});
end

fillValues = vision.internal.inputValidation.scalarExpandFillValues(...
    fillValues, I);

%--------------------------------------------------------------------------
function [interp, outputView, fillValues] = parseInputsMatlab(I, varargin)
defaultOutputView = 'same';
defaultInterp = 'bilinear';

persistent parser;

if isempty(parser)
    parser = inputParser();
    parser.addOptional('interp', defaultInterp, @validateInterpMethod);
    parser.addParameter('OutputView', defaultOutputView, @validateOutputView);
    parser.addParameter('FillValues', 0);
end

parser.parse(varargin{:});
interp = parser.Results.interp;
if ~strcmp(interp, defaultInterp)
    interp = vision.internal.inputValidation.validateInterp(interp);
end

outputView = parser.Results.OutputView;
if ~strcmp(outputView, defaultOutputView)
    outputView = validateOutputViewPartial(outputView);
end

fillValues = parser.Results.FillValues;
if ~(isscalar(fillValues) && fillValues == 0)
    vision.internal.inputValidation.validateFillValues(fillValues, I);
end

%--------------------------------------------------------------------------
function TF = validateOutputView(outputView)
validateattributes(outputView, {'char','string'}, {'vector'}, mfilename, 'OutputView');
TF = true;
        
%--------------------------------------------------------------------------
function tf = validateInterpMethod(method)
vision.internal.inputValidation.validateInterp(method);
tf = true;

%--------------------------------------------------------------------------
function outputView = validateOutputViewPartial(outputView)
outputView = ...
   validatestring(outputView, {'full', 'valid', 'same'}, mfilename, 'OutputView');

