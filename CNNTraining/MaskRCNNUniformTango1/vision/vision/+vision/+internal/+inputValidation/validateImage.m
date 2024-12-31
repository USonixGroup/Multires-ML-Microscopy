function validateImage(I, varName, imageType)
% Image validation function. The input image I can be logical, uint8,
% int16, uint16, single, or double, and it must be real and nonsparse. By
% default, the validation function also validates that I is only 2D or 3D
%
% validateImage(I,varName, imageType) checks that the image is either 'rgb'
% or 'grayscale'.

%   Copyright 2013-2020 The MathWorks, Inc.

%#codegen
%#ok<*EMTC>
%#ok<*EMCA>

if nargin < 2
    varName = 'Image';
end
isMultiChannel = false;
if nargin == 3                  
    switch imageType
        case 'grayscale'    
            imageSizeAttribute = [NaN NaN];
        case 'rgb'        
            imageSizeAttribute = [NaN NaN 3];
        case 'multi-channel'
            imageSizeAttribute = [NaN NaN NaN];
            isMultiChannel = true;
        otherwise
            error('Unknown image type');
    end
else
    imageSizeAttribute = [NaN NaN NaN];
end    

if isempty(coder.target)
    % use try/catch to throw error from calling function. This produces an
    % error stack that is better associated with the calling function.
    try 
        localValidate(I, varName, imageSizeAttribute, isMultiChannel)
    catch E        
        throwAsCaller(E); % to produce nice error message from caller.
    end
else
    localValidate(I, varName, imageSizeAttribute, isMultiChannel);
end

%--------------------------------------------------------------------------
function localValidate(I, varName, imageSizeAttribute, isMultiChannel)
classAttributes = {'double','single','int16','uint16','uint8','logical'};

validateattributes(I, classAttributes,...
    {'nonempty','real', 'nonsparse'},...
    'validateImage', varName);

% No objects allowed except gpuArrays
if coder.target('MATLAB') && ~isa(I, 'gpuArray') 
    vision.internal.inputValidation.validateNotObject(I, 'validateImage', varName);
end

n  = ndims(I);
sz = size(I);

if numel(imageSizeAttribute) == 2
    % must be 2D
    coder.internal.errorIf( n~=2, 'vision:dims:imageNot2D');
else
    
    if imageSizeAttribute(3) == 3 % must be RGB
        
        coder.internal.errorIf( n ~= 3 || sz(3) ~= 3, ...
            'vision:dims:imageNotRGB');
        
    else % size must be 2D or 3D                        
                
        coder.internal.errorIf( n < 2 || n > 3, ...
            'vision:dims:imageNot2DorRGB')
        
        if ( n == 3 && ~isMultiChannel)
            % check that 3rd dim for RGB is 3
            coder.internal.errorIf( sz(3) ~= 3, ...
                'vision:dims:imageNot2DorRGB');
        end
        
    end
end
