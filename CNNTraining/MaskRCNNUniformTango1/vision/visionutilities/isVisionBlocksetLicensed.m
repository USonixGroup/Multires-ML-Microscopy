function [ value, msg ] = isVisionBlocksetLicensed( operation )
    %

    % Copyright 2015-2017, The Mathworks, Inc.
    if nargin > 0
        operation = convertStringsToChars(operation);
    end
    
    value = dig.isProductInstalled( 'Computer Vision Toolbox' );
    msg = '';
    if strcmpi( operation, 'checkout' ) && ~value
       msg = 'Failed to checkout Video and Image Blockset license.'; 
    end    
end