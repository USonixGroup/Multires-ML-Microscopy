classdef ocrCodegenUtilities
    % ocrCodegenUtilities - Contains all the methods common for sharedlib
    % and portable versions of ocr codegen

    % Copyright 2024 The MathWorks, Inc.
    %#codegen
    methods(Static)
        % -------------------------------------------------------------------------
        % Convert UTF-8 encoded text into ASCII. Multibyte characters are truncated
        % to char(127).
        % -------------------------------------------------------------------------
        function asciiText = utf8ToAscii(utf8Text)
            coder.inline('never');
            idx = findCharacters(utf8Text);

            asciiText = utf8Text(idx);
            asciiText(asciiText>uint8(127)) = uint8(127);
        end

        % -------------------------------------------------------------------------
        % Null-terminate a string.
        % -------------------------------------------------------------------------
        function strout = nullTerminateString(strin)
            coder.inline('always');
            strout = [strin uint8(0)];
        end
    end
end
% -------------------------------------------------------------------------
% Returns indices to the start of characters in a UTF-8 encoded string,
% skipping over UTF-8 continuation bytes. The number of characters is also
% returned.
% -------------------------------------------------------------------------
function [idx, count, invalidIndex] = findCharacters(utf8String)
coder.inline('never');
bytesToProcess = length(utf8String);

idx = false(1,bytesToProcess);

invalidIndex = false(1,bytesToProcess);

i = 1;
count = 0;

while bytesToProcess

    count  = count + 1;
    idx(i) = true;

    if isASCIICompatiable(utf8String(i))

        i = i + 1;
        bytesToProcess = bytesToProcess - 1;

    elseif isUTF8ContinuationByte(utf8String(i))

        invalidIndex(i) = true;
        i = i + 1;
        bytesToProcess = bytesToProcess - 1;

    elseif isStartOfUTF8MultiByteCharacter(utf8String(i))

        numBytes = numBytesInSequence(utf8String(i));

        % skip over the multi-byte sequence
        i = i + numBytes;
        bytesToProcess = bytesToProcess - numBytes;

    else
        invalidIndex(i) = true;
        i = i + 1;
        bytesToProcess = bytesToProcess - 1;
    end
end

end

% -------------------------------------------------------------------------
% Return true if the UTF-8 byte holds an ASCII compatible value.
% -------------------------------------------------------------------------
function tf = isASCIICompatiable(utf8Byte)
coder.inline('always');
tf = utf8Byte < uint8(128) ;
end

% -------------------------------------------------------------------------
% Return true if the UTF-8 byte is a continuation byte (10XX XXXX)
% -------------------------------------------------------------------------
function tf = isUTF8ContinuationByte(utf8Byte)
coder.inline('always');
tf =  utf8Byte >= uint8(128) && utf8Byte < uint8(192);
end

% -------------------------------------------------------------------------
% Return true if byte is the start of a multi-byte UTF-8 character.
% -------------------------------------------------------------------------
function tf = isStartOfUTF8MultiByteCharacter(utf8Byte)
coder.inline('always');
tf = utf8Byte >= uint8(192) && utf8Byte <= uint8(252);
end

% -------------------------------------------------------------------------
% Return the number of bytes in a multi-byte UTF-8 character by inspecting
% the starting byte of the sequence.
% -------------------------------------------------------------------------
function n = numBytesInSequence(utf8Byte)
coder.inline('always');
n = 1;
while bitand(bitshift(uint8(utf8Byte),n),uint8(128))
    n = n + 1; % tells us how many bytes are in the multi-byte char
end
assert(n<7);  % invalid UTF-8 start byte
end