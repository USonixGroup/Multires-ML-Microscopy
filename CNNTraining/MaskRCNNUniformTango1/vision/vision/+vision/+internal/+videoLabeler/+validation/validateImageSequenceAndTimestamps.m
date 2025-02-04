function validateImageSequenceAndTimestamps(imgSequence,timestamps)
%validateImageSequenceAndTimestamps Validates image sequences and
%timestamps individually and checks consistency between them. imgSequence
%can be a cell array of character vectors specifying image file names
%(image files must be in the same directory) or imageDatastore and
%timestamps must be a duration or double vector


% Copyright 2016 The MathWorks, Inc.

import vision.internal.labeler.validation.*

assert((iscellstr(imgSequence) || isa(imgSequence,'matlab.io.datastore.ImageDatastore')) && ...
    (isduration(timestamps) || isa(timestamps,'double')), 'Unexpected inputs');

%Validate inputs individually and check consistency
validateTimestamps(timestamps);
validateImageSequence(imgSequence);
checkImageSequenceAndTimestampsAgreement(imgSequence,timestamps);
