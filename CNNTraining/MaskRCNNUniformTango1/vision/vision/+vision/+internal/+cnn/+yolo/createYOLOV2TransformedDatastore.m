function ds = createYOLOV2TransformedDatastore(trainingDatastore, params)
%createYOLOV2TransformedDatastore Create Yolo V2 tranformed training datastore.
%

% Copyright 2019 The MathWorks, Inc.

    datastoreOutSize = iGetDatastoreOutSize(params);

    ds = transform(trainingDatastore,...
        @(data,info)iDoYoloAugmentation(data,info,datastoreOutSize),...
        'IncludeInfo',true);
end

function [data, info] = iDoYoloAugmentation(data, info, datastoreOutSize)

    for ii = 1:size(data,1)
        data(ii,2) = vision.internal.cnn.validation.checkTrainingBoxes(data(ii,1), data(ii, 2));
        data(ii,1:2) = yolov2ObjectDetector.trainingTransformForDatastore(data(ii,:),datastoreOutSize);
    end
end

function datastoreOutSize = iGetDatastoreOutSize(params)
    trainingImageSize = params.TrainingImageSize;
    datastoreOutSize  = params.DatastoreOutSize;
    multiScaleIndex   = randi([1 size(trainingImageSize,1)],1);
    datastoreOutSize  = [trainingImageSize(multiScaleIndex,:),datastoreOutSize(1,3)];
end
