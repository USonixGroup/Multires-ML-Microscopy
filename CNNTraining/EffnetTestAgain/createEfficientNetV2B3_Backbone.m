function net = createEfficientNetV2FeatureExtractor(inputSize)
    % Create EfficientNetV2-style network with GroupNorm that outputs a feature map
    % with 1/16 scale of the input and 1024 channels

    if nargin < 1
        inputSize = [512 512 3]; % Default input size
    end

    lgraph = layerGraph();

    % Initial stem block with stride 2 (1/2 scale)
    stemLayers = [
        imageInputLayer(inputSize, 'Name', 'input', 'Normalization', 'none')
        convolution2dLayer(3, 32, 'Stride', 2, 'Padding', 'same', 'Name', 'stem_conv', 'BiasLearnRateFactor', 0, 'BiasInitializer', 'zeros')
        groupNormalizationLayer(calculateNumGroups(32), 'Name', 'stem_gn')
        swishLayer('Name', 'stem_swish')
    ];

    lgraph = addLayers(lgraph, stemLayers);

    blockConfigs = {
        {'fused', 32,  2, 1, 1}
        {'fused', 64,  3, 2, 4}
        {'mb',    128, 3, 2, 4}
        {'mb',    256, 4, 2, 4}
        {'mb',    512, 3, 1, 4}
        {'mb',    1024, 1, 1, 2}
    };

    prevLayerName = 'stem_swish';

    for stageIdx = 1:length(blockConfigs)
        config = blockConfigs{stageIdx};
        blockType = config{1};
        channels = config{2};
        numLayers = config{3};
        stride = config{4};
        expansion = config{5};

        for blockIdx = 1:numLayers
            currentStride = 1;
            if blockIdx == 1
                currentStride = stride;
            end

            if strcmp(blockType, 'fused')
                [lgraph, prevLayerName] = addFusedMBConvBlock(lgraph, prevLayerName, ...
                    channels, currentStride, expansion, ...
                    sprintf('stage%d_block%d', stageIdx, blockIdx));
            else
                [lgraph, prevLayerName] = addMBConvBlock(lgraph, prevLayerName, ...
                    channels, currentStride, expansion, ...
                    sprintf('stage%d_block%d', stageIdx, blockIdx));
            end
        end
    end

    finalLayers = [
        groupNormalizationLayer(calculateNumGroups(1024), 'Name', 'final_gn')
        swishLayer('Name', 'final_swish')
    ];

    lgraph = addLayers(lgraph, finalLayers);
    lgraph = connectLayers(lgraph, prevLayerName, 'final_gn');

    net = dlnetwork(lgraph);
end

function [lgraph, lastLayerName] = addFusedMBConvBlock(lgraph, inputName, channels, stride, expansion, blockName)
    expanded = channels * expansion;

    layers = [
        convolution2dLayer(3, expanded, 'Stride', stride, 'Padding', 'same', ...
            'Name', [blockName '_fused_conv'], 'BiasLearnRateFactor', 0, 'BiasInitializer', 'zeros')
        groupNormalizationLayer(calculateNumGroups(expanded), 'Name', [blockName '_gn1'])
        swishLayer('Name', [blockName '_swish1'])
    ];

    if expanded ~= channels
        layers = [layers
            convolution2dLayer(1, channels, 'Stride', 1, 'Padding', 'same', ...
                'Name', [blockName '_project_conv'], 'BiasLearnRateFactor', 0, 'BiasInitializer', 'zeros')
            groupNormalizationLayer(calculateNumGroups(channels), 'Name', [blockName '_gn2'])
        ];
    end

    lgraph = addLayers(lgraph, layers);
    lgraph = connectLayers(lgraph, inputName, [blockName '_fused_conv']);
    lastLayerName = layers(end).Name;

    if stride == 1 && ~strcmp(inputName, 'stem_swish')
        inputChannels = getChannelsByLayerName(lgraph, inputName);

        if inputChannels == channels
            lgraph = addLayers(lgraph, additionLayer(2, 'Name', [blockName '_add']));
            lgraph = connectLayers(lgraph, inputName, [blockName '_add/in1']);
            lgraph = connectLayers(lgraph, lastLayerName, [blockName '_add/in2']);
            lastLayerName = [blockName '_add'];
        end
    end
end

function [lgraph, lastLayerName] = addMBConvBlock(lgraph, inputName, channels, stride, expansion, blockName)
    expanded = channels * expansion;

    expandConv = convolution2dLayer(1, expanded, 'Stride', 1, 'Padding', 'same', ...
        'Name', [blockName '_expand_conv'], 'BiasLearnRateFactor', 0, 'BiasInitializer', 'zeros');
    gn1 = groupNormalizationLayer(calculateNumGroups(expanded), 'Name', [blockName '_gn1']);
    sw1 = swishLayer('Name', [blockName '_swish1']);

    dwConv = groupedConvolution2dLayer(3, expanded, expanded, 'Stride', stride, 'Padding', 'same', ...
        'Name', [blockName '_depthwise_conv'], 'BiasLearnRateFactor', 0, 'BiasInitializer', 'zeros');
    gn2 = groupNormalizationLayer(calculateNumGroups(expanded), 'Name', [blockName '_gn2']);
    sw2 = swishLayer('Name', [blockName '_swish2']);

    projConv = convolution2dLayer(1, channels, 'Stride', 1, 'Padding', 'same', ...
        'Name', [blockName '_project_conv'], 'BiasLearnRateFactor', 0, 'BiasInitializer', 'zeros');
    gn3 = groupNormalizationLayer(calculateNumGroups(channels), 'Name', [blockName '_gn3']);

    layers = [expandConv; gn1; sw1; dwConv; gn2; sw2; projConv; gn3];

    lgraph = addLayers(lgraph, layers);
    lgraph = connectLayers(lgraph, inputName, expandConv.Name);
    lastLayerName = gn3.Name;

    if stride == 1 && ~strcmp(inputName, 'stem_swish')
        inputChannels = getChannelsByLayerName(lgraph, inputName);
        if inputChannels == channels
            lgraph = addLayers(lgraph, additionLayer(2, 'Name', [blockName '_add']));
            lgraph = connectLayers(lgraph, inputName, [blockName '_add/in1']);
            lgraph = connectLayers(lgraph, lastLayerName, [blockName '_add/in2']);
            lastLayerName = [blockName '_add'];
        end
    end
end

function numGroups = calculateNumGroups(numChannels)
    numGroups = min(floor(numChannels / 16), 32);
    numGroups = max(numGroups, 1);
    while mod(numChannels, numGroups) ~= 0 && numGroups > 1
        numGroups = numGroups - 1;
    end
end

function layer = findLayerByName(lgraph, layerName)
    for i = 1:length(lgraph.Layers)
        if strcmp(lgraph.Layers(i).Name, layerName)
            layer = lgraph.Layers(i);
            return;
        end
    end
    layer = [];
end

function channels = getChannelsByLayerName(lgraph, layerName)
    layer = findLayerByName(lgraph, layerName);

    if isa(layer, 'nnet.cnn.layer.ImageInputLayer')
        channels = layer.InputSize(3);
    elseif isa(layer, 'nnet.cnn.layer.Convolution2DLayer') || ...
           isa(layer, 'nnet.cnn.layer.GroupedConvolution2DLayer') || ...
           isa(layer, 'nnet.cnn.layer.DepthwiseConvolution2DLayer')
        channels = layer.NumFilters;
    else
        incomingLayers = findIncomingLayers(lgraph, layerName);
        if ~isempty(incomingLayers)
            channels = getChannelsByLayerName(lgraph, incomingLayers{1});
        else
            channels = 0;
        end
    end
end

function incomingLayers = findIncomingLayers(lgraph, layerName)
    incomingLayers = {};
    for i = 1:size(lgraph.Connections, 1)
        if strcmp(lgraph.Connections.Destination{i}, layerName)
            incomingLayers{end+1} = lgraph.Connections.Source{i};
        end
    end
end
