function net = createEfficientNetDownsamplingTail()
    % EfficientNet-style downsampling tail block with residual connection
    % Input: 14x14x1024 → Output: 7x7x2048

    lgraph = layerGraph();

    blockName = 'tail_block';
    inChannels = 1024;
    outChannels = 2048;
    expansion = 6;
    expanded = inChannels * expansion;

    % Calculate numGroups for GroupNorm
    numGroupsExpanded = min(floor(expanded / 32), 32);
    numGroupsExpanded = max(numGroupsExpanded, 1);
    while mod(expanded, numGroupsExpanded) ~= 0 && numGroupsExpanded > 1
        numGroupsExpanded = numGroupsExpanded - 1;
    end
    numGroupsOut = min(floor(outChannels / 32), 32);
    numGroupsOut = max(numGroupsOut, 1);
    while mod(outChannels, numGroupsOut) ~= 0 && numGroupsOut > 1
        numGroupsOut = numGroupsOut - 1;
    end

    % Input layer for 14x14x1024 input
    inputLayer = imageInputLayer([14 14 inChannels], 'Name', 'input', 'Normalization', 'none');
    lgraph = addLayers(lgraph, inputLayer);

    %% Main path layers
    mainLayers = [
        convolution2dLayer(1, expanded, 'Stride', 1, 'Padding', 'same', ...
            'Name', [blockName '_expand_conv'], ...
            'BiasLearnRateFactor', 0, 'BiasInitializer', 'zeros')
        groupNormalizationLayer(numGroupsExpanded, 'Name', [blockName '_gn1'])
        swishLayer('Name', [blockName '_swish1'])

        groupedConvolution2dLayer(3, expanded, expanded, 'Stride', 2, ...
            'Padding', 'same', 'Name', [blockName '_depthwise_conv'])
        groupNormalizationLayer(numGroupsExpanded, 'Name', [blockName '_gn2'])
        swishLayer('Name', [blockName '_swish2'])

        convolution2dLayer(1, outChannels, 'Stride', 1, 'Padding', 'same', ...
            'Name', [blockName '_project_conv'], ...
            'BiasLearnRateFactor', 0, 'BiasInitializer', 'zeros')
        groupNormalizationLayer(numGroupsOut, 'Name', [blockName '_gn3'])
    ];
    lgraph = addLayers(lgraph, mainLayers);

    %% Shortcut projection path
    shortcutLayers = [
        convolution2dLayer(1, outChannels, 'Stride', 2, 'Padding', 'same', ...
            'Name', [blockName '_shortcut_conv'], ...
            'BiasLearnRateFactor', 0, 'BiasInitializer', 'zeros')
        groupNormalizationLayer(numGroupsOut, 'Name', [blockName '_shortcut_gn'])
    ];
    lgraph = addLayers(lgraph, shortcutLayers);

    %% Addition + final Swish activation
    addition = additionLayer(2, 'Name', [blockName '_add']);
    finalSwish = swishLayer('Name', [blockName '_swish_out']);
    lgraph = addLayers(lgraph, [addition, finalSwish]);

    %% Connections
    % Connect input to both paths
    lgraph = connectLayers(lgraph, 'input', [blockName '_expand_conv']);
    lgraph = connectLayers(lgraph, 'input', [blockName '_shortcut_conv']);

    % Connect main path
    lgraph = connectLayers(lgraph, [blockName '_gn3'], [blockName '_add/in1']);

    % Connect shortcut path
    lgraph = connectLayers(lgraph, [blockName '_shortcut_gn'], [blockName '_add/in2']);

    % Swish after addition (auto-connected if added as a sequence)
    % So no need to connect addition → swish manually

    %% Final dlnetwork
exampleInput = dlarray(zeros(14, 14, 1024, 1), 'SSCB');
net = dlnetwork(lgraph, 'Initialize', true);
end





function [lgraph, lastLayerName] = addFusedMBConvBlock(lgraph, inputName, channels, stride, expansion, blockName)
    % Fused MBConv block implementation with GroupNorm
    expanded = channels * expansion;
    
    layers = [
        convolution2dLayer(3, expanded, 'Stride', stride, 'Padding', 'same', ...
            'Name', [blockName '_fused_conv'])
        groupNormalizationLayer(calculateNumGroups(expanded), 'Name', [blockName '_gn1'])
        swishLayer('Name', [blockName '_swish1'])
    ];
    
    if expanded ~= channels
        layers = [layers
            convolution2dLayer(1, channels, 'Stride', 1, 'Padding', 'same', ...
                'Name', [blockName '_project_conv'])
            groupNormalizationLayer(calculateNumGroups(channels), 'Name', [blockName '_gn2'])
        ];
    end
    
    lgraph = addLayers(lgraph, layers);
    lgraph = connectLayers(lgraph, inputName, [blockName '_fused_conv']);
    
    lastLayerName = layers(end).Name;
    
    % Add residual connection if possible
    if stride == 1 && strcmp(inputName, 'stem_swish') == 0
        lgraph = addLayers(lgraph, additionLayer(2, 'Name', [blockName '_add']));
        lgraph = connectLayers(lgraph, inputName, [blockName '_add/in1']);
        lgraph = connectLayers(lgraph, lastLayerName, [blockName '_add/in2']);
        lastLayerName = [blockName '_add'];
    end
end

function [lgraph, lastLayerName] = addMBConvBlock(lgraph, inputName, channels, stride, expansion, blockName)
    % MBConv block implementation with GroupNorm
    expanded = channels * expansion;
    
    layers = [
        convolution2dLayer(1, expanded, 'Stride', 1, 'Padding', 'same', ...
            'Name', [blockName '_expand_conv'])
        groupNormalizationLayer(calculateNumGroups(expanded), 'Name', [blockName '_gn1'])
        swishLayer('Name', [blockName '_swish1'])
        
        groupedConvolution2dLayer(3, expanded, expanded, 'Stride', stride, ...
            'Padding', 'same', 'Name', [blockName '_depthwise_conv'])
        groupNormalizationLayer(calculateNumGroups(expanded), 'Name', [blockName '_gn2'])
        swishLayer('Name', [blockName '_swish2'])
        
        convolution2dLayer(1, channels, 'Stride', 1, 'Padding', 'same', ...
            'Name', [blockName '_project_conv'])
        groupNormalizationLayer(calculateNumGroups(channels), 'Name', [blockName '_gn3'])
    ];
    
    lgraph = addLayers(lgraph, layers);
    lgraph = connectLayers(lgraph, inputName, [blockName '_expand_conv']);
    
    lastLayerName = layers(end).Name;
    
    % Add residual connection if possible
    if stride == 1 && strcmp(inputName, 'stem_swish') == 0
        lgraph = addLayers(lgraph, additionLayer(2, 'Name', [blockName '_add']));
        lgraph = connectLayers(lgraph, inputName, [blockName '_add/in1']);
        lgraph = connectLayers(lgraph, lastLayerName, [blockName '_add/in2']);
        lastLayerName = [blockName '_add'];
    end
end

function numGroups = calculateNumGroups(numChannels)
    % Calculate number of groups based on requirements:
    % - Minimum 32 channels per group
    % - Maximum 32 groups
    numGroups = min(floor(numChannels / 32), 32);
    
    % Ensure at least 1 group
    numGroups = max(numGroups, 1);
    
    % Ensure numChannels is divisible by numGroups
    while mod(numChannels, numGroups) ~= 0 && numGroups > 1
        numGroups = numGroups - 1;
    end
end