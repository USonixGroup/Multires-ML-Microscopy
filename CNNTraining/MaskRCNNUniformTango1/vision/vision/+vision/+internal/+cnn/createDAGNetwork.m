function NNTNetwork = createDAGNetwork(lgraph)
% Create a DAGNetwork without training. Used to take internal Fast RCNN
% network and convert it to DAG. This is a temporary file until official
% MIMO DAG training is in place.

% Copyright 2018 The Mathworks, Inc.

assert(isa(lgraph,'nnet.cnn.LayerGraph'));

analysis = nnet.internal.cnn.analyzer.NetworkAnalyzer(lgraph);

lgraph = analysis.LayerGraph;

layersMap = nnet.internal.cnn.layer.util.InternalExternalMap(lgraph.Layers);

% Now make an internal network to pass to DAGNetwork.
internalLayerGraph = iExternalToInternalLayerGraph(lgraph);

topologicalOrder = extractTopologicalOrder( lgraph );

internalNetwork = nnet.internal.cnn.DAGNetwork(internalLayerGraph, topologicalOrder);

% Create the DAG network.
NNTNetwork = DAGNetwork(internalNetwork, layersMap);

% The following functions were copied from NNT's trainDAGNetwork.m.
    function internalLayerGraph = iExternalToInternalLayerGraph( externalLayerGraph )
        internalLayers = iGetInternalLayers( externalLayerGraph.Layers );
        hiddenConnections = externalLayerGraph.HiddenConnections;
        internalConnections = iHiddenToInternalConnections( hiddenConnections );
        internalLayerGraph = nnet.internal.cnn.LayerGraph(internalLayers, internalConnections);
    end

    function internalLayers = iGetInternalLayers( layers )
        internalLayers = nnet.internal.cnn.layer.util.ExternalInternalConverter.getInternalLayers( layers );
    end

    function internalConnections = iHiddenToInternalConnections( hiddenConnections )
        internalConnections = nnet.internal.cnn.util.hiddenToInternalConnections( hiddenConnections );
    end
end
