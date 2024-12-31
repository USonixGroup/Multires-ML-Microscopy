function [poseTable, solInfo] = optimizePoses(G, varargin)
%
%   This function is used to implement code generation support for the
%   vision.internal.optimizePoses function.

% Copyright 2021-2022 The MathWorks, Inc.
%#codegen
    if isempty(G.Edges)
        validateattributes(G, {'digraph'}, {'scalar'}, 'optimizePoses', 'pose graph G');
        requiredNodeVars = {'ViewId', 'AbsolutePose'};
        varNames = G.Nodes.Properties.VariableNames;
        coder.internal.errorIf(coder.ignoreConst(~isequal(varNames, requiredNodeVars)),...
                               'vision:viewSet:requiredColumnsMissing', 'Nodes of G', ...
                               strjoin(requiredNodeVars, ', '));
        hasAbsolutePose = false;
        hasViewId = false;
        for i=1:numel(varNames)
            if isequal(requiredNodeVars{1}, varNames{i})
                hasAbsolutePose = true;
            end
            if isequal(requiredNodeVars{2}, varNames{i})
                hasViewId = true;
            end
        end
        if ~isempty(G.Nodes) && hasAbsolutePose && hasViewId
            % Return transformation matrix of absolute poses when edges are
            % empty
            numNodes = size(G.Nodes, 1);
            tformCell = cell(numNodes, 1);
            for i=1:numNodes
                tformCell{i} = G.Nodes.AbsolutePose{i}.T;
            end
            poseTable = table(G.Nodes.ViewId, tformCell, ones(numNodes, 1), ...
                'VariableNames', {'ViewId', 'AbsolutePose', 'Scale'});
            solInfo = struct('Iterations', [], 'Error', [], 'ExitFlag', []);
            return;
        end
    end

    params = parseInputsCodegen(G, varargin{:});
    if coder.internal.isTargetMATLABHost()
        [optimPoses, optimStruct, optimPoseScales] = vision.internal.buildable.optimizePosesBuildable.visionOptimizePoses(params);
    else
        [optimPoses, optimStruct, optimPoseScales] = vision.internal.buildable.optimizePosesBuildablePortable.visionOptimizePoses(params);
    end

    poseTable = table(params.Nodes.ViewId, optimPoses, optimPoseScales, 'VariableNames', {'ViewId', 'AbsolutePose', 'Scale'});

    if nargout > 1
        solInfo = struct('Iterations', optimStruct.ExitFlag, 'Error', optimStruct.FinalChi, 'ExitFlag', optimStruct.ExitFlag);
    end
end

%--------------------------------------------------------------------------
function params = parseInputsCodegen(G, varargin)

    narginchk(1, 9);
    % Specify defaults of paramteres
    defaults = struct(...
        'MaxIterations', 300, ...
        'Tolerance',     1e-8, ...
        'Verbose',       false, ...
        'MaxTime',       inf);

    % Define parser mapping struct
    pvPairs = struct(...
        'MaxIterations', uint32(0), ...
        'Tolerance',     uint32(0), ...
        'Verbose',       false, ...
        'MaxTime',       uint32(0));

    % Specify parser options
    poptions = struct( ...
        'CaseSensitivity', false, ...
        'StructExpand',    true, ...
        'PartialMatching', true);

    % Parse PV pairs
    pstruct = coder.internal.parseParameterInputs(pvPairs, poptions, varargin{:});

    % Extract inputs
    maxIterations = coder.internal.getParameterValue(pstruct.MaxIterations, defaults.MaxIterations, varargin{:});
    tolerance = coder.internal.getParameterValue(pstruct.Tolerance, defaults.Tolerance, varargin{:});
    verbose = coder.internal.getParameterValue(pstruct.Verbose, defaults.Verbose, varargin{:});
    maxTime = coder.internal.getParameterValue(pstruct.MaxTime, defaults.MaxTime, varargin{:});

    [nodes, edges, isSimilarityPoseGraph, hasInfoMat, infoMatDoF] = checkPoseGraphCodegen(G);

    if hasInfoMat
        info = checkAndConvertInformationMatrixCodegen( ...
            edges.InformationMatrix, infoMatDoF, isSimilarityPoseGraph);
    else
        info = repmat({eye(6+isSimilarityPoseGraph)}, size(edges.EndNodes,1), 1);
    end

    params.MaxIterations = checkIterations(maxIterations);
    params.Tolerance = checkTolerance(tolerance);
    params.Verbose   = checkVerbose(verbose);
    params.MaxTime   = checkMaxTime(maxTime);
    params.Nodes = nodes;
    params.Edges = edges;
    params.isSimilarityPoseGraph = isSimilarityPoseGraph;
    params.InfoMats = info;
end

%------------------------------------------------------------------------
function [Nodes, Edges, isSimilarityPoseGraph, hasInfoMat, infoMatDoF] = checkPoseGraphCodegen(G)

    validateattributes(G, {'digraph'}, {'scalar'}, 'optimizePoses', 'pose graph G');

    nodes        = G.Nodes;
    nodeVarNames = nodes.Properties.VariableNames;

    requiredNodeVars = {'ViewId', 'AbsolutePose'};
    requiredEdgeVars = {'EndNodes', 'RelativePose'};
    optionalEdgeVars = {'InformationMatrix', 'Weight'};

    % Check graph nodes
    coder.internal.errorIf(coder.ignoreConst(~isequal(nodeVarNames, requiredNodeVars)),...
                           'vision:viewSet:requiredColumnsMissing', 'Nodes of G', ...
                           strjoin(requiredNodeVars, ', '));
    hasViewId = false;
    hasAbsolutePose = false;
    for i=1:numel(nodeVarNames)
        if isequal(nodeVarNames{i}, requiredNodeVars{1})
            hasViewId = true;
        end
        if isequal(nodeVarNames{i}, requiredNodeVars{2})
            hasAbsolutePose = true;
        end
    end

    if hasViewId
        validateattributes(nodes.ViewId, {'numeric'}, ...
                           {'nonsparse', 'nonempty', 'vector', 'integer', 'positive', 'real'}, ...
                           'optimizePoses', 'ViewId of Nodes table');
        Nodes.ViewId = nodes.ViewId;
    else
        Nodes.ViewId = [];
    end
    if hasAbsolutePose
        Nodes.AbsolutePose = checkAndConvertAbsolutePoseCodegen(nodes);
    else
        Nodes.AbsolutePose = [];
    end

    edges        = G.Edges;
    edgeVarNames = edges.Properties.VariableNames;

    % Check graph edges
    coder.internal.errorIf(coder.ignoreConst(numel(edgeVarNames) < 2),...
                           'vision:viewSet:requiredColumnsMissing', 'Edges of G', ...
                           strjoin(requiredEdgeVars, ', '));
    hasEndNodes = false;
    hasReativePose = false;
    for i=1:numel(edgeVarNames)
        if isequal(edgeVarNames{i}, requiredEdgeVars{1})
            hasEndNodes = true;
        end
        if isequal(edgeVarNames{i}, requiredEdgeVars{2})
            hasReativePose = true;
        end
    end
    coder.internal.errorIf(coder.ignoreConst(~hasEndNodes || ~hasReativePose), ...
                           'vision:viewSet:requiredColumnsMissing', 'Edges of G', ...
                           strjoin(requiredEdgeVars, ', '));

    hasInfoMat = false;
    hasWeight = false;
    hasOptionalEdgeVars = numel(edgeVarNames)>2;
    for i=3:numel(edgeVarNames)
        if isequal(edgeVarNames{i}, optionalEdgeVars{1})
            hasInfoMat = true;
        end
        if isequal(edgeVarNames{i}, optionalEdgeVars{2})
            hasWeight = true;
        end
    end

    hasInfoMat = hasOptionalEdgeVars && hasInfoMat;
    hasWeight  = hasOptionalEdgeVars && hasWeight;
    coder.internal.errorIf(coder.ignoreConst(hasOptionalEdgeVars && ~(hasInfoMat || hasWeight)),...
                           'vision:viewSet:optionalColumnsInvalid', 'Edges of G', ...
                           strjoin(requiredEdgeVars, ', '), strjoin(optionalEdgeVars, ', '));

    [Edges, isSimilarityPoseGraph, infoMatDoF] = checkAndConvertRelativePoseCodegen(edges, ...
                                                                                    hasInfoMat, hasWeight, hasReativePose);
    if hasEndNodes
        validateattributes(edges.EndNodes, {'numeric'}, ...
                           {'ncols', 2, 'nonsparse', 'integer', 'positive', 'real'}, ...
                           'optimizePoses', 'EndNodes of Edges table');
        Edges.EndNodes = edges.EndNodes;
    end
end

%--------------------------------------------------------------------------
function absolutePose = checkAndConvertAbsolutePoseCodegen(nodes)
    absolutePose = nodes(1,:).AbsolutePose{1};
    if iscell(nodes.AbsolutePose)
        for i=1:size(nodes,1)
            absPose = nodes(i,:).AbsolutePose{1};
            validateattributes(absPose, {'rigidtform3d','rigid3d'}, {'scalar'}, ...
                               'optimizePoses', 'RelativePose of Edges table');
            absolutePose(i) = absPose;
        end
    else
        validateattributes(nodes.AbsolutePose, {'rigidtform3d','rigid3d'}, ...
                           {'vector', 'nonempty'}, 'optimizePoses', 'AbsolutePose of Nodes table');
        absolutePose = nodes.AbsolutePose;
    end
end

%--------------------------------------------------------------------------
function [edges, isSimilarityPoseGraph, infoMatDoF] = checkAndConvertRelativePoseCodegen(Edges, hasInfoMat, hasWeight, hasRelativePose)
    isSimilarityPoseGraph = false;
    infoMatDoF            = 6 * ones(size(Edges, 1), 1);

    if hasRelativePose
        if iscell(Edges.RelativePose) % imageviewset
            [relativePose, isSim3Pose, edgeScales] = ...
                checkAndDecomposeSimilarityTformCodegen(Edges);
            edges.RelativePose = relativePose;
            isSimilarityPoseGraph         = any(isSim3Pose);
            if isSimilarityPoseGraph
                edges.Scale               = edgeScales;
                infoMatDoF(isSim3Pose, :) = 7;
            else
                edges.Scale = [];
            end
            if(hasInfoMat)
                edges.InformationMatrix = Edges.InformationMatrix;
            end
            if hasWeight
                edges.Weight = Edges.Weight;
            end
        else % pcviewset
            validateattributes(Edges.RelativePose, {'rigidtform3d','rigid3d'}, {'vector'}, ...
                               'optimizePoses', 'RelativePose of Edges table');
            edges = Edges;
        end
    else
        edges.RelativePose = [];
        edges.Scale = [];
    end
end

%--------------------------------------------------------------------------
function [relPose, isSim3Pose, scale] = checkAndDecomposeSimilarityTformCodegen(Edges)
    poseClass = class(Edges(1,:).RelativePose{1}.T);
    relPose = rigidtform3d(eye(4, 4, poseClass));
    nEdges = size(Edges,1);
    isSim3Pose = coder.nullcopy(false(nEdges,1));
    scale = coder.nullcopy(zeros(nEdges,1));
    for i=1:nEdges
        rPose = Edges(i,:).RelativePose{1};
        validateattributes(rPose, {'rigidtform3d','rigid3d', 'affine3d'}, {'scalar'}, ...
                           'optimizePoses', 'RelativePose of Edges table');
        if isa(rPose, 'affine3d')
            isSim3Pose(i) = true;
            T          = rPose.T;
            scale(i)      = double(nthroot(det(T(1:3, 1:3)), 3)); % visionOptimizePoses only supports double
            T(:, 1:3)  = T(:, 1:3)/scale(i);
            relPose(i)    = rigidtform3d(T');
        else
            isSim3Pose(i) = false;
            scale(i)      = 1;
            relPose(i) = rPose;
        end
    end
end

%--------------------------------------------------------------------------
function infoMats = checkAndConvertInformationMatrixCodegen(infoMatsInput, infoMatDoF, isSimilarityPoseGraph)
    validateattributes(infoMatsInput, {'cell'}, {'vector'}, 'optimizePoses', ...
                       'InformationMatrix of Edges table');
    numRows = numel(infoMatsInput);
    infoMats = cell(numRows, 1);
    for i = 1:numRows
        validateattributes(infoMatsInput{i}, {'single','double'}, ...
                           {'real','size', [infoMatDoF(i), infoMatDoF(i)]}, 'optimizePoses', ...
                           'element of InformationMatrix column');

        % Add values for the scale DoF in the case of similarity pose graph
        if isSimilarityPoseGraph && infoMatDoF(i) == 6
            infoMats{i,1} = [infoMatsInput{i}, zeros(6, 1); zeros(1, 6), 1];
        else
            infoMats{i,1} = [infoMatsInput{i}]  ;
        end
    end
end

%--------------------------------------------------------------------------
function iter = checkIterations(iter)

% g2o uses a 32-bit signed integer for iterations
    maxIter = double(intmax('int32'));

    validateattributes(iter, {'numeric'}, ...
                       {'scalar', 'real', 'nonsparse', 'positive', 'integer', '<=', maxIter}, ...
                       'optimizePoses', 'MaxIterations');

    iter = double(iter);
end

%--------------------------------------------------------------------------
function tol = checkTolerance(tol)

    validateattributes(tol, {'single', 'double'}, ...
                       {'scalar', 'real','positive', 'nonsparse', 'finite'}, 'optimizePoses', ...
                       'Tolerance');
    tol = double(tol);
end

%--------------------------------------------------------------------------
function verbose = checkVerbose(verbose)

    validateattributes(verbose, {'numeric', 'logical'}, {'scalar', 'binary'}, ...
                       'optimizePoses', 'Verbose');
    verbose = logical(verbose);
end

%--------------------------------------------------------------------------
function maxTime = checkMaxTime(maxTime)
    validateattributes(maxTime, {'single','double'}, ...
                       {'scalar','real','positive','nonsparse'}, 'optimizePoses', 'MaxTime');
    maxTime = double(maxTime);
end
