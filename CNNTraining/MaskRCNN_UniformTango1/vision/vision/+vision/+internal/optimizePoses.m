%optimizePoses Optimize absolute poses using relative pose constraints
%   optimizePoses optimizes a pose graph whose nodes correspond to absolute
%   poses and edges correspond to relative pose constraints. You can use
%   the createPoseGraph method of imageviewset or pcviewset to create a
%   pose graph.
%
%   poseTable = optimizePoses(G) optimizes the pose graph G, and returns
%   the optimized poses in poseTable. G must be a digraph object with Nodes
%   containing columns ViewId and AbsolutePose, and Edges containing
%   columns EndNodes, RelativePose, and optionally InformationMatrix and
%   Weight. poseTable is a table containing columns ViewId, AbsolutePose,
%   and optionally Scale when G is a similarity pose graph. 
%
%   [poseTable, solInfo] = optimizePoses(G) also returns additional
%   statistics about the optimization process in a struct, solInfo, as the
%   second output.
%
%   poseTable = optimizePoses(G, Name, Value) specifies additional
%   name-value pair arguments as described below:
%
%   'MaxIterations'     A positive integer specifying the maximum number of
%                       iterations before optimization is terminated.
%                       Increase this value at the expense of speed for
%                       more accurate results.
%
%                       Default: 300
%
%   'Tolerance'         A positive scalar specifying the tolerance of the
%                       optimization cost function. If the cost function
%                       changes by less than this value between two
%                       iterations, optimization is terminated.
%
%                       Default: 1e-8
%
%
%   'Verbose'           Set true to display progress information.
%
%                       Default: false
%
%   'MaxTime'           A positive scalar specifying the maximum time
%                       allowed (in seconds) before optimization is
%                       terminated.
%
%                       Default: inf
%
%
%   Notes
%   -----
%   - The optimizePoses function uses the Levenberg Marquardt optimization
%     algorithm with sparse Cholesky factorization from the g2o graph
%     optimization library.
%   - To update a view set with optimized poses, use the updateView method
%     of pcviewset or imageviewset.
%   - The optimizePoses function holds the first node in the digraph fixed.
%
%
%   Example: Create and optimize a pose graph
%   -----------------------------------------
%   % Define 4 nodes
%   ViewId = [1:4]';
%
%   % Specify absolute poses for each node
%   AbsolutePose = repelem(rigidtform3d, 4, 1);
%
%   AbsolutePose(1).Translation = [ 0   0 0];
%   AbsolutePose(2).Translation = [ 1   0 0];
%   AbsolutePose(3).Translation = [ 2   0 0];
%   AbsolutePose(4).Translation = [ 0.1 0 0];
%
%   % Define 4 edges - 3 odometry, and 1 loop closure
%   EndNodes = [1 2; 2 3; 3 4; 4 1];
%
%   % Specify relative poses for each edge
%   RelativePose = repelem(rigidtform3d, 4, 1);
%
%   RelativePose(1).Translation = [ 1   0 0];
%   RelativePose(2).Translation = [ 1   0 0];
%   RelativePose(3).Translation = [-1.9 0 0];
%   RelativePose(4).Translation = [ 0.2 0 0];
%
%   % Create a pose graph as a digraph
%   edgeTable = table(EndNodes, RelativePose);
%   nodeTable = table(ViewId, AbsolutePose);
%
%   G = digraph(edgeTable, nodeTable)
%
%   % Optimize pose graph
%   poseTable = vision.internal.optimizePoses(G);
%
%   % Print original and optimized locations
%   disp('Original Absolute Translations:')
%   disp(vertcat(AbsolutePose.Translation))
%
%   disp('Optimized Absolute Translations:')
%   disp(vertcat(poseTable.AbsolutePose.Translation))
%
%   See also digraph, rigidtform3d, imageviewset, pcviewset.

%   References
%   ----------
%   [1] Rainer Kuemmerle, Giorgio Grisetti, Hauke Strasdat, Kurt Konolige,
%       and Wolfram Burgard. g2o: A General Framework for Graph
%       Optimization IEEE International Conference on Robotics and
%       Automation (ICRA), 2011.

% Copyright 2019-2022 The MathWorks, Inc.

%#codegen

function [poseTable, solInfo] = optimizePoses(G, varargin)
if isempty(coder.target)

    [Nodes, Edges, isSimilarityPoseGraph, infoMats, maxIterations, funcTolerance, verboseFlag, maxTime] = parseInputs(G, varargin{:});

    % If there are no constraints, there is no need to optimize. Just return
    % the nodes.
    if isempty(Edges)
        poseTable = Nodes(:, {'ViewId', 'AbsolutePose'});
        return;
    end

    if isSimilarityPoseGraph
        [optimPoses, optimStruct, optimPoseScales] = visionOptimizePoses(Nodes.AbsolutePose, Edges.RelativePose, ...
            infoMats, Edges.EndNodes, maxIterations, funcTolerance, verboseFlag, maxTime, Edges.Scale);
        poseTable = table(Nodes.ViewId, optimPoses, optimPoseScales, 'VariableNames', {'ViewId', 'AbsolutePose', 'Scale'});
    else
        [optimPoses, optimStruct] = visionOptimizePoses(Nodes.AbsolutePose, Edges.RelativePose, ...
            infoMats, Edges.EndNodes, maxIterations, funcTolerance, verboseFlag, maxTime);
        poseTable = table(Nodes.ViewId, optimPoses, 'VariableNames', {'ViewId', 'AbsolutePose'});
    end
    poseTable.AbsolutePose = checkAndConvertToSingle(Nodes, poseTable.AbsolutePose);

    if verboseFlag
        switch optimStruct.ExitFlag
            case 1
                messageToPrint = getString(message('vision:optimizePoses:minimumFound'));
            case 2
                messageToPrint = getString(message('vision:optimizePoses:maxIterationsReached'));
            case 5
                messageToPrint = getString(message('vision:optimizePoses:belowTolerance'));
        end

        fprintf('%s\n', messageToPrint);
    end

    if nargout > 1
        solInfo = struct('Iterations', optimStruct.ExitFlag, 'Error', optimStruct.FinalChi, 'ExitFlag', optimStruct.ExitFlag);
    end
else
    [poseTable, solInfo] = vision.internal.codegen.poseGraph.optimizePoses(G, varargin{:});
end
end
    
    %--------------------------------------------------------------------------
    function [Nodes, Edges, isSimilarityPoseGraph, infoMats, maxIterations, ...
        funcTolerance, verboseFlag, maxTime] = parseInputs(G, varargin)
    
    p = inputParser;
    p.FunctionName = 'optimizePoses';
    addRequired(p, 'G');
    addParameter(p, 'Type',          'rigid');
    addParameter(p, 'MaxIterations', 300);
    addParameter(p, 'Tolerance',     1e-8);
    addParameter(p, 'Verbose',       false);
    addParameter(p, 'MaxTime',       inf);
    
    parse(p, G, varargin{:});
    [Nodes, Edges, isSimilarityPoseGraph, hasInfoMat] = checkPoseGraph(G);
    
    if hasInfoMat
        infoMats = Edges.InformationMatrix;
    else
        infoMats = repmat({eye(6+isSimilarityPoseGraph)}, size(Edges,1), 1);
    end
    
    maxIterations = checkIterations(p.Results.MaxIterations);
    funcTolerance = checkTolerance(p.Results.Tolerance);
    verboseFlag   = checkVerbose(p.Results.Verbose);
    maxTime       = checkMaxTime(p.Results.MaxTime);
    end
    
    %--------------------------------------------------------------------------
    function [Nodes, Edges, isSimilarityPoseGraph, hasInfoMat] = checkPoseGraph(G)
    
    validateattributes(G, {'digraph'}, {'scalar'}, 'optimizePoses', 'pose graph G');
    
    nodeVarNames = G.Nodes.Properties.VariableNames;
    
    requiredNodeVars = {'ViewId', 'AbsolutePose'};
    requiredEdgeVars = {'EndNodes', 'RelativePose'};
    optionalEdgeVars = {'InformationMatrix', 'Weight'};
    
    % Check graph nodes
    if ~isequal(nodeVarNames, requiredNodeVars)
        error(message('vision:viewSet:requiredColumnsMissing', 'Nodes of G', ...
            strjoin(requiredNodeVars, ', ')))
    end
    
    validateattributes(G.Nodes.ViewId, {'numeric'}, ...
        {'nonsparse', 'nonempty', 'vector', 'integer', 'positive', 'real'}, ...
        'optimizePoses', 'ViewId of Nodes table');
    validateattributes(G.Nodes.AbsolutePose, {'rigidtform3d','rigid3d'}, ...
        {'vector', 'nonempty'}, 'optimizePoses', 'AbsolutePose of Nodes table');

    if isa(G.Nodes.AbsolutePose, 'rigid3d')
        AbsolutePose = repelem(rigidtform3d, height(G.Nodes.AbsolutePose), 1);
        for i = 1:height(G.Nodes.AbsolutePose)
            AbsolutePose(i,1) = rigidtform3d(G.Nodes.AbsolutePose(i).T');
        end
        ViewId = G.Nodes.ViewId;
        Nodes = table(ViewId, AbsolutePose);
    else
        Nodes = G.Nodes;
    end
    
    Edges        = G.Edges;
    edgeVarNames = Edges.Properties.VariableNames;
    
    % Check graph edges
    if ~isequal(edgeVarNames(1:numel(requiredEdgeVars)), requiredEdgeVars)
        error(message('vision:viewSet:requiredColumnsMissing', 'Edges of G', ...
            strjoin(requiredEdgeVars, ', ')));
    end
    
    hasOptionalEdgeVars = numel(edgeVarNames)>2;
    hasInfoMat = hasOptionalEdgeVars && ismember(optionalEdgeVars{1}, edgeVarNames(3:end));
    hasWeight  = hasOptionalEdgeVars && ismember(optionalEdgeVars{2}, edgeVarNames(3:end));
    if hasOptionalEdgeVars && ~(hasInfoMat || hasWeight)
        error(message('vision:viewSet:optionalColumnsInvalid', 'Edges of G', ...
            strjoin(requiredEdgeVars, ', '), strjoin(optionalEdgeVars, ', ')));
    end
    
    validateattributes(Edges.EndNodes, {'numeric'}, ...
        {'ncols', 2, 'nonsparse', 'integer', 'positive', 'real'}, ...
        'optimizePoses', 'EndNodes of Edges table');
    
    % Check if the pose graph is a similarity pose graph:
    %   1) Pose graph created from pcviewset
    %   2) Pose graph created from imageviewset with simtform3d in connections
    %   3) Pose graph created from imageviewset without simtform3d in connections
    % Only in the 2nd case is the pose graph a similarity pose graph
    
    [Edges, isSimilarityPoseGraph, infoMatDoF] = checkAndConvertRelativePose(Edges);
    
    % Check information matrix
    if hasInfoMat
        Edges.InformationMatrix = checkAndConvertInformationMatrix( ...
            Edges.InformationMatrix, infoMatDoF, isSimilarityPoseGraph);
    end
    end
    
    %--------------------------------------------------------------------------
    function [Edges, isSimilarityPoseGraph, infoMatDoF] = checkAndConvertRelativePose(Edges)
    isSimilarityPoseGraph = false;
    infoMatDoF            = 6 * ones(size(Edges, 1), 1);
    
    if iscell(Edges.RelativePose) % imageviewset
        [Edges.RelativePose, isSim3Pose, edgeScales] = ...
            cellfun(@(p)checkAndDecomposeSimilarityTform(p), Edges.RelativePose);
        
        isSimilarityPoseGraph         = any(isSim3Pose);
        if isSimilarityPoseGraph
            Edges.Scale               = edgeScales;
            infoMatDoF(isSim3Pose, :) = 7;
        end
    else % pcviewset
        validateattributes(Edges.RelativePose, {'rigidtform3d','rigid3d'}, {'vector'}, ...
            'optimizePoses', 'RelativePose of Edges table');
        if isa(Edges.RelativePose, 'rigid3d')
            RelativePose = repelem(rigidtform3d, height(Edges.RelativePose), 1);
            for i = 1:height(Edges.RelativePose)
                RelativePose(i,1) = rigidtform3d(Edges.RelativePose(i).T');
            end
            EndNodes = Edges.EndNodes;
            Edges = table(EndNodes, RelativePose);
        end
    end
    end
    
    %--------------------------------------------------------------------------
    function [relPose, isSim3Pose, scale] = checkAndDecomposeSimilarityTform(relPose)
    validateattributes(relPose, {'rigidtform3d', 'simtform3d', 'rigid3d', 'affine3d'},...
        {'scalar'}, 'optimizePoses', 'RelativePose of Edges table');
    
    if isa(relPose, 'simtform3d') || isa(relPose, 'affine3d')
        isSim3Pose = true;
        T          = relPose.T;
        scale      = double(nthroot(det(T(1:3, 1:3)), 3)); % visionOptimizePoses only supports double
        T(:, 1:3)  = T(:, 1:3)/scale;
        relPose    = rigidtform3d(T');
    else
        isSim3Pose = false;
        scale      = 1; % double
        T          = relPose.T;
        relPose    = rigidtform3d(T');
    end
    end
    
    %--------------------------------------------------------------------------
    function infoMats = checkAndConvertInformationMatrix(infoMats, infoMatDoF, isSimilarityPoseGraph)
    validateattributes(infoMats, {'cell'}, {'vector'}, 'optimizePoses', ...
        'InformationMatrix of Edges table');
    
    for i = 1:numel(infoMats)
        validateattributes(infoMats{i}, {'single','double'}, ...
            {'real','size', [infoMatDoF(i), infoMatDoF(i)]}, 'optimizePoses', ...
            'element of InformationMatrix column');
        
        % Add values for the scale DoF in the case of similarity pose graph
        if isSimilarityPoseGraph && infoMatDoF(i) == 6
            infoMats{i} = [infoMats{i}, zeros(6, 1); zeros(1, 6), 1];
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

    %--------------------------------------------------------------------------
    function optimPoses = checkAndConvertToSingle(nodes, optimPoses)
    % Convert 'T' property of optimized poses to single type if the 
    % 'T' property of input poses are single
    for i=1:numel(optimPoses)
        if isa(nodes.AbsolutePose(i).T, 'single')
            optimPoses(i) = rigidtform3d(single(optimPoses(i).T'));
        end
    end
    end