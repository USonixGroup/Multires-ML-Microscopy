classdef optimizePosesBuildable < coder.ExternalDependency %#codegen
% optimizePosesBuildable - encapsulate optimizePosesBuildable implementation library

% Copyright 2021-2022 The MathWorks, Inc.
    methods (Static)

        function name = getDescriptiveName(~)
            name = 'optimizePosesBuildable';
        end

        function b = isSupportedContext(context)
            b = context.isMatlabHostTarget();
        end

        function updateBuildInfo(buildInfo, context)
            vision.internal.buildable.cvstBuildInfo(buildInfo, context, ...
                                                    'optimizePoses',{});
            vision.internal.buildable.portableOptimizePosesBuildInfo(buildInfo, context);
        end

        %------------------------------------------------------------------
        %                    Function to optimizePoses
        %------------------------------------------------------------------
        function [optimPoses, optimStruct, optimPoseScales] = visionOptimizePoses(params)

            coder.inline('always');
            coder.cinclude('cvstCG_optimizePoses.h');

            absolutePose = params.Nodes.AbsolutePose;
            relativePose = params.Edges.RelativePose;
            info = params.InfoMats;
            edgeIds  = params.Edges.EndNodes;
            maxIter = params.MaxIterations;
            funcTol = params.Tolerance;
            verboseFlag = params.Verbose;
            maxTime = params.MaxTime;
            isSimilar = params.isSimilarityPoseGraph;

            % Obtain number of nodes and edges
            numNodes = numel(absolutePose);
            numEdges = numel(relativePose);

            % Setting the dimension of pose and type of block solver based
            % on similarity of graph
            if(isSimilar)
                poseDim = 8;
                blockSolverType = 2;
                edgeScales = params.Edges.Scale;
            else
                poseDim = 7;
                blockSolverType = 1;
                edgeScales = [];
            end

            edgeDim = poseDim + 2;

            % Creating 2D matrices to store poses, edges, optimized poses,
            % compact information matrices
            nodes = coder.nullcopy(zeros(numNodes, poseDim));
            edges = coder.nullcopy(zeros(numEdges, edgeDim));
            optimNodes = coder.nullcopy(zeros(numNodes, poseDim));
            if isSimilar
                infoDim = 28;
                infoMatsCompact = coder.nullcopy(zeros(numEdges, infoDim));
            else
                infoDim = 21;
                infoMatsCompact = coder.nullcopy(zeros(numEdges, infoDim));
            end

            % Array stores additional solver info
            solverInfo = zeros(3, 1, 'double');

            nodesVec = coder.opaquePtr('void', coder.internal.null);
            edgesVec = coder.opaquePtr('void', coder.internal.null);
            infoMatsVec = coder.opaquePtr('void', coder.internal.null);

            for n=1:numNodes
                % Array to store pose from of each node
                pose = zeros(1, poseDim, "double");
                % Obtain tform of each node
                if isa(absolutePose(n).T, 'single')
                    tform = double(absolutePose(n).T);
                else
                    tform = absolutePose(n).T;
                end
                fcnName = 'tform2quatpose';
                % Calling the function that converts tform to pose form
                coder.ceval(fcnName, coder.ref(tform), coder.ref(pose));
                if isSimilar
                    pose(end)= 1.0;
                end
                % Storing the pose form of each node
                nodes(n,:) = pose;
            end

            for i=1:numEdges
                % Array to store pose from of each edge
                edge = zeros(1, edgeDim, "double");
                % Store the endnodes of each edge
                edge(1) = edgeIds(i) - 1;
                edge(2) = edgeIds(i + numEdges) -1;
                % Obtain tform of each edge
                if isa(relativePose(i).T, 'single')
                    relPose = double(relativePose(i).T);
                else
                    relPose = relativePose(i).T;
                end
                fcnName = 'tform2quatpose';
                % Calling the function that converts tform to pose form
                coder.ceval(fcnName, coder.ref(relPose), coder.ref(edge(3)));
                if (isSimilar)
                    % Convert [x/scale, y/scale, z/scale] to [x, y, z] to follow
                    % the format in shared backend
                    edge(3) = edge(3)*edgeScales(i);
                    edge(4) = edge(4)*edgeScales(i);
                    edge(5) = edge(5)*edgeScales(i);
                    edge(edgeDim) = edgeScales(i);
                end
                edges(i,:) = edge;
                % Compactify similarity matrix
                if(isSimilar)
                    infoMatCompact = vision.internal.buildable.optimizePosesBuildable.compactifySimilarityInformationMatrix(info{i});
                else
                    infoMatCompact = vision.internal.buildable.optimizePosesBuildable.compactifyRigidInformationMatrix(info{i});
                end
                % Store the compact information matrix
                infoMatsCompact(i,:) = infoMatCompact;
            end

            % Calling the function to convert 2-D matrices to 2-D vectors
            % so that the data format matches the backend requirement
            fcnName = 'initializeVectors';

			% Integer values of variables to be passed to initializeVectors

            nNodes = cast(numNodes, 'uint32');
            nEdges = cast(numEdges, 'uint32');
            poseD = cast(poseDim, 'uint32');
            edgeD = cast(edgeDim, 'uint32');
            infoD = cast(infoDim, 'uint32');
            coder.ceval(fcnName, nNodes, poseD, nodes, coder.ref(nodesVec));
            coder.ceval(fcnName, nEdges, edgeD, edges, coder.ref(edgesVec));
            coder.ceval(fcnName, nEdges, infoD, infoMatsCompact, coder.ref(infoMatsVec));

            % Calling the function which optimizes poses
            fcnName = 'poseOptimizer';
            coder.ceval(fcnName, blockSolverType, maxIter, funcTol,...
                        verboseFlag, maxTime, nodesVec, edgesVec,...
                        infoMatsVec, coder.ref(optimNodes), coder.ref(solverInfo));

            % Cell array which stores the tforms of all optimized nodes
            optimPoses = coder.nullcopy(cell(numNodes, 1));
            k = 1;
            j = poseDim;
            for i = 1:numNodes
                tform = zeros(4, 4, "double");
                % Get the optimized pose form of each node
                pose1 = optimNodes(k:j);
                fcnName = 'quatpose2tform';
                % Calling the function which converts pose form to t-form
                coder.ceval(fcnName, coder.ref(pose1), coder.ref(tform));
                if isa(absolutePose(i).T, 'single')
                    optimPoses{i} = single(tform);
                else
                    optimPoses{i} = tform;
                end
                k = k+poseDim;
                j = j+poseDim;
            end

            % Copying additional solver info to struct
            optimStruct.FinalChi = solverInfo(1);
            optimStruct.NumIterations = solverInfo(2);
            optimStruct.ExitFlag = solverInfo(3);

            % Obtatining node scales
            if isSimilar
                nodeScales = coder.nullcopy(zeros(numNodes));
                for i=1:numNodes
                    nodeScales(i) = optimNodes(i*poseDim);
                end
                optimPoseScales = nodeScales;
            else
                optimPoseScales = ones(numNodes, 1);
            end
        end

        %------------------------------------------------------------------
        % Function to get compact Information matrix if the input is a
        % similarity pose graph
        %------------------------------------------------------------------
        function infoMatCompact = compactifySimilarityInformationMatrix(infoMat)
            k = 1;
            iter = 1;
            infoMatCompact = zeros(1,28);
            for i=1:28
                infoMatCompact(1,i) = infoMat(k);
                k = k+7;
                if (k > 49)
                    k = iter+8;
                    iter = k;
                end
            end
        end

        %------------------------------------------------------------------
        % Function to get compact Information matrix if the input is not a
        % similarity pose graph
        %------------------------------------------------------------------
        function infoMatCompact = compactifyRigidInformationMatrix(infoMat)
            k = 1;
            iter = 1;
            infoMatCompact = zeros(1,21);
            for i=1:21
                infoMatCompact(1,i) = infoMat(k);
                k = k+6;
                if (k > 36)
                    k = iter+7;
                    iter = k;
                end
            end
        end
    end
end
