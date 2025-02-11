classdef ClusterAssignmentApproximateKDTree < vision.internal.kmeans.ClusterAssignmentStrategy
% ClusterAssignmentApproximateKDTree Assign features to clusters using
% approximate KD-Tree search.

% Copyright 2020-2022 The MathWorks, Inc.
%#codegen
    properties
        DistanceMetric char = 'L2'
    end

    methods
        function this = ClusterAssignmentApproximateKDTree(distMetric)
            this.DistanceMetric = distMetric;
        end

        %------------------------------------------------------------------
        function [assignments, dists, varargout] = assign(this, features, clusterCenters, randState)
            if isempty(features)
                if coder.target('MATLAB')
                    assignments = [];
                    dists       = [];
                    if nargout == 3
                        varargout{1} = [];
                    end
                else
                    assignments = zeros(coder.ignoreConst(0), 1, 'uint32');
                    dists       = zeros(coder.ignoreConst(0), 1, 'single');
                    if nargout == 3
                        varargout{1} = zeros(coder.ignoreConst(0), coder.ignoreConst(0), 'uint32');
                    end
                end
            else
                if coder.target('MATLAB')
                    searcher = vision.internal.Kdtree(this.DistanceMetric);
                elseif coder.internal.isTargetMATLABHost()
                    % Kd-tree of clusterCenters in shared library codegen mode.
                    % Construct Kdtree.
                    searcher = vision.internal.buildable.kdtreeBuildable.kdtreeConstruct(class(clusterCenters));
                else
		            % Kd-tree of clusterCenters in portable codegen mode.
                    % Construct Kdtree.
                    searcher = vision.internal.buildable.kdtreeBuildablePortable.kdtreeConstruct(class(clusterCenters));
                end

                % Set rand state explicity prior to indexing. This allows
                % the rand state to be provided as an input argument in
                % parallel code paths and ensure deterministic results.
                sPrev = rng(randState);
                if coder.target('MATLAB')
                    searcher.index(clusterCenters);
                elseif coder.internal.isTargetMATLABHost()
                    pLocationHandle = vision.internal.buildable.kdtreeBuildable.kdtreeGetLocationPointer(clusterCenters, class(clusterCenters));
                    vision.internal.buildable.kdtreeBuildable.kdtreeIndex(searcher, class(clusterCenters), pLocationHandle, size(clusterCenters, 1), size(clusterCenters, 2));
                else
                    pLocationHandle = vision.internal.buildable.kdtreeBuildablePortable.kdtreeGetLocationPointer(clusterCenters, class(clusterCenters));
                    vision.internal.buildable.kdtreeBuildablePortable.kdtreeIndex(searcher, class(clusterCenters), pLocationHandle, size(clusterCenters, 1), size(clusterCenters, 2));
                end
                rng(sPrev);

                opts.eps       = single(0);
                opts.grainSize = int32(10000);
                opts.tbbQueryThreshold = uint32(10000);

                if coder.target('MATLAB')
                    [assignments, dists, varargout{1:nargout-2}] = searcher.knnSearch(features, 1, opts); % find only the closest neighbor
                elseif coder.internal.isTargetMATLABHost()
                    [assignments, dists, varargout{1:nargout-2}] = vision.internal.buildable.kdtreeBuildable.kdtreeKNNSearch(searcher, ...
                                                                                                                             class(features), features, 1, opts);
                    deleteKDtree(searcher, pLocationHandle, class(clusterCenters));
                else
                    [assignments, dists, varargout{1:nargout-2}] = vision.internal.buildable.kdtreeBuildablePortable.kdtreeKNNSearch(searcher, ...
                        class(features), features, 1, opts);
                    deleteKDtreePortable(searcher, pLocationHandle, class(clusterCenters));
                end

                % Return assignments and dists as column vectors.
                assignments = reshape(assignments,[],1);
                dists = reshape(dists,[],1);
            end
        end

        %------------------------------------------------------------------
        function s = saveobj(this)
            s.DistanceMetric = this.DistanceMetric;
            s.Version = 1;
        end
    end

    %----------------------------------------------------------------------
    methods (Static)
        function this = loadobj(s)
            this = vision.internal.kmeans.ClusterAssignmentApproximateKDTree(s.DistanceMetric);
        end
    end

end
function deleteKDtree(kdTree, locationHandle, clusterClass)
vision.internal.buildable.kdtreeBuildable.kdtreeDeleteLocationPointer(locationHandle, clusterClass);
vision.internal.buildable.kdtreeBuildable.kdtreeDelete(kdTree, clusterClass);
end
function deleteKDtreePortable(kdTree, locationHandle, clusterClass)
vision.internal.buildable.kdtreeBuildablePortable.kdtreeDeleteLocationPointer(locationHandle, clusterClass);
vision.internal.buildable.kdtreeBuildablePortable.kdtreeDelete(kdTree, clusterClass);
end
