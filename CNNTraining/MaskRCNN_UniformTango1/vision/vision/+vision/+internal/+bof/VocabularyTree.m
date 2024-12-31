classdef VocabularyTree
% VocabularyTree Container for a visual vocabulary tree.

% Copyright 2021-2022 The MathWorks, Inc.
%#codegen

    properties
        % Words The visual words. Stored as a numeric array.
        Words

        % Nodes An array of nodes of the tree. Each node is a
        %       VocabularyTree object.
        Nodes (:,1) = vision.internal.bof.VocabularyTree.empty();
    end

    methods
        %------------------------------------------------------------------
        function n = numVisualWords(this)
        % Return the number of visual words at the tree leaves. This is
        % the number of visual words used during the bagOfFeatures
        % encode step.
            if isempty(this.Nodes)
                n = size(this.Words,1);
            else
                n = 0;
                for i = 1:numel(this.Nodes)
                    n = n + numVisualWords(this.Nodes(i));
                end
            end
        end

        %------------------------------------------------------------------
        function n = maxNumLevels(this)
        % Returns the maximum number of levels in the tree.
            if isempty(this.Nodes)
                n = 1;
            else
                n = ones(1,numel(this.Nodes));
                for i = 1:numel(this.Nodes)
                    n(i) = n(i) + maxNumLevels(this.Nodes(i));
                end
                n = max(n);
            end
        end

        %------------------------------------------------------------------
        function n = maxBranchingFactor(this)
        % Return the maximum branching factor in the tree. Some nodes
        % in the tree may not have the same branching factor, hence the
        % need to return the max.
            n = size(this.Words,1);
            for i = 1:numel(this.Nodes)
                n = max(n, maxBranchingFactor(this.Nodes(i)));
            end

        end
    end
end
