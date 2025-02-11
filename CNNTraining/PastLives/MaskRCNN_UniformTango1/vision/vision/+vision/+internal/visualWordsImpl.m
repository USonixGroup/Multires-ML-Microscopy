% visualWordsImpl has code common for simulation and codegen
% for vision.internal.visualWords

%#codegen

%   Copyright 2022 The MathWorks, Inc.

classdef visualWordsImpl
    properties(GetAccess = public, SetAccess = protected)
        % WordIndex - A vector of visual word identifiers
        WordIndex
        % Location - Visual word locations within an image.
        Location
        % VocabularySize - Number of visual words in the vocabulary.
        VocabularySize
    end

    % ---------------------------------------------------------------------
    properties(Dependent)
        % Count - Number of visual words held by the object.
        Count
    end

    methods
        % -----------------------------------------------------------------
        function this = visualWordsImpl(words, locations, sz)
            if nargin > 0
                vision.internal.visualWordsImpl.checkInputs(words, locations, sz);

                this.WordIndex      = uint32(words(:));
                this.Location       = locations;
                this.VocabularySize = double(full(sz));
            end
        end

        %------------------------------------------------------------------
        function n = get.Count(this)
            n = numel(this.WordIndex);
        end
    end
    methods(Hidden, Static, Access=private)
        % -----------------------------------------------------------------
        function checkInputs(words, locations, sz)

            validateattributes(words, {'numeric'},...
                               {'vector','nonsparse','real'},...
                               'visualWords', 'WordIndex', 1);

            validateattributes(locations, {'single'},...
                               {'size', [NaN 2], 'nonsparse', 'real'},...
                               'visualWords', 'Location', 2);

            validateattributes(sz, {'numeric'}, ...
                               {'scalar', 'positive', 'integer', 'real'}, ...
                               'visualWords', 'VocabularySize', 3);

            coder.internal.errorIf(numel(words) ~= size(locations,1), ...
                    'vision:visualWords:invalidNumelWords');
        end
    end

    % ---------------------------------------------------------------------
    methods(Hidden, Static, Access=protected)
        % -----------------------------------------------------------------
        function h = createSparseHistogram(words)
            w = words.WordIndex;
            h = sparse(1,double(w),1,1,words.VocabularySize);
        end

        % -----------------------------------------------------------------
        function checkScalar(val,name)
            validateattributes(val, {'numeric'},...
                               {'scalar','integer','positive','real','nonsparse'},...
                               'visualWords', name);
        end
    end
end
