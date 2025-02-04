classdef kdtreeBuildable < coder.ExternalDependency
% kdtreeBuildable - encapsulate kdtree implementation library

% Copyright 2018-2024 The MathWorks, Inc.
%#codegen
    methods (Static)

        function name = getDescriptiveName(~)
            name = 'kdtreeBuildable';
        end

        function b = isSupportedContext(context)
            b = context.isMatlabHostTarget();
        end

        function updateBuildInfo(buildInfo,context)
            vision.internal.buildable.cvstBuildInfo(buildInfo, context, ...
                                                    'kdtree', ...
                                                    {'use_tbb'});
        end

        %------------------------------------------------------------------
        %                      Constructor
        %------------------------------------------------------------------
        function ptrObj = kdtreeConstruct(dataType)

            coder.inline('always');
            coder.cinclude('cvstCG_kdtree.h');
            ptrObj = coder.opaquePtr('void', coder.internal.null);

            if coder.isColumnMajor
                fcnName = ['kdtreeConstruct_' dataType];
                coder.ceval('-col', fcnName, coder.ref(ptrObj));
            else
                fcnName = ['kdtreeConstructRM_' dataType];
                coder.ceval('-row', fcnName, coder.ref(ptrObj));
            end
        end

        %------------------------------------------------------------------
        %                    NeedsReindex function
        %------------------------------------------------------------------
        function isIndexed = kdtreeNeedsReindex(kdtreeObj, dataType, locationPtr)
            coder.inline('always');
            coder.cinclude('cvstCG_kdtree.h');

            % call function
            numOut = false;
            if coder.isColumnMajor
                fcnName = ['kdtreeNeedsReindex_' dataType];
                numOut = coder.ceval('-col', fcnName, kdtreeObj, locationPtr);
            else
                fcnName = ['kdtreeNeedsReindexRM_' dataType];
                numOut = coder.ceval('-row', fcnName, kdtreeObj, locationPtr);
            end

            isIndexed = numOut;
        end

        %------------------------------------------------------------------
        %                    index function
        %------------------------------------------------------------------
        function kdtreeIndex(kdtreeObj, dataType, locationPtr, dataSize, dims, varargin)

            coder.inline('always');
            coder.cinclude('cvstCG_kdtree.h');

            % No point in indexing the kdtree with empty data
            coder.internal.errorIf(dataSize == 0, 'vision:ocvShared:emptyInput');

            % default values for optional argument
            bucketSize = 10;
            if nargin >= 6
                bucketSize = varargin{1}.bucketSize;
            end


            if coder.isColumnMajor
                fcnName = ['kdtreeIndex_' dataType];
                coder.ceval('-col', fcnName, kdtreeObj, locationPtr, uint32(dataSize), uint32(dims), bucketSize);
            else
                fcnName = ['kdtreeIndexRM_' dataType];
                coder.ceval('-row', fcnName, kdtreeObj, locationPtr, uint32(dataSize), uint32(dims), bucketSize);
            end
        end

        %------------------------------------------------------------------
        %                    Box Search function
        %------------------------------------------------------------------
        function indices = kdtreeBoxSearch(kdtreeObj, dataType, roi)
            coder.inline('always');

            coder.cinclude('cvstCG_kdtree.h');
            ptrIndices = coder.opaquePtr('void', coder.internal.null);

            % call function
            numOut = int32(0);

            if coder.isColumnMajor
                fcnName = ['kdtreeBoxSearch_' dataType];
                numOut = coder.ceval('-col', fcnName, kdtreeObj, coder.ref(roi), coder.ref(ptrIndices));
            else
                fcnName = ['kdtreeBoxSearchRM_' dataType];
                numOut = coder.ceval('-row', fcnName, kdtreeObj, roi', coder.ref(ptrIndices));
            end

            coder.varsize('indices', [inf 1]);
            indices = coder.nullcopy(zeros(numOut,1,'uint32'));

            % Copy indices to output
            coder.ceval('-layout:any', 'kdtreeBoxSearchSetOutputs', ptrIndices, coder.ref(indices));

        end

        %------------------------------------------------------------------
        %                    KNN Search function
        %------------------------------------------------------------------
        function [indices, dists, valid] = kdtreeKNNSearch(kdtreeObj, dataType, queryPoints, K, searchOpts)
            coder.inline('always');

            coder.cinclude('cvstCG_kdtree.h');
            numQueries = uint32(size(queryPoints, 1));
            numQueryDims = uint32(size(queryPoints, 2));

            % check searchOpts Inputs
            coder.internal.errorIf((~isfield(searchOpts,'eps')),'vision:ocvShared:invalidInputClass');
            coder.internal.errorIf((searchOpts.eps < 0 ),'vision:ocvShared:invalidInputClass');

            % default values for optional argument
            grainSize = int32(2000);
            tbbQueryThreshold = uint32(500);

            if isfield(searchOpts, 'grainSize')
                grainSize = int32(searchOpts.grainSize);
            end
            if isfield(searchOpts, 'tbbQueryThreshold')
                tbbQueryThreshold = uint32(searchOpts.tbbQueryThreshold);
            end

            searchOptsEps    = cast(searchOpts.eps, dataType);

            coder.varsize('indices', [inf inf]);
            coder.varsize('dists', [inf inf]);
            coder.varsize('valid', [inf 1]);

            indices = coder.nullcopy(zeros(K, numQueries, 'uint32'));
            dists = coder.nullcopy(zeros(K, numQueries, dataType));
            valid = coder.nullcopy(zeros(numQueries, 1, 'uint32'));

            if coder.isColumnMajor
                fcnName = ['kdtreeKNNSearch_' dataType];
                coder.ceval('-col', fcnName, kdtreeObj, coder.ref(queryPoints), uint32(numQueries), uint32(numQueryDims), uint32(K), ...
                            searchOptsEps, coder.ref(indices), coder.ref(dists), coder.ref(valid), int32(grainSize), uint32(tbbQueryThreshold));
            else
                fcnName = ['kdtreeKNNSearchRM_' dataType];
                coder.ceval('-row', fcnName, kdtreeObj, queryPoints', uint32(numQueries), uint32(numQueryDims), uint32(K), ...
                            searchOptsEps, coder.ref(indices), coder.ref(dists), coder.ref(valid), int32(grainSize), uint32(tbbQueryThreshold));
            end
        end

        %------------------------------------------------------------------
        %                    radius Search function
        %------------------------------------------------------------------
        function [indices, dists, valid] = kdtreeRadiusSearch(kdtreeObj, dataType, queryPoints, radius, searchOpts)
            coder.inline('always');
            coder.cinclude('cvstCG_kdtree.h');

            numQueries = uint32(size(queryPoints, 1));
            numQueryDims = uint32(size(queryPoints, 2));

            % check searchOpts Inputs
            coder.internal.errorIf((~isfield(searchOpts,'eps')),'vision:ocvShared:invalidInputClass');
            coder.internal.errorIf((searchOpts.eps < 0 ),'vision:ocvShared:invalidInputClass');

            ptrIndices = coder.opaquePtr('void', coder.internal.null);
            ptrDists = coder.opaquePtr('void', coder.internal.null);

            searchOptsEps    = cast(searchOpts.eps, dataType);

            % default values for optional argument
            grainSize = int32(1000);
            tbbQueryThreshold = uint32(500);

            if isfield(searchOpts, 'grainSize')
                grainSize = int32(searchOpts.grainSize);
            end

            if isfield(searchOpts, 'tbbQueryThreshold')
                tbbQueryThreshold = int32(searchOpts.tbbQueryThreshold);
            end

            radius_ = cast(radius, dataType);

            valid = coder.nullcopy(zeros(numQueries, 1, 'uint32'));

            % call function
            if coder.isColumnMajor
                fcnName = ['kdtreeRadiusSearch_' dataType];
                coder.ceval('-col', fcnName, kdtreeObj, coder.ref(queryPoints), numQueries, numQueryDims,...
                            radius_, searchOptsEps, coder.ref(ptrIndices), coder.ref(ptrDists), coder.ref(valid), grainSize, tbbQueryThreshold);
            else
                fcnName = ['kdtreeRadiusSearchRM_' dataType];
                coder.ceval('-row', fcnName, kdtreeObj, queryPoints', numQueries, numQueryDims,...
                            radius_, searchOptsEps, coder.ref(ptrIndices), coder.ref(ptrDists), coder.ref(valid), grainSize, tbbQueryThreshold);
            end

            indices_ = coder.nullcopy(zeros(sum(valid),1,'uint32'));
            dists_ = coder.nullcopy(zeros(sum(valid),1, dataType));

            % Copy indices to output
            coder.ceval('-layout:any', ['kdtreeRadiusSearchSetOutputs_' dataType], ptrIndices, ptrDists, ...
                        coder.ref(indices_), coder.ref(dists_));

            cumSumValid = cumsum(valid);

            if coder.internal.isConst(size(queryPoints)) && (numQueries == 1) && (numQueryDims == 3)
                indices = indices_;
                dists = dists_;
            else
                indices = coder.nullcopy(cell(numQueries, 1));
                dists = coder.nullcopy(cell(numQueries, 1));

                for i = 1:numQueries
                    if i == 1
                        if valid(1) ~= 0
                            indices{i} = indices_(1:cumSumValid(i));
                            dists{i} = dists_(1:cumSumValid(i));
                        end
                    else
                        if valid(i) ~= 0
                            indices{i} = indices_(cumSumValid(i-1)+1:cumSumValid(i));
                            dists{i} = dists_(cumSumValid(i-1)+1:cumSumValid(i));
                        end
                    end
                end
            end
        end


        %------------------------------------------------------------------
        %                   Hybrid Search function
        %------------------------------------------------------------------
        function [indices, dists, valid] = kdtreeHybridSearch(kdtreeObj, dataType, queryPoints, K, radius, searchOpts)
            coder.inline('always');

            coder.cinclude('cvstCG_kdtree.h');
            numQueries = uint32(size(queryPoints, 1));
            numQueryDims = uint32(size(queryPoints, 2));

            % check searchOpts Inputs
            coder.internal.errorIf((~isfield(searchOpts,'eps')),'vision:ocvShared:invalidInputClass');
            coder.internal.errorIf((searchOpts.eps < 0 ),'vision:ocvShared:invalidInputClass');

            % default values for optional argument
            grainSize = int32(2000);
            tbbQueryThreshold = uint32(500);

            if isfield(searchOpts, 'grainSize')
                grainSize = int32(searchOpts.grainSize);
            end
            if isfield(searchOpts, 'tbbQueryThreshold')
                tbbQueryThreshold = uint32(searchOpts.tbbQueryThreshold);
            end

            radius_ = cast(radius, dataType);

            searchOptsEps    = cast(searchOpts.eps, dataType);

            coder.varsize('indices', [inf inf]);
            coder.varsize('dists', [inf inf]);
            coder.varsize('valid', [inf 1]);

            indices = coder.nullcopy(zeros(K, numQueries, 'uint32'));
            dists = coder.nullcopy(zeros(K, numQueries, dataType));
            valid = coder.nullcopy(zeros(numQueries, 1, 'uint32'));

            if coder.isColumnMajor
                fcnName = ['kdtreeHybridSearch_' dataType];
                coder.ceval('-col', fcnName, kdtreeObj, coder.ref(queryPoints), uint32(numQueries), uint32(numQueryDims), uint32(K), ...
                            radius_, searchOptsEps, coder.ref(indices), coder.ref(dists), coder.ref(valid), int32(grainSize), uint32(tbbQueryThreshold));
            else
                fcnName = ['kdtreeHybridSearchRM_' dataType];
                coder.ceval('-row', fcnName, kdtreeObj, queryPoints', uint32(numQueries), uint32(numQueryDims), uint32(K), ...
                            radius_, searchOptsEps, coder.ref(indices), coder.ref(dists), coder.ref(valid), int32(grainSize), uint32(tbbQueryThreshold));
            end
        end        

        %------------------------------------------------------------------
        %                    function for deleting kdtree obj
        %------------------------------------------------------------------
        function kdtreeDelete(kdtreeObj, dataType)

            coder.inline('always');
            coder.cinclude('cvstCG_kdtree.h');

            if coder.isColumnMajor
                fcnName = ['kdtreeDeleteObj_'  dataType];
                coder.ceval('-col', fcnName, kdtreeObj);
            else
                fcnName = ['kdtreeDeleteObjRM_'  dataType];
                coder.ceval('-row', fcnName, kdtreeObj);
            end
        end

        %------------------------------------------------------------------
        %                    function to get location data pointer
        %------------------------------------------------------------------
        function locationPtr = kdtreeGetLocationPointer(location, dataType)

            coder.inline('always');
            coder.cinclude('cvstCG_kdtree.h');
            locationPtr = coder.opaquePtr('void', coder.internal.null);

            if ~isempty(location)
                if ismatrix(location)
                    dataSize = uint32(size(location, 1));
                    dims = uint32(size(location, 2));
                else
                    dataSize = uint32(size(location, 1) * size(location, 2));
                    dims = uint32(size(location, 3));
                end

                if coder.isColumnMajor
                    coder.ceval('-col', ['kdtreeGetLocationDataPointer_' dataType], coder.ref(location), dataSize, dims, coder.ref(locationPtr));
                else
                    if ismatrix(location)
                        coder.ceval('-row', ['kdtreeGetLocationDataPointer_' dataType], coder.ref(location), dataSize, dims, coder.ref(locationPtr));
                    else
                        locationT = permute(location, [2 1 3]);
                        coder.ceval('-row', ['kdtreeGetLocationDataPointer_' dataType], coder.ref(locationT), dataSize, dims, coder.ref(locationPtr));
                    end
                end
            end
        end

        %------------------------------------------------------------------
        %                    function for deleting location data pointer
        %------------------------------------------------------------------
        function kdtreeDeleteLocationPointer(locationPtr, dataType)
            coder.inline('always');
            coder.cinclude('cvstCG_kdtree.h');

            coder.ceval('-layout:any', ['kdtreeDeleteLocationDataPointer_' dataType], locationPtr);
        end
    end
end
