classdef scanContextLoopDetector < handle

    % Copyright 2021-2023 The MathWorks, Inc.

    %#codegen

    properties (SetAccess = private)
        ViewIds
        Descriptors
        DescriptorsSingle
    end

    properties (Access = {?matlab.unittest.TestCase})
        % N-by-P matrix of ring key descriptors extracted from scan context
        % descriptors.
        RingKeyDescriptors

        % N-by-P matrix of ring key descriptors extracted from scan context
        % descriptors initialized in codegen mode if class of descriptors
        % is single
        RingKeyDescriptorsSingle

        % Number of concentric radial bins used to extract the scan context
        % descriptors.
        NumRadialBins

        % Number of angular bins used to extract the scan context
        % descriptors.
        NumAzimuthalBins
    end

    properties (Access = protected) % For shared library codegen
        HasLocationHandleAllocated = false
        LocationHandle
        Kdtree
        Location
    end

    properties (Constant, Access = private)
        Version = 1.0;
    end

    properties (GetAccess = private, SetAccess = protected)
        outputClass
    end

    methods(Static)
        function props = matlabCodegenNontunableProperties(~)
            props = {'outputClass'}; % Used for code generation.
        end
    end

    methods
        %------------------------------------------------------------------
        function this = scanContextLoopDetector 
            if isSimMode()
                this.ViewIds = zeros(1, 0, 'uint32');
            else
                this.ViewIds = zeros(coder.ignoreConst(0), 'uint32');
                this.NumAzimuthalBins = zeros(coder.ignoreConst(0));
                this.NumRadialBins = zeros(coder.ignoreConst(0));

                % Initializing descriptors as variable size.
                this.Descriptors = zeros(coder.ignoreConst(0), ...
                    coder.ignoreConst(0), coder.ignoreConst(0));
                this.DescriptorsSingle = zeros(coder.ignoreConst(0), ...
                    coder.ignoreConst(0), coder.ignoreConst(0), 'single');

                % Initializing RingKeyDescriptors as variable size.
                this.RingKeyDescriptorsSingle = zeros(coder.ignoreConst(0), ...
                    coder.ignoreConst(0), 'single');
                this.RingKeyDescriptors = zeros(coder.ignoreConst(0), ...
                    coder.ignoreConst(0));
            end
        end
        
        %------------------------------------------------------------------
        function addDescriptor(this, viewId, descriptor)

            narginchk(3, 3)

            validateLoopDetector(this);

            % Validate viewId input.
            viewId = vision.internal.inputValidation.checkViewIds(...
                viewId, true, 'addDescriptor', 'viewId');

            coder.internal.errorIf(vision.internal.inputValidation.checkIfHasView(this.ViewIds, viewId), ...
                'vision:viewSet:viewIdAlreadyExists', viewId);

            % Set the number of bins properties when adding the first descriptor.
            if isempty(this.NumRadialBins)
                [this.NumRadialBins, this.NumAzimuthalBins] = size(descriptor); 
            end
             
            % Validate descriptor input.
            validateDescriptor(this, descriptor, 'addDescriptor')

            % Set viewId.
            this.ViewIds = [this.ViewIds; viewId];

            % Set the scan context descriptor.
            
            if isSimMode()
                % In Simulation mode, only 'Descriptors' is initialized.
                this.outputClass = class(descriptor);
                if isempty(this.Descriptors)
                    this.Descriptors = descriptor;
                else
                    this.Descriptors = cat(3, this.Descriptors, descriptor);
                end
            else
                if isa(descriptor, 'single')
                    % Initialize DescriptorsSingle in codegen mode if class of
                    % descriptors is single.
                    this.outputClass = 'single';
                    if isempty(this.DescriptorsSingle)
                        this.DescriptorsSingle = descriptor;
                    else
                        this.DescriptorsSingle = cat(3, this.DescriptorsSingle, descriptor);
                    end
                else
                    % Initialize Descriptors for class 'double'.
                    this.outputClass = 'double';
                    if isempty(this.Descriptors)
                        this.Descriptors = descriptor;
                    else
                        this.Descriptors = cat(3, this.Descriptors, descriptor);
                    end
                end
            end

            % Compute the ring key subdescriptor.
            numBins = this.NumAzimuthalBins;
            sumNanDescriptors = sum(isnan(descriptor), 2, 'double');
            ringKey = (repmat(numBins, size(sumNanDescriptors)) - sumNanDescriptors) / numBins;
            ringKeySingle = cast(ringKey, 'single');

            if isSimMode()
                this.RingKeyDescriptors = [this.RingKeyDescriptors, cast(ringKey, 'like', descriptor)];
            else
                if isa(descriptor, 'single')
                    this.RingKeyDescriptorsSingle = [this.RingKeyDescriptorsSingle, ringKeySingle];
                else
                    this.RingKeyDescriptors = [this.RingKeyDescriptors, ringKey];
                end
            end
        end

        function deleteDescriptor(this, viewIds)

            narginchk(2, 2)
            validateLoopDetector(this);

            % Validate viewIds.
            viewIds = vision.internal.inputValidation.checkViewIds(...
                viewIds, false, 'deleteDescriptor', 'viewIds');
            vision.internal.inputValidation.checkIfViewIsMissing(this.ViewIds, viewIds);

            idx = ismember(this.ViewIds, viewIds);
            this.ViewIds(idx) = [];

            if isSimMode()
                this.Descriptors(:, :, idx) = [];
                this.RingKeyDescriptors(:, idx) = [];
            else
                if isempty(this.Descriptors)
                    this.DescriptorsSingle(:, :, idx) = [];
                    this.RingKeyDescriptorsSingle(:, idx) = [];
                else
                    this.Descriptors(:, :, idx) = [];
                    this.RingKeyDescriptors(:, idx) = [];
                end
            end
            if isempty(this.ViewIds)
                this.Descriptors = [];
                this.DescriptorsSingle = single([]);
                this.NumAzimuthalBins = zeros(coder.ignoreConst(0));
                this.NumRadialBins = zeros(coder.ignoreConst(0));
            end
        end

        function [loopViewIds, dists] = detectLoop(this, varargin)

            narginchk(1, 10)
            validateLoopDetector(this);

            coder.internal.errorIf(isempty(this.ViewIds), ...
                'vision:pointcloud:detectorHasNoDescriptors');

            if nargin > 1 && ~(isstring(varargin{1}) || ischar(varargin{1}))
                % Validate inputs for detectLoop(loopDetector, descriptor)
                % syntax.
                scanContext = varargin{1};
                validateDescriptor(this, scanContext, 'detectLoop');
                scanContextSingle = cast(scanContext, 'single');
                scanContextDouble = cast(scanContext, 'double');
                if isSimMode()
                    args = parseOptionalDetectLoopSim(varargin{2:end});
                else
                    args = parseOptionalDetectLoopCodegen(varargin{2:end});
                end
                
                % Validating optional NV pairs.
                validateDistanceThreshold(args.DistanceThreshold);
                validateNumExcludedDescriptors(args.NumExcludedDescriptors);
                validateSearchRadius(args.SearchRadius);
                validateMaxDetections(args.MaxDetections);
                numExcludedDescriptors = args.NumExcludedDescriptors;

                % Compute ring key subdescriptor for descriptor input.
                numBins = this.NumAzimuthalBins;
                sumNanDescriptors = sum(isnan(scanContextDouble), 2);
                numBinsD = double(numBins);
                ringKey = ((repmat(numBinsD, size(sumNanDescriptors)) - sumNanDescriptors) / numBinsD);
                ringKeySingle = cast(ringKey, 'single');
            else
                % Validate inputs for detectLoop(loopDetector) syntax.
                if isSimMode()
                    args = parseOptionalDetectLoopSim(varargin{:});
                else
                    args = parseOptionalDetectLoopCodegen(varargin{:});
                end

                % Validating optional NV pairs.
                validateDistanceThreshold(args.DistanceThreshold);
                validateNumExcludedDescriptors(args.NumExcludedDescriptors);
                validateSearchRadius(args.SearchRadius);
                validateMaxDetections(args.MaxDetections);

                if ~isSimMode() && isempty(this.Descriptors)
                    scanContextSingle = this.DescriptorsSingle(:, :, end);
                    scanContextDouble = cast(scanContextSingle, 'double');
                    ringKeySingle = this.RingKeyDescriptorsSingle(:, end);
                    ringKey = cast(ringKeySingle, 'double');

                elseif ~isSimMode()
                    scanContextDouble = this.Descriptors(:, :, end);
                    scanContextSingle = cast(scanContextDouble, 'single');
                    ringKey = this.RingKeyDescriptors(:, end);
                    ringKeySingle = cast(ringKey, 'single');
                else
                    scanContextDouble = cast(this.Descriptors(:, :, end), 'double');
                    scanContextSingle = cast(scanContextDouble, 'single');
                    ringKey = this.RingKeyDescriptors(:, end);
                    ringKeySingle = cast(ringKey, 'single');
                end

                % Exclude the last added descriptor from the search for
                % detectLoop(loopDetector) syntax.
                numExcludedDescriptors = args.NumExcludedDescriptors + 1;
            end

            loopViewIds = uint32([]);
            dists = zeros(0, this.outputClass);

            if numel(this.ViewIds) < (numExcludedDescriptors + 1)
                return
            end

            if isempty(this.RingKeyDescriptorsSingle)
                descriptors = double(this.RingKeyDescriptors(:, 1:end-numExcludedDescriptors)');
            else
                descriptors = double(this.RingKeyDescriptorsSingle(:, 1:end-numExcludedDescriptors)');
            end
            if isSimMode()
                % Build a kd-tree of descriptors.
                this.Kdtree = vision.internal.Kdtree();
                this.Kdtree.index(descriptors);
            elseif coder.internal.isTargetMATLABHost()
                % Kd-tree of descriptors in codegen mode.
                % Allocate Location Pointer
                this.LocationHandle = vision.internal.buildable.kdtreeBuildable.kdtreeGetLocationPointer(descriptors, class(descriptors));
                % Construct Kdtree.
                this.Kdtree = vision.internal.buildable.kdtreeBuildable.kdtreeConstruct(class(descriptors));
                % Index Kdtree.
                vision.internal.buildable.kdtreeBuildable.kdtreeIndex(this.Kdtree, class(descriptors), this.LocationHandle, size(descriptors, 1), size(descriptors, 2));
                this.HasLocationHandleAllocated = true;
            else
                % Allocate Location Pointer
                this.LocationHandle = vision.internal.buildable.kdtreeBuildablePortable.kdtreeGetLocationPointer(descriptors, class(descriptors));
                % Construct Kdtree.
                this.Kdtree = vision.internal.buildable.kdtreeBuildablePortable.kdtreeConstruct(class(descriptors));
                % Index Kdtree.
                vision.internal.buildable.kdtreeBuildablePortable.kdtreeIndex(this.Kdtree, class(descriptors), this.LocationHandle, size(descriptors, 1), size(descriptors, 2));
                this.HasLocationHandleAllocated = true;
            end


            % Find loop closure candidates.
            maxDetections = args.MaxDetections;
            if ~isfinite(maxDetections)
                maxDetections = numel(this.ViewIds) - numExcludedDescriptors;
            end
            if isSimMode()
                [idxViewIds, distSquared] = this.Kdtree.knnSearch(double(ringKey'), double(maxDetections));
            elseif coder.internal.isTargetMATLABHost()
                searchOpts.eps = 0;
                if isa(descriptors, 'single')
                    [idxViewIds, distSquared] = vision.internal.buildable.kdtreeBuildable.kdtreeKNNSearch(this.Kdtree, 'single', double(ringKeySingle'), double(maxDetections), searchOpts);
                else
                    [idxViewIds, distSquared] = vision.internal.buildable.kdtreeBuildable.kdtreeKNNSearch(this.Kdtree, 'double', double(ringKey'), double(maxDetections), searchOpts);
                end
            else
                if isa(descriptors, 'single')
                    [idxViewIds, distSquared] = vision.internal.buildable.kdtreeBuildablePortable.kdtreeKNNSearch(this.Kdtree, 'single', double(ringKeySingle'), double(maxDetections), searchOpts);
                else
                    [idxViewIds, distSquared] = vision.internal.buildable.kdtreeBuildablePortable.kdtreeKNNSearch(this.Kdtree, 'double', double(ringKey'), double(maxDetections), searchOpts);
                end
            end

            isLoopCandidate = (idxViewIds ~= 0) & (distSquared <= args.SearchRadius^2);

            if any(isLoopCandidate)
                % Select loop closure candidate view ids.
                selectedIdxViewIds = idxViewIds(isLoopCandidate);
                loopCandidateViewIds = this.ViewIds(selectedIdxViewIds);

                % Find the scan context distance for each candidate.
                numCandidates = numel(loopCandidateViewIds);
                scanContextDists = zeros(numCandidates, 1, this.outputClass);
                for i = 1:numCandidates
                    % Compute the scan context distance.
                    if isSimMode()
                        scanContextDists(i, 1) = scanContextDistance(cast(scanContextSingle, this.outputClass), ...
                            this.Descriptors(:, :, selectedIdxViewIds(i)));
                    elseif ~isSimMode() && strcmp(this.outputClass, 'single')
                        scanContextDists(i, 1) = scanContextDistance(scanContextSingle, ...
                            this.DescriptorsSingle(:, :, selectedIdxViewIds(i)));
                    else
                        scanContextDists(i, 1) = scanContextDistance(scanContextDouble, ...
                            this.Descriptors(:, :, selectedIdxViewIds(i)));
                    end
                end

                % Threshold the scan context distance.
                selectedLoops = scanContextDists <= args.DistanceThreshold;
                if ~any(selectedLoops)
                    loopViewIds = uint32([]);
                    dists = zeros(0, this.outputClass);
                    return
                end

                dists = scanContextDists(selectedLoops);
                loopViewIds = loopCandidateViewIds(selectedLoops);

                % Sort outputs starting from the best detection.
                [dists, indices] = sort(dists);
                loopViewIds = loopViewIds(indices);
            end
        end
    end

    methods (Access = protected)
        function validateDescriptor(this, descriptor, methodName)
            validateattributes(descriptor, {'single', 'double'}, ...
                {'2d', 'nonempty', 'real', 'nonsparse', 'size', ...
                [this.NumRadialBins, this.NumAzimuthalBins]}, ...
                methodName, 'descriptor');
        end
    end
end

%--------------------------------------------------------------------------
function loopDetector = validateLoopDetector(loopDetector)
    validateattributes(loopDetector, {'scanContextLoopDetector'}, ...
        {'size', [1, 1]})
end

%--------------------------------------------------------------------------
function args = parseOptionalDetectLoopSim(args)
arguments
    args.DistanceThreshold = 0.1;
    args.NumExcludedDescriptors = 30;
    args.SearchRadius = 0.3;
    args.MaxDetections = 3;
end

end

%--------------------------------------------------------------------------
function args = parseOptionalDetectLoopCodegen(varargin)
% Parsing optional Detect loop NV pairs.
coder.inline('always');
coder.internal.prefer_const(varargin);
    nvPairNames = {'DistanceThreshold', ...
        'NumExcludedDescriptors', ...
        'SearchRadius', ...
        'MaxDetections'};
    poptions = struct( ...
    'CaseSensitivity', false, ...
    'PartialMatching', 'unique', ...
    'StructExpand', false, ...
    'IgnoreNulls', false, ...
    'SupportOverrides', false);
    
    % define default values for NV pairs
    defaults = struct('DistanceThreshold', 0.1, ...
        'NumExcludedDescriptors', 30, ...
        'SearchRadius', 0.3, ...
        'MaxDetections', 3);
    pstruct = coder.internal.parseInputs({}, nvPairNames, poptions, varargin{:});

    args.DistanceThreshold = coder.internal.getParameterValue(pstruct.DistanceThreshold, ...
        defaults.DistanceThreshold, varargin{:});
    args.NumExcludedDescriptors = coder.internal.getParameterValue(pstruct.NumExcludedDescriptors, ...
        defaults.NumExcludedDescriptors, varargin{:});
    args.SearchRadius = coder.internal.getParameterValue(pstruct.SearchRadius, ...
        defaults.SearchRadius, varargin{:});
    args.MaxDetections = coder.internal.getParameterValue(pstruct.MaxDetections, ...
        defaults.MaxDetections, varargin{:});

end

%--------------------------------------------------------------------------
function validateDistanceThreshold(in)

validateattributes(in, {'numeric'}, ...
    {'scalar', 'nonsparse', 'real', '>', 0, '<=', 1}, ...
    'detectLoop', 'DistanceThreshold')

end

%--------------------------------------------------------------------------
function validateNumExcludedDescriptors(in)

validateattributes(in, {'numeric'}, ...
    {'scalar', 'nonsparse', 'real', 'integer', 'nonnegative', 'finite'}, ...
    'detectLoop', 'NumExcludedDescriptors')

end

%--------------------------------------------------------------------------
function validateSearchRadius(in)

validateattributes(in, {'numeric'}, ...
    {'scalar', 'nonsparse', 'real', 'positive', 'nonnan'}, ...
    'detectLoop', 'SearchRadius')

end

%--------------------------------------------------------------------------
function validateMaxDetections(in)

if isfinite(in)
    validateattributes(in, {'numeric'}, ...
        {'scalar', 'nonsparse', 'real', 'nonnan', 'integer', 'positive'}, ...
        'detectLoop', 'MaxDetections');
else
    validateattributes(in, {'numeric'}, {'scalar', 'nonnan', 'positive'}, ...
        'detectLoop', 'MaxDetections');
end

end

%------------------------------------------------------------------
% delete function
%------------------------------------------------------------------
function delete(this)
if this.HasLocationHandleAllocated
    if coder.internal.isTargetMATLABHost() % Shared Lib code
        vision.internal.buildable.kdtreeBuildable.kdtreeDeleteLocationPointer(this.LocationHandle, class(this.Location));
        vision.internal.buildable.kdtreeBuildable.kdtreeDelete(this.Kdtree, class(this.Location));
    else % Portable Code
        vision.internal.buildable.kdtreeBuildablePortable.kdtreeDeleteLocationPointer(this.LocationHandle, class(this.Location));
        vision.internal.buildable.kdtreeBuildablePortable.kdtreeDelete(this.Kdtree, class(this.Location));      
    end
    this.HasLocationHandleAllocated = false;
end
end

%--------------------------------------------------------------------------
function out = isSimMode()
    % check if simulation mode
    out = isempty(coder.target);
end