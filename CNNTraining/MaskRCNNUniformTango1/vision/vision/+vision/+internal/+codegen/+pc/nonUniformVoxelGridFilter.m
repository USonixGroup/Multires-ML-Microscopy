function [indices, varargout] = nonUniformVoxelGridFilter(location, maxVoxelPoints)
%   vision.internal.codegen.pc.nonUniformVoxelGridFilter is used in pcdownsample
%   to sample with nonUniformGridSample method. This file generates
%   portable code.
%   Shared library code generation version is found in
%   vision.internal.buildable.nonUniformVoxelGridFilterBuildable.
%
%   Inputs:
%   -------
%    location       : input point location, Mx3 or MxNx3 matrix.
%    maxVoxelPoints : the maximum number of points in voxel.
%
%   Outputs:
%   -------
%    indices        : Nx1 indices of filtered points.
%    normal         : Nx3 filtered normals if the input normal is empty.

% Copyright 2021-2023 The MathWorks, Inc.
%#codegen

    coder.inline('always');
    
    needNormalOutput = nargout >= 2;
    coder.internal.prefer_const(needNormalOutput);
    
    if ((~needNormalOutput && maxVoxelPoints < 2) || (needNormalOutput && maxVoxelPoints < 3))
        coder.internal.error('vision:ocvShared:invalidInputClass');
    end
    
    if ismatrix(location)
        numPoints = uint32(size(location, 1));
    else
        numPoints = uint32(size(location, 1) * size(location, 2));
    end
    
    if needNormalOutput
        [indices,normal] = nonUniformVoxelGridFilterImpl(location,numPoints,maxVoxelPoints);
        varargout{1} = normal;
    else  % We don't need normal output.
        indices = nonUniformVoxelGridFilterImpl(location,numPoints,maxVoxelPoints);
    end
end

%==========================================================================
% Implementation of non-uniform voxel grid filter
%==========================================================================
function [outIndices, varargout] = nonUniformVoxelGridFilterImpl(location,numPoints,maxVoxelPoints)
    needNormalOutput = nargout >= 2;
    coder.internal.prefer_const(needNormalOutput);
    
    minPoint = coder.nullcopy(zeros([3 1], 'like', location));
    maxPoint = coder.nullcopy(zeros([3 1], 'like', location));
    
    minPoint(1) = inf(class(location));
    minPoint(2) = inf(class(location));
    minPoint(3) = inf(class(location));
    maxPoint(1) = -inf(class(location));
    maxPoint(2) = -inf(class(location));
    maxPoint(3) = -inf(class(location));
    
    % Attach the indices to locations to be later used for coordinate ordering.
    indexVector = coder.nullcopy(zeros([numPoints 1], 'uint32'));
    % Find the PC boundaries.
    for i=1:numPoints
        indexVector(i) = uint32(i);
        x = location(i);
        y = location(i + numPoints);
        z = location(i + numPoints*2);
        
        if ~isfinite(x) || ~isfinite(y) || ~isfinite(z)
            continue;
        end
        
        minPoint(1) = min(x,minPoint(1));
        minPoint(2) = min(y,minPoint(2));
        minPoint(3) = min(z,minPoint(3));
        maxPoint(1) = max(x,maxPoint(1));
        maxPoint(2) = max(y,maxPoint(2));
        maxPoint(3) = max(z,maxPoint(3));
    end
    
    % Calculate upper bound of the size of the output indices.
    outIndicesSize = uint32(2^ceil(log2(double(idivide(numPoints,uint32(maxVoxelPoints))))));
    mOutIndices = coder.nullcopy(zeros([outIndicesSize 1], 'uint32'));
    mOutIndicesEnd = uint32(1);
    
    % Recursively partition the voxel grid.
    if needNormalOutput
        mNormal = coder.nullcopy(zeros(3,outIndicesSize, 'like', location));
        mNormalEnd = uint32(1);

        % coder.ignoreConst instructs MATLAB Coder NOT to attempt to make
        % the parameter a run-time constant. This is necessary for this 
        % recursive fcn, because the the number of call is unlimited and
        % MATLAB Coder could stall trying to analyze every fcuntion call.
        [indexVector,mOutIndices,mOutIndicesEnd,mNormal] = partition(indexVector, ...
                                                         numPoints, ...
                                                         coder.ignoreConst(uint32(1)), ...
                                                         coder.ignoreConst(numPoints+1), ...
                                                         minPoint, ...
                                                         maxPoint, ...
                                                         location, ...
                                                         mOutIndices, ...
                                                         mOutIndicesEnd, ...
                                                         maxVoxelPoints, ...
                                                         mNormal, ...
                                                         mNormalEnd);
    else
        [indexVector,mOutIndices,mOutIndicesEnd] = partition(indexVector, ...
                                                 numPoints, ...
                                                 coder.ignoreConst(uint32(1)), ...
                                                 coder.ignoreConst(numPoints+1), ...
                                                 minPoint, ...
                                                 maxPoint, ...
                                                 location, ...
                                                 mOutIndices, ...
                                                 mOutIndicesEnd, ...
                                                 maxVoxelPoints);
    end
    
    % Write indices and normal to output.
    nOutSize = mOutIndicesEnd - 1;
    outIndices = coder.nullcopy(zeros([nOutSize 1], 'double'));
    if needNormalOutput
        normal = coder.nullcopy(zeros([nOutSize 3], 'like', location));
    end
    for i=1:nOutSize
        outIndices(i) = mOutIndices(i);
        if needNormalOutput
            normal(i) = mNormal((i - 1) * 3 + 1);
            normal(i+nOutSize) = mNormal((i - 1) * 3 + 2);
            normal(i+nOutSize*2) = mNormal((i - 1) * 3 + 3);
        end
    end
    if needNormalOutput
        varargout{1} = normal;
    end
end

%==========================================================================
% Recursive partition
%==========================================================================
function [indexVector,mOutIndices,mOutIndicesEnd,varargout] = partition(indexVector,numPoints,first,last,minPoint,maxPoint,location,mOutIndices,mOutIndicesEnd,maxVoxelPoints,varargin)
    % Recursively partition indices in range [first, last).
    needNormalOutput = nargout >= 4;
    coder.internal.prefer_const(needNormalOutput);
    
    if needNormalOutput
        mNormal = varargin{1};
        mNormalEnd = varargin{2};
    end
    
    count = uint32(last-first);
    if count <= maxVoxelPoints
        %------- SAMPLE PARTITION ----------------------------------------
        % If the voxel is smaller than maxVoxelPoints, take a random sample
        % in voxel and store in output.
        neighborIndices = coder.nullcopy(zeros([count 1], 'uint32'));
        neighborIndicesEnd = 1;
        idx = coder.nullcopy(zeros(1, 'uint32'));
        for i = first:last-1
            idx = indexVector(i);
            if ~isfinite(location(idx)) || ~isfinite(location(idx+numPoints)) || ~isfinite(location(idx+numPoints*2))
                continue;
            end
            neighborIndices(neighborIndicesEnd) = idx;
            neighborIndicesEnd = neighborIndicesEnd + 1;
        end
        if ((~needNormalOutput && neighborIndicesEnd == 1) || (needNormalOutput && neighborIndicesEnd < 4))
            if needNormalOutput
                varargout{1} = mNormal;
                varargout{2} = mNormalEnd;
            end
            return;
        end
        numNeighbors = neighborIndicesEnd - 1;
        randIndex = uint32(ceil(rand * numNeighbors));
        mOutIndices(mOutIndicesEnd) = neighborIndices(randIndex);
        mOutIndicesEnd = mOutIndicesEnd + 1;
        
        if needNormalOutput
            neighbors = coder.nullcopy(zeros([numNeighbors 3], 'like', location));
            neighborsEnd = 1;
            for i = 1:numNeighbors
                neighbors(i) = location(neighborIndices(i));
                neighbors(i+numNeighbors) = location(neighborIndices(i)+numPoints);
                neighbors(i+numNeighbors*2) = location(neighborIndices(i)+numPoints*2);
                neighborsEnd = neighborsEnd + 1;
            end
            [~, normalVector] = vision.internal.codegen.pc.estimateNormalVectorWithPCA(neighbors, numNeighbors);
            mNormal(mNormalEnd) = normalVector(1);
            mNormal(mNormalEnd+1) = normalVector(2);
            mNormal(mNormalEnd+2) = normalVector(3);
            mNormalEnd = mNormalEnd + 3;
        end
        %-----------------------------------------------------------------
        
        if needNormalOutput
            varargout{1} = mNormal;
            varargout{2} = mNormalEnd;
        end
        return
    end
    
    cutDim = uint32(1);
    maxDiff = zeros(1,'like',maxPoint);
    
    for j = 1:3
        diff = maxPoint(j) - minPoint(j);
        if diff > maxDiff
            maxDiff = diff;
            cutDim = uint32(j);
        end
    end
    
    % Use integer division to have same rounding behaviour as simulation.
    rightCount = idivide(count,2);
    leftCount = count - rightCount;
    % select the indexes of first leftCount points in voxel with respect to the cutDim.    
    depthLimit = ceil(log2(double(count)))*2;
    [cutVal,indexVector] = introSelectIdx(depthLimit,location,indexVector,first,last,leftCount+1,cutDim,numPoints);  
    % Partition points in voxel with respect to cutVal.
    leftMaxValues = maxPoint;
    leftMaxValues(cutDim) = cutVal;
    rightMinValues = minPoint;
    rightMinValues(cutDim) = cutVal;
    
    if needNormalOutput
        [indexVector,mOutIndices,mOutIndicesEnd,mNormal,mNormalEnd] = partition(indexVector,numPoints,first,first+leftCount,minPoint,leftMaxValues,location,mOutIndices,mOutIndicesEnd,maxVoxelPoints,mNormal,mNormalEnd);
        [indexVector,mOutIndices,mOutIndicesEnd,mNormal,mNormalEnd] = partition(indexVector,numPoints,first+leftCount,last,rightMinValues,maxPoint,location,mOutIndices,mOutIndicesEnd,maxVoxelPoints,mNormal,mNormalEnd);
        varargout{1} = mNormal;
        varargout{2} = mNormalEnd;
    else
        [indexVector,mOutIndices,mOutIndicesEnd] = partition(indexVector,numPoints,first,first+leftCount,minPoint,leftMaxValues,location,mOutIndices,mOutIndicesEnd,maxVoxelPoints);
        [indexVector,mOutIndices,mOutIndicesEnd] = partition(indexVector,numPoints,first+leftCount,last,rightMinValues,maxPoint,location,mOutIndices,mOutIndicesEnd,maxVoxelPoints);
    end
end

%==========================================================================
% Intro select
%==========================================================================
function [kvalue,indexVector] = introSelectIdx(depthLimit,location,indexVector,first,last,k,dim,numPoints)    
%   This is a hybrid of quickselect and heapselect, which starts with the quickselect and fall back to heapselect if the quickselect does not rapidly enough.
%   It rearranges elements in [first, last) of indexVector such that: 
%       1. location(indexVector(first+k), dim) is the selected kvalue;
%       2. all of the elements in location(indexVector(first: first + k - 1), dim) are less than or equal to kvalue;
%       3. all of the elements in location(indexVector(first + k + 1: last - 1), dim) are great than or equal to kvalue.
%
%   Inputs:
%   -------
%   location     : the unchanged point location, Mx3 or MxNx3 matrix
%   indexVector  : the reordered index vector, Mx1 vector
%   first        : the index of indexVector specifying at which position to start the rearrangement
%   last         : the index of indexVector specifying at which position to stop the rearrangement (not included)
%   k            : the k-th smallest element that needs to be selected as kvalue
%   dim          : the dimension of location to be selected by
%   numPoints    : the number of points in location
%
%   Outputs:
%   -------
%   kvalue       : the k-th smallest element in location(indexVector(first: last - 1), dim) 
%   indexVector  : the rearranged index vector, Nx1
  
  coder.inline('always');
  l = first; 
  r = last-1;
  
  temp = coder.nullcopy(zeros(1,'like',indexVector)); 
  
  while true
      if depthLimit == 0
            [kvalue,indexVector] = heapSelect(location,indexVector,l,r,k,dim,numPoints);
            return;
      else
            depthLimit = depthLimit-1;
            % choose median of three (vl, vm, vr) as pivot
            % this generally improves performance and eliminates worst-case behavior
            % for sorted/reverse-sorted data
            m = fix((r+l) / 2);
            % value corresponding to the index
            vl = location(indexVector(l) + (dim-1)*numPoints);
            vm = location(indexVector(m) + (dim-1)*numPoints);
            vr = location(indexVector(r) + (dim-1)*numPoints);

            if vm < vr
                indexVector = swapIndexVector(indexVector,m,r);
            end

            if vm < vl
                indexVector = swapIndexVector(indexVector,m,l);
            end

            if vr < vl
                indexVector = swapIndexVector(indexVector,l,r);
            end

            pivot = location(indexVector(r) + (dim-1)*numPoints); 
            i = l;
        
            for j = l:r-1
              vj = location(indexVector(j) + (dim-1)*numPoints);
              if(vj <= pivot)
                  indexVector = swapIndexVector(indexVector,i,j);
                  i = i+1;
              end
            end

            indexVector = swapIndexVector(indexVector,i,r);
            m = i-l+1;
            if(k < m)
              r = i-1;
            elseif(k > m)
              l = i+1; 
              k = k-m;
            else
              kvalue = location(indexVector(i) + (dim-1)*numPoints);
              return;
            end
       end
   end
end

%==========================================================================
% Heap select
%==========================================================================
function [kvalue,indexVector] = heapSelect(location,indexVector,l,r,k,dim,numPoints)
    prefix = l-1;
    % Construct the maxHeap for the first k elements.
    for idx = 2:k
        j = idx;
        while (j>1) && (location(indexVector(prefix+idivide(j,2,"fix")) + (dim-1)*numPoints) < location(indexVector(prefix+j) + (dim-1)*numPoints))
            indexVector = swapIndexVector(indexVector,prefix+j,prefix+idivide(j,2,"fix"));            
            j = idivide(j,2,"fix");
        end
    end
    % If find new element smaller than the max element in the maxHeap,
    % remove the max element and add the new element in the maxHeap.
    for idx = k+1:r-l+1
        j = idx;
        if location(indexVector(prefix+j) + (dim-1)*numPoints) < location(indexVector(prefix+1) + (dim-1)*numPoints)
            indexVector = swapIndexVector(indexVector,prefix+j,prefix+1); % remove the max element and add new element
            indexVector = maxHeapifyIndexVector(location,indexVector, 1, k, prefix,dim,numPoints);
        end
    end
    kvalue = location(indexVector(prefix+1) + (dim-1)*numPoints);
end

%==========================================================================
% Max Heapify
%==========================================================================
function indexVector = maxHeapifyIndexVector(location,indexVector,i,size,prefix,dim,numPoints)
    left = 2*i; 
    right = 2*i+1; 
    vI = location(indexVector(prefix+i) + (dim-1)*numPoints);
    vLeft = location(indexVector(prefix+left) + (dim-1)*numPoints);
    vRight = location(indexVector(prefix+right) + (dim-1)*numPoints);
    if (left <= size) && (vLeft > vI)
        largest = left;
        vLargest = vLeft;
    else
        largest = i;
        vLargest = vI;
    end
    
    if (right <= size) && (vRight > vLargest)
        largest = right;        
    end

    if (largest ~= i)
        indexVector = swapIndexVector(indexVector,prefix+i,prefix+largest);        
        indexVector = maxHeapifyIndexVector(location,indexVector,largest,size,prefix,dim,numPoints);
    end
end

%==========================================================================
% Swap index in index vector
%==========================================================================
function indexVector = swapIndexVector(indexVector,l,r)
    temp = indexVector(r);
    indexVector(r) = indexVector(l); 
    indexVector(l) = temp; 
end
