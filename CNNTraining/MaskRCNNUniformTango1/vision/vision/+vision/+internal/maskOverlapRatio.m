function overlapRatio = maskOverlapRatio(maskA, maskB)
%maskOverlapRatio Compute object mask overlap ratio.
%  overlapRatio = maskOverlapRatio(maskA, maskBB) returns the overlap ratio
%  between each pair of masks contained in maskA and maskB.
%  Masks in maskA and maskB are a stack logical masks. maskA is a
%  H-by-W-by-M1 logical array and maskB is a H-by-W-by-M1 logical array. 
%  Each slice in maskA and maskB defines one object mask. By
%  default, the overlap ratio between two masks A and B is defined as
%  area(A intersect B) / area(A union B) also known as the iou or the 
%  jaccard score. The range of overlapRatio is between 0 and 1, where 1 
%  implies a perfect overlap.
%  overlapRatio is a M1-by-M2 matrix, containing the overlap ratio between
%  ever pair of masks in maskA and maskB.

% TODO: Error checks needed here

M1 = size(maskA,3);
M2 = size(maskB,3);

overlapRatio = zeros(M1,M2);

for idxA = 1:M1
    for idxB = 1:M2

        overlapRatio(idxA, idxB) = jaccard(maskA(:,:,idxA), maskB(:,:,idxB));

    end
end