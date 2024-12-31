function [m,rs,cs,hs1] = getBoundingBoxes(chns, thrs, hs, fids, child, shrink, modelHt, ...
                                          modelWd, stride, cascThr, nTreeNodes, nTrees, ...
                                          height1, width1, treeDepth, flag) %#codegen
                                                                                                                                % Detect object in a multi-channeled input.

    % Copyright 2020 The MathWorks, Inc.
    %
    % References
    % ----------
    %   Dollar, Piotr, et al. "Fast feature pyramids for object detection."
    %   Pattern Analysis and Machine Intelligence, IEEE Transactions on 36.8
    %   (2014): 1532-1545.

    cs = [];
    rs = [];
    hs1 = [];
    k = int32(0);
    k0 = int32(0);

    % make loop result constant
    coder.const(treeDepth);
    coder.const(modelWd);
    coder.const(modelHt);
    coder.const(shrink);
    coder.const(fids);
    nFtrs = modelHt / shrink * modelWd / shrink * 10;
    coder.const(nFtrs);

    % compute unified cid and fid array and cast to compile-time constant
    % for faster generated code
    cidsFids = coder.const(calculateCidFid(nFtrs,modelWd,shrink,modelHt,fids));

    strideShrink = int32(stride/shrink);
    fn = 0;
    for c = 0:width1-1
        for r = 0:height1-1
            if flag ~= -1
                fn = fn + 1;
                if ~flag(fn)
                    continue
                end
            end

            h = single(0);

            % get offset of current window
            chns1 = [int32(r)*int32(strideShrink), int32(c)*int32(strideShrink)];
            if treeDepth == 1
                % specialized case for treeDepth = 1
                for t = 0:nTrees-1
                    offset = int32(t * nTreeNodes);
                    k = int32(offset);
                    k0 = int32(0);
                    [k,k0] = getChild(chns, chns1, cidsFids, thrs, offset, k0, k);
                    h = h+hs(k+1);
                    if (h <= cascThr)
                        break;
                    end
                end
            elseif treeDepth == 2
                % specialized case for treeDepth = 2
                for t = 0:nTrees-1
                    offset = int32(t * nTreeNodes);
                    k = int32(offset);
                    k0 = int32(0);
                    [k,k0] = getChild(chns, chns1, cidsFids, thrs, offset, k0, k);
                    [k,k0] = getChild(chns, chns1, cidsFids, thrs, offset, k0, k);
                    h = h+hs(k+1);
                    if (h <= cascThr)
                        break;
                    end
                end
            elseif treeDepth > 2
                % specialized case for treeDepth > 2
                for t = 0:nTrees-1
                    offset = int32(t * nTreeNodes);
                    k = int32(offset);
                    k0 = int32(0);
                    for i = 1:treeDepth
                        [k,k0] = getChild(chns, chns1, cidsFids, thrs, offset, k0, k);
                    end
                    h = h+hs(k+1);
                    if (h <= cascThr)
                        break;
                    end
                end
            else
                % general case for variable tree depth
                for t = 0:nTrees-1
                    offset = int32(t * nTreeNodes);
                    k = int32(offset);
                    k0 = int32(0);

                    while child(k+1)
                        x = cidsFids(1,k+1) + chns1(1);
                        y = cidsFids(2,k+1) + chns1(2);
                        z = cidsFids(3,k+1);
                        ftr = chns(x,y,z);
                        if ftr < thrs(k+1)
                            k = int32(1);
                        else
                            k = int32(0);
                        end
                        k = int32(child(k0+1))-k+offset;
                        k0 = k;
                    end
                    h = h+hs(k+1);
                    if (h <= cascThr)
                        break;
                    end
                end
            end

            if h > cascThr
                cs = [cs c];
                rs = [rs r];
                hs1 = [hs1 h];
            end
        end
    end

    m = numel(cs);
end

function cidsFids = calculateCidFid(nFtrs,modelWd,shrink,modelHt,fids)
    cids = zeros([nFtrs 3],'uint32');
    m = 1;
    for z = 1:10
        for c = 1:floor(modelWd/shrink)
            for r = 1:floor(modelHt/shrink)
                cids(m,:) = [r c z];
                m = m+1;
            end
        end
    end
    cidsFids = zeros(3,numel(fids),'int32');
    for ki = 0:numel(fids)-1
        idx = fids(ki+1)+1;
        cidsFids(1,ki+1) = cids(idx,1);
        cidsFids(2,ki+1) = cids(idx,2);
        cidsFids(3,ki+1) = cids(idx,3);
    end
end

function [k,k0] = getChild(chns, chns1, cidsFids, thrs, offset, k0i, ki)
    i = cidsFids(1,ki+1)+chns1(1);
    j = cidsFids(2,ki+1)+chns1(2);
    c = cidsFids(3,ki+1);
    ftr = chns(i,j,c);
    k = int32(2);
    if ftr < thrs(ki+1)
        k = int32(1);
    end
    k = k + k0i * 2;
    k0 = k;
    k = k + offset;
end
