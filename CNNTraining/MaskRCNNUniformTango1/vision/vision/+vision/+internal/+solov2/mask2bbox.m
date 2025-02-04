function bbox = mask2bbox(mask)


[ptsR, ptsC] = find(mask);

minR = min(ptsR);
maxR = max(ptsR);
minC = min(ptsC);
maxC = max(ptsC);

bbox = [minC minR maxC-minC maxR-minR];


