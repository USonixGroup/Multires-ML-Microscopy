function dist = scanContextDistance(descriptor1, descriptor2)

% Copyright 2020-2023 The MathWorks, Inc.

%#codegen

validateattributes(descriptor1, {'single','double'}, {'2d','nonempty','real'}, mfilename, 'descriptor1');
validateattributes(descriptor2, {'single','double'}, {'2d','nonempty','real','size',size(descriptor1)}, mfilename, 'descriptor2');

numSectors = size(descriptor1, 2);

activeSectors1 = ~all(isnan(descriptor1));
activeSectors2 = ~all(isnan(descriptor2));

% Replace NaN's in the descriptor with 0 to make the math below cleaner
descriptor1(isnan(descriptor1)) = 0;
descriptor2(isnan(descriptor2)) = 0;

if isa(descriptor1, 'single') || isa(descriptor2, 'single')
    type = 'single';
else
    type = 'double';
end

cosineSimilarity = zeros(1, numSectors, type);
for s = 1 : numSectors
    % Shift descriptor1 to the right
    descriptor1     = circshift(descriptor1, 1, 2);
    activeSectors1  = circshift(activeSectors1, 1, 2);
    
    % Compute a.b, a.a and b.b
    adotb = sum(descriptor1 .* descriptor2);
    adota = sqrt(sum(descriptor1 .* descriptor1));
    bdotb = sqrt(sum(descriptor2 .* descriptor2));
    
    % Find the number of active sectors for this configuration
    numActiveSectors = nnz(activeSectors1 | activeSectors2);
    
    perColumnCosineSimilarity = adotb ./ (adota .* bdotb);
    
    % Adjust per column similarity for edge cases
    perColumnCosineSimilarity( adota==0 & bdotb==0 )        = 1;
    perColumnCosineSimilarity( xor(adota==0, bdotb==0) )    = -1;
    
    % Normalize by number of active sectors
    cosineSimilarity(s) = sum(perColumnCosineSimilarity) / numActiveSectors;
end

% Compute cosine distance (in range 0-2) and normalize to 0-1
dist = min( 1 - cosineSimilarity ) / 2;

% Rounding may push value outside the range, so clamp it
dist = min(1, max(0, dist));
end