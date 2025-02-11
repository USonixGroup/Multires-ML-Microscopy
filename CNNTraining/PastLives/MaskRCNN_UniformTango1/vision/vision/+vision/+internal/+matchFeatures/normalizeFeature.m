function X = normalizeFeature(X)
%normalizeFeature Normalize the columns in X to have unit norm

%   Copyright 2020 The MathWorks, Inc.

%#codegen

Xnorm = sqrt(sum(X.^2, 1));
X = bsxfun(@rdivide, X, Xnorm);

% Set effective zero length vectors to zero
X(:, (Xnorm <= eps(single(1))) ) = 0;