function P = leastSquaresPolynomialFit(pts, N) %#codegen
% leastSquaresPolynomialFit compute least squares fit
%  p = leastSquaresPolynomialFit(points, N) computes the least squares fit
%  of a Nth degree polynomial to the M-by-2 coordinates in points.
%

% Copyright 2016 The MathWorks, Inc.

% Code trimmed from polyfit.m

x = pts(:,1);
y = pts(:,2);

% Construct Vandermonde matrix.
V = coder.nullcopy(zeros(length(x), N+1, class(x)));
V(:,N+1) = ones(length(x),1,class(x));
for j = N:-1:1
    V(:,j) = x.*V(:,j+1);
end

% Solve least squares problem.
[Q,R] = qr(V,0);

if isempty(coder.target)
    ws = warning('off','all');
end

P = R\(Q'*y); % Same as p = V\y;

if isempty(coder.target)
    warning(ws);
end

% Polynomial coefficients are row vectors by convention.
P = P.';
end

