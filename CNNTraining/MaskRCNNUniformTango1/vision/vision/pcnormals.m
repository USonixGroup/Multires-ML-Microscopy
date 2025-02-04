function normals = pcnormals(ptCloud, varargin)

% Copyright 2015-2023 The MathWorks, Inc.

%#codegen

validateattributes(ptCloud, {'pointCloud'}, {'scalar'}, mfilename, 'ptCloud');

if(isSimMode())
    parser = inputParser;
    parser.CaseSensitive = false;
    
    parser.addOptional('K', 6, @(x)validateattributes(x, {'single', 'double'}, ...
        {'real', 'nonsparse', 'scalar', 'nonnan', 'finite', 'integer', '>=', 3}));
    parser.parse(varargin{:});
    
    K = parser.Results.K;
else
    K = 6;
    if( length(varargin) >=1)
        validateattributes(varargin{1}, {'single', 'double'}, ...
            {'real', 'nonsparse', 'scalar', 'nonnan', 'finite', 'integer', '>=', 3})
        K = varargin{1};
    end
end
% Find normal vectors for each point
normals = surfaceNormalImpl(ptCloud, K);

function flag = isSimMode()
flag = isempty(coder.target);
