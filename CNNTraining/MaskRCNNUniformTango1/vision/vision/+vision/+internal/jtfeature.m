function flag = jtfeature(featureName, varargin)
%

%   Copyright 2020 The MathWorks, Inc.

%jtfeature Undocumented function for internal use

%jtfeature Get and set feature control value for AppContainer
% infrastructure of the CVT Apps
% Currently supported feature control keyword:
% useAppContainer (case insensitive)
%
% jtfeature('useAppContainer') returns the current feature value
% jtfeature('useAppContainer', value) sets the value for
% 'useAppContainer' and returns the new value. value can be true or
% false
%
% The function uses mlock to lock the function in memory once it is invoked.
% Locking a function prevents clear from removing it from memory, and
% prevents reinitialization of any persistent variables defined in the
% file.

mlock
persistent featureVal_useAppCont

if isempty(featureVal_useAppCont)
    featureVal_useAppCont = true; % feature is true by default
end

narginchk(1,2);
validatestring(lower(featureName), {'useappcontainer'}, ...
    mfilename, 'Feature Name');

if nargin==2    
    val = varargin{1};
    validateattributes(val, {'logical'}, {'scalar','nonempty'}, mfilename, 'Feature Value');
end

switch lower(featureName)
    case 'useappcontainer'
        if nargin==2 
            featureVal_useAppCont = val;
        end

        flag = featureVal_useAppCont;
    otherwise
        error('Unsupported feature control keyword');
end

end
