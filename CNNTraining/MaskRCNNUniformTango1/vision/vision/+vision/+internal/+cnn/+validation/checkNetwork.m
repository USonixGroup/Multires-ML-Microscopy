function network = checkNetwork(network, name, supportedNetworkClasses, varargin)
% network = checkNetwork(network, name, supportedNetworkClasses) checks whether network of the
% types listed in cls (e.g. SeriesNetwork).
%
% network = checkNetwork(network, name, supportedNetworkClasses, supportedNetworkNames)
% additionally checks whether network is one of the types listed or one of
% the names listed in supportedNetworkNames.

%   Copyright 2016-2020 The MathWorks, Inc.

networkMaybeGivenByName = ~isempty(varargin);

if networkMaybeGivenByName
    % network maybe a name or one of the listed types.
    if ischar(network) || isstring(network)
        validateattributes(network,...
            {'char','string'},{'scalartext','nonempty'},name,'network');
        network = validatestring(network, varargin{1}, name);
    else
        validateattributes(network,supportedNetworkClasses,{},name,'network');
    end
else
    % only listed class types supported.
    validateattributes(network,supportedNetworkClasses,{},name,'network');
end
