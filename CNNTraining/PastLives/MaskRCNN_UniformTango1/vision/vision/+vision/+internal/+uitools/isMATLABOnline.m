function out = isMATLABOnline()
%ISMATLABONLINE Check if MATLAB is being run on the web
out = matlab.internal.environment.context.isWebAppServer;
end