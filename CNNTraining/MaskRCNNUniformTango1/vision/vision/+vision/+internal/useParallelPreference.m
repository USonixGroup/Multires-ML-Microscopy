function useParallel = useParallelPreference()
% Returns true if the UseParallel preference is enabled, otherwise returns
% false.

%   Copyright 2014-2020 The MathWorks, Inc.

% Call upon settings API object.
Settings = settings;
% Get the current value of the preference.
pref = Settings.vision.parallelsupport.UseParallel.ActiveValue;
useParallel = logical(pref);
