function fileExtension = getFileExtensionFromVideoProfile(videoProfile)
%

%   Copyright 2021 The MathWorks, Inc.
    videoProfile = string(videoProfile);
    profiles = VideoWriter.getProfiles();
    profileNames = string({profiles.Name});
    profileIdx = find(ismember(profileNames,videoProfile));
    assert(~isempty(profileIdx), "Could not find any profile that matches the VideoProfile.");
    fileExtension = string(profiles(profileIdx).FileExtensions);
    fileExtension = fileExtension(1); % Choose the first profile extension.
end
