function out = isUsingDirectShowPlugin(videoFileName)
%ISUSINGDIRECTSHOWPLUGIN Check if DirectShow plugin is being used to read
%the video file. This is applicable only on the Windows platform.
out = false;
if ispc
    instance = matlab.internal.video.PluginManager.getInstance;
    pluginPath = lower(instance.getPluginForRead(videoFileName));
    out = contains(pluginPath, "directshow");
end

end