function tf = hasHardwareRendering(hFig)
% Check if rendering has hardware support
info = rendererinfo(axes(hFig));
tf = ~strcmp(info.Details.HardwareSupportLevel, 'None');
end