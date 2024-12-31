function out = isSSR(hFig)
%ISSSR Check if Server Side Rendering (SSR) is being used
c1 = hFig.getCanvas;
drawnow;
out = c1.ServerSideRendering;  % should be "off"
end