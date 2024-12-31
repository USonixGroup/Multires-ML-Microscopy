function update(this)
%UPDATE   Update the Point Cloud viewer display.

%   Copyright 2022 The MathWorks, Inc.

% Cache the new frame data
source = this.Application.DataSource;

inputValues = source.getRawData;
if size(inputValues{1},2) < 2
    return;
end
    
view(this.PcplayerObj,inputValues{:});

this.XData = this.Primitive.XData;
this.YData = this.Primitive.YData;
this.ZData = this.Primitive.ZData;
this.CData = this.Primitive.CData;
    
end

% [EOF]
