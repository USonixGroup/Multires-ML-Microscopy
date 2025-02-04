function postUpdate(this)
%POSTUPDATE

%   Copyright 2022 The MathWorks, Inc.
      
hScope = this.Application;
            
sendEvent(hScope, 'VisualUpdated');
end

% [EOF] 
