function updatePrintAxes( this,inputFig )
%UPDATEPRINTAXES Makes Point Cloud visual specific updates for Printing
%   Updates the axis limits for the zoom feature in Point Cloud visuals. Resets 
%   the position of print axes and print figure.

%   Copyright 2022-2023 The MathWorks, Inc.


% If the data source is empty there is no Point Cloud in the scope. The scope
% could be blank or with a text message.
if ~isempty(this.DataType) && ~this.Application.screenMsg
    
    % Get the default print axes set on the print to figure
    printAxes = get(inputFig,'CurrentAxes');

    oldPosition = get(inputFig ,'Position');

    figPosition  = oldPosition;

    set(inputFig ,'Position',figPosition);

    % Reset some properties (axes limits and position) 
    set(printAxes, 'Units', 'normalized', ...
        'Position', [0 0 1 1], ...
        'XLim', this.XLimit,...
        'YLim', this.YLimit);
end
