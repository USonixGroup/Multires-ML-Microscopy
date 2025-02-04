function drawCoordinateAxes(detector, hAxes, p)
% Draw origin marker

%   Copyright 2014-2023 The MathWorks, Inc.

colorOrange500 = uint8([255, 165, 0]);

if isa(detector, 'vision.calibration.stereo.CheckerboardDetector') || ... 
    isa(detector, 'vision.calibration.monocular.CheckerboardDetector')
   
    % Checkerboard
    
    % Plot origin.
    plot(hAxes, p(1,1),p(1,2), 'MarkerEdgeColor', colorOrange500, ...
        'Marker', 'square', 'LineStyle', 'none', ...
        'LineWidth', 1.5,'MarkerSize', 10);
    
    numBoardRows = detector.BoardSize(1)-1;
    numBoardCols = detector.BoardSize(2)-1;
    [alignmentOrigin, alignmentX, alignmentY] = ...
        getLabelAlignment(numBoardRows, numBoardCols, p);
elseif isa(detector, 'vision.internal.calibration.webTool.CircleGridDetectorImpl')
    % Circle grid patterns
    
    % Plot origin.
    plot(hAxes, p(1,1),p(1,2), 'MarkerEdgeColor', colorOrange500, ...
        'Marker', 'x', 'LineStyle', 'none', ...
        'LineWidth', 2,'MarkerSize', 10);
    
    numBoardRows = detector.PatternDims(1);
    numBoardCols = detector.PatternDims(2);
    [alignmentOrigin, alignmentX, alignmentY] = ...
        getLabelAlignment(numBoardRows, numBoardCols, p);
else
    % Custom patterns
    
    % Plot origin.
    plot(hAxes, p(1,1),p(1,2), 'MarkerEdgeColor', colorOrange500, ...
        'Marker', 'x', 'LineStyle', 'none', ...
        'LineWidth', 2,'MarkerSize', 10);

    alignmentOrigin = 'middle';
    alignmentX      = 'middle';
    alignmentY      = 'middle';
end

try
    [origin, X, Y] = drawImageAxesLabels(detector, p);
    [textOrigin, textX, textY] = getLabelText;
    color = 'black';
    
    % Display axes labels.
    % Origin
    if all(ismember({'Orientation','Location'},fieldnames(origin)))
        if ~isempty(origin.Orientation) && ~isempty(origin.Location)
            [angleOrigin, textLabelOrigin] = findIdealLabelOrientation(origin.Orientation, ...
                textOrigin);
            displayAxesLabel(textLabelOrigin, origin.Location, angleOrigin, alignmentOrigin, color);   % Origin
        end
    end
    
    % X-axis
    if all(ismember({'Orientation','Location'},fieldnames(X)))
        if ~isempty(X.Orientation) && ~isempty(X.Location)
            [angleX, textLabelX] = findIdealLabelOrientation(X.Orientation, ...
                textX);
            displayAxesLabel(textLabelX, X.Location, angleX, alignmentX, color);
        end
    end
    
    % Y-axis
    if all(ismember({'Orientation','Location'},fieldnames(Y)))
        if ~isempty(Y.Orientation) && ~isempty(Y.Location)
            [angleY, textLabelY] = findIdealLabelOrientation(Y.Orientation, ...
                textY);
            displayAxesLabel(textLabelY, Y.Location, angleY, alignmentY, color);
        end
    end
catch
    % No op. If the function call to the "drawImageAxesLabels" fails, the
    % origin and axes labels are not rendered.
end

    %----------------------------------------------------------------------
    function displayAxesLabel(label, loc, theta, alignment, labelColor)
        text(loc(1), loc(2), label, 'Parent', hAxes, 'Color', labelColor,...
            'FontUnits', 'normalized', 'FontSize', 0.05,...
            'Rotation', theta, ...
            'BackgroundColor', 'white',...
            'EdgeColor', 'black',...
            'VerticalAlignment', alignment, 'Clipping', 'on');
    end

    %----------------------------------------------------------------------
    function [textOrigin, textX, textY] = getLabelText
        % The labels have to be placed such that it can be read easily
        % without tilting one's head too much. For this reason, there are 4
        % configurations used.
    
        numConfigs = 4;
        textOrigin = cell(numConfigs,1);
        textX      = cell(numConfigs,1);
        textY      = cell(numConfigs,1);
    
        % Config1 - straight text.
        % (0,0)  X--->
        %
        %   |
        %   | Y
        %   v
        textOrigin{1} = '(0,0)';
        textX{1} = 'X\rightarrow';
        textY{1} = '\downarrowY';
    
        % Config2 - verticalText (straight text rotated by 90 degrees).
        %   ^
        %   | X
        %   |
        %
        % (0,0)  Y--->
        textOrigin{2} = '(0,0)';
        textX{2} = '\uparrowX';
        textY{2} = 'Y\rightarrow';
    
        % Config3 - invertedText (straight text rotated by 180 degrees).
        %           ^
        %           | Y
        %           |
        %
        %  <---X  (0,0)
        textOrigin{3} = '(0,0)';
        textX{3} = '\leftarrowX';
        textY{3} = '\uparrowY';
    
        % Config4 - inverted vertical text (vertical text rotated by 180 degrees).
        %  <---Y  (0,0)
        %           
        %           | X
        %           |
        %           v
        textOrigin{4} = '(0,0)';
        textX{4} = '\downarrowX';
        textY{4} = '\leftarrowY';
    end
    %----------------------------------------------------------------------
    function [idealAngle, idealText] = findIdealLabelOrientation(angle, angledText)

        angle = wrapAngleTo360Degrees(angle);
        isbetween = @(x, lb, ub)  x < ub && x >= lb; % returns true if x belongs [lb, ub).
        
        if isbetween(angle, 0, 45) || isbetween(angle, 315, 360) 
            % Config1 - straight text.
            idealAngle = angle;
            idealText = angledText{1};
        elseif isbetween(angle, 45, 135) 
            % Config2 - vertical text.
            idealAngle = angle-90;
            idealText = angledText{2};
        elseif isbetween(angle, 135, 225)
            % Config3 - inverted text.
            idealAngle = angle-180;
            idealText = angledText{3};
        else
            % Config4 - inverted vertical text.
            idealAngle = angle-270;
            idealText = angledText{4}; 
        end

        function angle = wrapAngleTo360Degrees(angle)
            % Wrap the angle to [0,360] degrees.
            angle = rem(angle, 360);
            if angle < 0
                angle = angle + 360;
            end
        end
    end
end

%--------------------------------------------------------------------------
function [alignmentOrigin, alignmentX, alignmentY] = ...
                getLabelAlignment(numBoardRows, numBoardCols, p)
    
    % Reshape checkerboard corners to boardSize shaped array.
    boardCoordsY = reshape(p(:,2), [numBoardRows, numBoardCols]);
    
    % Determine alignment for the text labels.
    signChanges = sign(diff(boardCoordsY(:,1)));
    isOriginOnTop = nnz(signChanges > 0) < nnz(signChanges < 0);
    if isOriginOnTop
        alignmentOrigin = 'top';
        alignmentX      = 'top';
        alignmentY      = 'bottom';
    else
        alignmentOrigin = 'bottom';
        alignmentX      = 'bottom';
        alignmentY      = 'top';
    end
end