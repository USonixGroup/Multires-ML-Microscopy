classdef MSERRegions

    % Copyright 2011-2023 The MathWorks, Inc.
    
    properties (SetAccess='private', GetAccess='public', Dependent = true)
        Count;
        Location;
    end
    
    properties (SetAccess='private', GetAccess='public', Hidden=true)
        Centroid = ones(0,2,'single');
    end
    
    properties (SetAccess='private', GetAccess='public')
        Axes = ones(0,2,'single');
        Orientation = ones(0,1,'single');
    end
    
    properties (Access='public', Dependent = true)
        PixelList;
    end
    
    % Internal properties that are accessible only indirectly through
    % dependent properties
    properties (Access='private')
        pPixelList = cell(0,1);
    end
    
    methods % Accessors for Dependent properties
        %-------------------------------------------------
        function this = set.PixelList(this, in)
            checkPixelList(in);
            this.pPixelList  = in;
            pixelListLen     = size(this.pPixelList,1);
            
            this.Centroid    = single(zeros(pixelListLen,2));
            this.Axes        = single(zeros(pixelListLen,2));
            this.Orientation = single(zeros(pixelListLen,1));
            
            for idx = 1:pixelListLen
                ellipseStruct           = computeEllipseProps(this.pPixelList{idx});
                this.Centroid(idx,:)    = single(ellipseStruct.Centroid);
                this.Axes(idx,:)        = single(ellipseStruct.Axes);
                this.Orientation(idx,1) = single(ellipseStruct.Orientation);
            end
        end
        function out = get.PixelList(this)
            out = this.pPixelList;
        end
        %-------------------------------------------------
        function out = get.Count(this)
            out = size(this.pPixelList,1);
        end
        %-----------------------------------------------
        function out = get.Location(this)
            out = this.Centroid;
        end
        %-----------------------------------------------
        function out = get.Centroid(this)
            out = this.Centroid;
        end
        %------------------------------------------------
        function out = get.Axes(this)
            out = this.Axes;
        end
        %-------------------------------------------------
        function out = get.Orientation(this)
            out = this.Orientation;
        end
    end
    
    methods(Access=public, Static, Hidden)
        function name = matlabCodegenRedirect(~)
            name = 'vision.internal.MSERRegions_cg';
        end
    end
   
    %-----------------------------------------------------------------------
    methods (Access='public')
        function this = MSERRegions(varargin)
            if nargin >= 1
                inputs         = parseInputs(varargin{:});
                this.PixelList = inputs.PixelList;
            end
        end
           
        %------------------------------------------------------------------
        % Note:  NUMEL is not overridden because it interferes with the
        %        desired operation of this object.  MSERRegions is a scalar
        %        object which pretends to be a vector. NUMEL is used during
        %        subsref operations and therefore needs to represent true
        %        number of elements for the object, which is always 1.
        %------------------------------------------------------------------
        function out = length(this)
            out = this.Count;
        end
        
        %------------------------------------------------------------------
        function out = isempty(this)
            out = this.Count == 0;
        end
        
        %------------------------------------------------------------------
        function varargout = size(this, varargin)  
            try
                % size(obj, 1, 2, 3, ...) -> error
                % size(obj, vector)       -> error
                narginchk(1,2);
                if ~isempty(varargin)
                    validateattributes(varargin{1},{'numeric'},...
                        {'positive','scalar','real','finite','nonsparse'},...
                        'size');
                end
                
                % Use the builtin function to validate the inputs and
                % outputs.
                switch nargout
                    case 0
                        % size(obj)       :  ans = [this.Count 1]
                        % size(obj, 1)    :  ans = this.Count
                        % size(obj, 2)    :  ans = 1
                        % size(obj, d > 2):  ans = 1
                        [varargout{1:nargout}] = ...
                            builtin('size', this, varargin{:});
                        if isempty(varargin)
                            % size(obj)
                            varargout{1}(1) = this.Count;
                        elseif numel(varargin) == 1 && varargin{1} ~= 1
                            % size(obj, 2), size(obj,n) n~=1 = 1
                            varargout{1} = 1;
                        else
                            % size(obj, 1)
                            varargout{1} = this.Count;
                        end
                        
                    case 1
                        % D = size(obj)       :  D = [this.Count, 1]
                        % n = size(obj, 1)    :  n = this.Count
                        % m = size(obj, 2)    :  m = 1
                        % p = size(obj, d > 2):  p = 1
                        n = builtin('size', this, varargin{:});
                        if isempty(varargin)
                            % D = size(obj);
                            varargout{1} = [this.Count, 1];
                        elseif numel(varargin) == 1 && varargin{1} ~= 1
                            % m = size(obj, 2);
                            % p = size(obj, d > 3);
                            varargout{1} = n;
                        else
                            % n = size(obj, 1);
                            varargout{1} = this.Count;
                        end
                        
                    case 2
                        % [n, m] = size(obj);
                        % [n, m] = size(obj, d) --> issues error
                        [n, ~] = builtin('size', this, varargin{:});
                        varargout{1} = this.Count;
                        varargout{2} = n;
                        
                    otherwise
                        % [n, m, p, ...] = size(obj)
                        % [n, m, p, ...] = size(obj, d) ---> issues error
                        %  p, ... are always 1
                        [n, ~, varargout{3:nargout}] = ...
                            builtin('size', this, varargin{:});
                        varargout{1} = this.Count;
                        varargout{2} = n;
                end
            catch e
                % throwAsCaller(e) in order to prevent the line:
                % Error using MSERRegions/size. Issue only
                % the error message.
                throwAsCaller(e);
            end
        end
        %-------------------------------------------------------------------
        function varargout = plot(this, varargin)
            
            nargoutchk(0,1);
            
            [ax, inputs] = parsePlotInputs(varargin{:});
            
            ax = newplot(ax);
            
            if ~inputs.showEllipses && ~inputs.showOrientation ...
                    && ~inputs.showPixelList
                % invoke basic points plot
                plot(ax, this.Centroid(:,1), this.Centroid(:,2),'g+');
            end
            
            if inputs.showPixelList
                % plot regions only
                wasHeld = ishold(ax);
                               
                if (this.Count)
                    rgbVals = label2rgb(1:this.Count,'jet',[1 1 1],'shuffle');
                end
                
                for k = 1:length(this.pPixelList)
                    % plot larger regions first
                    [~, idx] = sort(...
                        cellfun(@length, this.pPixelList),'descend');
                    pt = this.pPixelList{idx(k)};
                    regionColor = squeeze(rgbVals(1,idx(k),:))';
                    regionColor = double(regionColor)/255;
                    
                    plot(ax, pt(:,1), pt(:,2), 'Marker', '.',...
                        'LineStyle', 'none', 'Color', regionColor);
                    
                    % turn the hold state on so that all points are
                    % plotted; this is necessary since we are plotting in
                    % a loop, otherwise each plot command will overwrite 
                    % the previous result
                    if k==1 && ~wasHeld
                        hold(ax, 'on');
                    end
                end
                
                if ~wasHeld
                    hold(ax, 'off'); % restore original states of hold
                end
            end
            
            if inputs.showEllipses
                phi = linspace(0,2*pi);
                x   = cos(phi);
                y   = sin(phi);
                
                if inputs.showOrientation % Plot orientation
                    % the two zeros result in a horizontal line which
                    % will be rotated at a later stage
                    unitCircle = [x 0; y 0];
                else
                    unitCircle = [x; y];
                end
                
                wasHeld = ishold(ax);
                
                for k = 1:this.Count
                    % Update scale using ellipse Axes
                    scale  = [this.Axes(k,1)/2 0; 0 this.Axes(k,2)/2]; 
                    pt     = this.Centroid(k,:)';
                    orient = this.Orientation(k);
                    
                    rotationMat = [cos(orient) sin(orient);...
                        -sin(orient) cos(orient)];
                    
                    mserEllipse = rotationMat*scale*unitCircle + ...
                        pt*ones(1,size(unitCircle,2));
                    
                    plot(ax, pt(1), pt(2), 'g+'); % + marking center
                    % turn the hold state on so that all points are
                    % plotted; this is necessary since we are plotting in
                    % a loop, otherwise each plot command will overwrite 
                    % the previous result
                    if k==1 && ~wasHeld
                        hold(ax, 'on');
                    end
                    plot(ax, mserEllipse(1,:),mserEllipse(2,:),'g');
                    
                end
                
                if ~wasHeld
                    hold(ax, 'off'); % restore original states of hold
                end
            end
            
            if nargout == 1
                varargout{1} = ax;
            end
            
        end
        
        %-------------------------------------------------------------------
        function ind = end(this,varargin)

            if isempty(varargin) || varargin{1} == 1
                ind = this.Count;
            else
                ind = 1;
            end
        end
        
        %-------------------------------------------------------------------
        function varargout = subsref(this,s)
            try
                switch s(1).type
                    case '()'
                        nargoutchk(0,1);
                        % Centroid and Axes are Mx2 matrices while
                        % Orientation and PixelList are Mx1 matrices. When
                        % the indices for sub-referencing is a 1-D array,
                        % we explicitly specify the size for the second
                        % dimension.
                        opt1 = s(1);
                        opt2 = s(1);
                        if length(s(1).subs) == 1
                            opt1.subs{2} = 1;
                            opt2.subs{2} = 1:2;
                        end
                        
                        this.Orientation = subsref(this.Orientation,opt1);
                        this.pPixelList  = subsref(this.pPixelList,opt1);
                        this.Centroid    = subsref(this.Centroid,opt2);
                        this.Axes        = subsref(this.Axes,opt2);
                        
                        % invocation of plot or disp would result in setting
                        % isDotMethod
                        if numel(s) >= 2
                            isDotMethod = any(strcmp(s(2).subs, {'plot','disp'}));
                        else
                            isDotMethod = false;
                        end
                                                
                        % protect against indexing that would affect integrity
                        % of the object
                        if (~isDotMethod)
                            if ~(size(this.pPixelList,2)   == 1 && ...
                                    ismatrix(this.pPixelList) && ...
                                    numel(s) <= 2)                                    
                                error(message('vision:MSERRegions:invalidIndexingOperation'));
                            end
                        end
                        
                        if numel(s) <=1
                            varargout{1} = this;
                        else
                            % don't set 'ans' for disp() and plot()
                            if  isDotMethod && nargout ==  0
                                builtin('subsref',this,s(2));
                            else
                                outSubs = builtin('subsref',this,s(2));
                                
                                if iscell(outSubs) && (length(outSubs) == 1)
                                    % Extract the contents of the cell array if the
                                    % values are all numerical string.
                                    varargout{1} = cell2mat(outSubs);
                                else
                                    varargout{1} = outSubs;
                                end
                            end
                        end
                    case '{}'
                        % use of {} indexing is not supported by MSERRegions;
                        % let the builtin function error out as appropriate
                        builtin('subsref',this,s);
                    case '.'
                        % don't set 'ans' for disp() and plot()
                        if  strcmp(s(1).subs, 'disp') || ...
                                (strcmp(s(1).subs, 'plot') && nargout ==  0)
                            builtin('subsref',this,s);
                        else
                            outSubs = builtin('subsref',this,s);
                            if iscell(outSubs) && (length(outSubs) == 1)
                                % Extract the contents of the cell array if the
                                % values are all numerical string.
                                varargout{1} = cell2mat(outSubs);
                            else
                                varargout{1} = outSubs;
                            end
                        end
                end
            catch e
                throwAsCaller(e);
            end
        end
        
        
        %-------------------------------------------------------------------
        function out = subsasgn(this,s,in)
            try
                switch s(1).type
                    case '()'
                        if numel(s) == 2
                            % Centroid, Axes and Orientation are dependent
                            % properties of PixelList. They cannot be modified.
                            if ~(strcmp(s(2).subs,'PixelList'))
                                error(message('vision:MSERRegions:cannotSetDependentProps'));
                            end
                            this.(s(2).subs) = ...
                                subsasgn(this.(s(2).subs), s(1), in);
                        else
                            if ~((isa(in,'MSERRegions') && ~isempty(in)) ||...
                                    (isa(in,'double') && isempty(in)))
                                error(message('vision:MSERRegions:badAssignmentInput'));
                            end
                            if isempty(in)
                                this.PixelList = subsasgn(this.PixelList,s,in);
                            else
                                this.PixelList = ...
                                    subsasgn(this.PixelList,s,in.PixelList);
                            end
                        end
                        out = this;
                    case {'{}', '.'}
                        if ~(strcmp(s(1).subs,'PixelList'))
                            error(message('vision:MSERRegions:cannotSetDependentProps'));
                        end
                        
                        out = builtin('subsasgn',this,s,in);
                end
            catch e
                throwAsCaller(e);
            end
        end
        
        %-------------------------------------------------------------------
        % All of the methods below have to be managed because they can
        % create non-scalar arrays of objects and MSERRegions is strictly
        % a scalar object.
        %-------------------------------------------------------------------
        function out = cat(~, varargin) %#ok<STOUT>
            error(message('vision:MSERRegions:noCatAllowed'));
        end
        %
        function out = horzcat(varargin) %#ok<STOUT>
            error(message('vision:MSERRegions:noCatAllowed'));
        end
        %
        function out = vertcat(varargin) %#ok<STOUT>
            error(message('vision:MSERRegions:noCatAllowed'));
        end
        %
        function out = repmat(varargin) %#ok<STOUT>
            error(message('vision:MSERRegions:noCatAllowed'));
        end        
    end
     methods (Hidden)
        function sz = numArgumentsFromSubscript(~, ~, callingContext)
           switch callingContext.char
                case 'Statement'  % this(1:n).prop or this.plot
                    sz = 0;
                case {'Assignment', ... % [this(1:n).prop] = val
                      'Expression'}     % sin(this(1:n).prop)
                    sz = 1;
           end           
        end
    end
end

%--------------------------------------------------------------------------
% Plot input parser
%--------------------------------------------------------------------------
function [h, inputs] = parsePlotInputs(varargin)

% Parse the PV pairs
parser = inputParser;

uiaxesSupported = true;
parser.addOptional('AXIS_HANDLE', [], ...
    @(p)vision.internal.inputValidation.validateAxesHandle(p, uiaxesSupported));

parser.addParameter('showEllipses', true,  @checkFlag);

parser.addParameter('showOrientation', false, @checkFlag);

parser.addParameter('showPixelList', false, @checkFlag);

% Parse input
parser.parse(varargin{:});

% Assign return values
h = parser.Results.AXIS_HANDLE;

inputs.showEllipses     = parser.Results.showEllipses;
inputs.showOrientation  = parser.Results.showOrientation;
inputs.showPixelList    = parser.Results.showPixelList;

end

%--------------------------------------------------------------------------
%  Plot checkers
%--------------------------------------------------------------------------
function tf = checkFlag(in)

validateattributes(in, {'logical'}, {'scalar'}, mfilename);

tf = true;
end

%--------------------------------------------------------------------------
% Main parser for the class
%--------------------------------------------------------------------------
function [inputs, unused] = parseInputs(varargin)

% Parse the PV pairs
parser = inputParser;

parser.addOptional('PixelList', cell(0,1), @checkPixelList);

% Parse input
parser.parse(varargin{:});

% Populate the parameters to pass into OpenCV's ocvExtractMSER()
inputs.PixelList = parser.Results.PixelList;

unused = parser.UsingDefaults;

end

%--------------------------------------------------------------------------
function tf = checkPixelList(in)
validateattributes(in, {'cell'}, {'size',[NaN,1]}, mfilename);

for idx = 1:length(in)
    validateattributes(in{idx},{'int32'}, {'nonnan', 'finite', ...
        'nonsparse', 'finite', 'nonsparse', 'real', 'size',[NaN,2]}, ...
        mfilename);
end

tf = true;
end

%==========================================================================
% Calculate Ellipse parameters
%==========================================================================
function EllipseStruct = computeEllipseProps(region)
%computeEllipseProps  Calculate ellipse properties 
%
%   Find the ellipse that has the same normalized second central moments as 
%   the region. Compute the axes lengths and orientation of the ellipse. 

%   Reference:
%       Haralick and Shapiro, Computer and Robot Vision vol I, 
%       Addison-Wesley 1992, Appendix A.

EllipseStruct.Centroid = mean(region, 1);
EllipseStruct.Axes = [0 0]; %[majorAxis minorAxis]
EllipseStruct.Orientation = 0;

% Assign X and Y variables so that we're measuring orientation
% counterclockwise from the horizontal axis.

xbar = EllipseStruct.Centroid(1);
ybar = EllipseStruct.Centroid(2);

x =   region(:,1) - xbar;
y = -(region(:,2) - ybar); % This is negative for the
% orientation calculation (measured in the
% counter-clockwise direction).

N = length(x);

% Calculate normalized second central moments for the region. 1/12 is
% the normalized second central moment of a pixel with unit length.
uxx = sum(x.^2)/N + 1/12;
uyy = sum(y.^2)/N + 1/12;
uxy = sum(x.*y)/N;

% Calculate major axis length, minor axis length.
common = sqrt((uxx - uyy)^2 + 4*uxy^2);
EllipseStruct.Axes(1) = 2*sqrt(2)*sqrt(uxx + uyy + common);
EllipseStruct.Axes(2) = 2*sqrt(2)*sqrt(uxx + uyy - common);

% Calculate orientation.
if (uyy > uxx)
    num = uyy - uxx + sqrt((uyy - uxx)^2 + 4*uxy^2);
    den = 2*uxy;
else
    num = 2*uxy;
    den = uxx - uyy + sqrt((uxx - uyy)^2 + 4*uxy^2);
end

if (num == 0) && (den == 0)
    EllipseStruct.Orientation = 0;
else
    EllipseStruct.Orientation = atan(num/den);
end

end
% LocalWords:  OpenCV
