function varargout = bbox2poly(bbox)
% [X,Y] = bbox2poly(bbox) returns (x,y) vertices for each bounding box in
% bbox, an M-by-N matrix, where each row defines an axis-aligned bounding
% or a rotated rectangle. Vertices in X and Y are ordered going clock-wise
% from the upper-left of the box (y-axis pointing down).
%
% [X,Y,Z] = bbox2poly(bbox) returns (x,y,z) vertices for each cuboid in
% bbox.
%
% This function is for internal use only and is likely to change in future
% releases.

% Copyright 2019-2021 The MathWorks, Inc.

%#codegen
M = size(bbox,1);
N = size(bbox,2);

% Assigning the outputs
varargout{1} = zeros(coder.ignoreConst(0),M,'like',bbox);
varargout{2} = zeros(coder.ignoreConst(0),M,'like',bbox);
varargout{3} = zeros(coder.ignoreConst(0),M,'like',bbox);

% Output vertices.
X = zeros(4,M,'like',bbox);
Y = zeros(4,M,'like',bbox);

if N == 4
    % axis-aligned rectangle

    % upper-left (xmin,ymin)
    X(1,:) = bbox(:,1);
    Y(1,:) = bbox(:,2);

    % bottom-right (xmax,ymax)
    X(3,:) = bbox(:,3);
    Y(3,:) = bbox(:,4);

    % upper-right
    X(2,:) = X(3,:);
    Y(2,:) = Y(1,:);

    % bottom-left
    X(4,:) = X(1,:);
    Y(4,:) = Y(3,:);

    varargout{1} = X;
    varargout{2} = Y;

elseif N == 5
    % rotated rectangle

    r = bbox(:,5);
    u = [bbox(:,3)/2 bbox(:,3)/2].* [ cosd(r) sind(r)];
    v = [bbox(:,4)/2 bbox(:,4)/2].* [-sind(r) cosd(r)];

    X(1,:) = bbox(:,1) - u(:,1) - v(:,1);
    Y(1,:) = bbox(:,2) - u(:,2) - v(:,2);

    X(2,:) = bbox(:,1) + u(:,1) - v(:,1);
    Y(2,:) = bbox(:,2) + u(:,2) - v(:,2);

    X(3,:) = bbox(:,1) + u(:,1) + v(:,1);
    Y(3,:) = bbox(:,2) + u(:,2) + v(:,2);

    X(4,:) = bbox(:,1) - u(:,1) + v(:,1);
    Y(4,:) = bbox(:,2) - u(:,2) + v(:,2);

    varargout{1} = X;
    varargout{2} = Y;

elseif N == 9
    % Cuboid: [X,Y,Z,W,H,D,ThetaX,ThetaY,ThetaZ].

    if isempty(bbox)
        varargout{1} = zeros(8,M,'like',bbox);
        varargout{2} = zeros(8,M,'like',bbox);
        varargout{3} = zeros(8,M,'like',bbox);
    else
        % Extract Euler angles in ZYX order and convert to radians.
        eul = bbox(:,9:-1:7);
        eul = deg2rad(eul);

        % Convert Euler angles to pre-multiply rotation matrix.
        R = vision.internal.eul.eulerAngleToRotationMatrix(eul,'ZYX');

        % Convert to a post-multiply rotation matrix.
        R = R';

        XYZ = zeros(8,3,M,'like',bbox);

        reshapeFun = @(x) repmat(transpose(x),[4 1]);

        % Build 8x3xM matrices (numVertices x numDims x numBoxes)
        XYZ([1 4 5 8],1,:) = reshapeFun(bbox(:,1)-bbox(:,4)/2);
        XYZ([2 3 6 7],1,:) = reshapeFun(bbox(:,1)+bbox(:,4)/2);
        XYZ([3 4 7 8],2,:) = reshapeFun(bbox(:,2)-bbox(:,5)/2);
        XYZ([1 2 5 6],2,:) = reshapeFun(bbox(:,2)+bbox(:,5)/2);
        XYZ([1 2 3 4],3,:) = reshapeFun(bbox(:,3)-bbox(:,6)/2);
        XYZ([5 6 7 8],3,:) = reshapeFun(bbox(:,3)+bbox(:,6)/2);

        % Shift all XYZ points so that the center is at the origin so we can
        % apply rotation
        offsetToOrigin = reshape(bbox(:,1:3),1,3,[]);
        XYZout = XYZ - repmat(offsetToOrigin, size(XYZ,1), 1, 1);

        % Vertex locations are simply the axis-aligned vertex locations
        % multiplied by each cuboid's rotation matrix.
        if isempty(coder.target)
            XYZout = iBatchMatrixMultiply(XYZout,R);
        else
            for i = 1:M
                XYZout(:,:,i) = XYZout(:,:,i)*R;
            end
        end

        XYZout = XYZout + repmat(offsetToOrigin, size(XYZ,1), 1, 1);

        varargout{1} = squeeze(XYZout(:,1,:));
        varargout{2} = squeeze(XYZout(:,2,:));
        varargout{3} = squeeze(XYZout(:,3,:));
    end
end

end

function C = iBatchMatrixMultiply(A,B)
% yields C(:,:,P) = A(:,:,P)*B(:,:,P) for all P.
C = pagemtimes(A,B);
end