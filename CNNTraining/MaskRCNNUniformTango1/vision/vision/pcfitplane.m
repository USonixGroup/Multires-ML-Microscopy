function [model, inlierIndices, outlierIndices, meanError] = pcfitplane(varargin)

% Copyright 2015-2023 The MathWorks, Inc.

%#codegen

narginchk(2, 10);

[ptCloud, ransacParams, sampleIndices, ...
    referenceVector, maxAngularDistance] = ...
    vision.internal.ransac.validateAndParseRansacInputs(mfilename, true, varargin{:});

% Use three points to fit a plane
sampleSize = 3;
ransacParams.sampleSize = sampleSize;

% For Simulation and ML Coder.
if ~isGPUTarget()
    % Initialization
    [statusCode, status, pc, validPtCloudIndices] = ...
        vision.internal.ransac.initializeRansacModel(ptCloud, sampleIndices, sampleSize);
    
    modelParams = zeros(1, 4, 'like', pc.Location);
    distances = cast([], 'like', pc.Location);
   
    % Compute the geometric model parameter with MSAC
    if status == statusCode.NoError
        
        if isempty(referenceVector)
       
            ransacFuncs.fitFunc = @fitPlane;
            ransacFuncs.evalFunc = @evalPlane;
            ransacFuncs.checkFunc = @checkPlane;
            
            [isFound, modelParams] = vision.internal.ransac.msac(pc.Location, ...
                ransacParams, ransacFuncs);
            
        else
            % Fit the plane with orientation constraint
            
            % In codegen, type mismatch: function_handle checkPlane ~=
            % checkPerpendicularPlane, and so a different variable
            % 'ransacFuncsPerpendicularPlane' is used.
            ransacFuncsPerpendicularPlane.fitFunc = @fitPlane;
            ransacFuncsPerpendicularPlane.evalFunc = @evalPlane;
            ransacFuncsPerpendicularPlane.checkFunc = @checkPerpendicularPlane;
            
            denorm = sqrt(dot(referenceVector, referenceVector));
            normAxis = referenceVector./repmat(denorm, size(referenceVector));

            [isFound, modelParams] = vision.internal.ransac.msac(pc.Location, ...
                ransacParams, ransacFuncsPerpendicularPlane, normAxis, maxAngularDistance);
               
            if isFound
                % Adjust the plane normal vector so that its direction matches the
                % referenceVector. This makes the absolute angular distance always
                % smaller than 90 degrees.
                a = min(1, max(-1, dot(normAxis, modelParams(1:3))));
                angle = abs(acos(a));
                if angle > pi/2
                    modelParams = -modelParams;
                end
            end
        end
        if ~isFound
            status = statusCode.NotEnoughInliers;
        end
    end
    
    if(isempty(modelParams))
        modelParams = zeros(1, 4, 'like', pc.Location);
    end
    
    % Construct the plane object
    model = planeModel(modelParams);
    
    % Report runtime error
    vision.internal.ransac.checkRansacRuntimeStatus(statusCode, status);
    
    % Extract inliers
    needInlierIndices = (nargout > 1);
    needOutlierIndices = (nargout > 2);
    needMeanError = (nargout > 3);
    
    if needInlierIndices
        if status == statusCode.NoError
            % Re-evaluate the best model
            if ~isempty(sampleIndices)
                [pc, validPtCloudIndices] = removeInvalidPoints(ptCloud);
            end
            distances = evalPlane(modelParams, pc.Location);
            inlierIndices = validPtCloudIndices(distances < ransacParams.maxDistance);
        else
            inlierIndices = [];
        end
    end
    % Extract outliers
    if needOutlierIndices
        if status == statusCode.NoError
            flag = true(ptCloud.Count, 1);
            flag(inlierIndices) = false;
            outlierIndices = find(flag);
        else
            outlierIndices = [];
        end
    end
    
    % Report MeanError
    if needMeanError
        if status == statusCode.NoError
            meanError = mean(distances(distances < ransacParams.maxDistance));
        else
            meanError = cast([], 'like', pc.Location);
        end
    end
else
    % GPU specific implementation of pcfitplane.
    [model, inlierIndices, outlierIndices, meanError] = ...
        vision.internal.codegen.gpu.pcfitplane.pcfitplaneGpuImpl(ptCloud, ...
        ransacParams, sampleIndices, referenceVector, maxAngularDistance);
end

%==========================================================================
% Plane equation: ax + by + cz + d = 0;
%==========================================================================
function model = fitPlane(points, varargin)
a = points(2, :) - points(1, :);
b = points(3, :) - points(1, :);
% Cross product
normal = [a(2).*b(3)-a(3).*b(2), ...
    a(3).*b(1)-a(1).*b(3), ...
    a(1).*b(2)-a(2).*b(1)];
denom = sum(normal.^2);
if denom < eps(class(points))
    model = cast([],'like', points);
else
    normal = normal / sqrt(denom);
    d = -points(1,:) * normal';
    model = cast([normal, d] ,'like', points);
end

%==========================================================================
% Calculate the distance from the point to the plane.
% D = (a*x + b*y + c*z + d)/sqrt(a^2+b^2+c^2). Denominator is 1 because
% the normal is a unit vector here.
%==========================================================================
function dis = evalPlane(model, points, varargin)
dis = abs(points * model(1:3)' + model(4));

%==========================================================================
% Validate the plane coefficients
%==========================================================================
function isValid = checkPlane(model)
isValid = (numel(model) == 4 & all(isfinite(model(:))));

%==========================================================================
% Validate the plane coefficients with orientation constraints
%==========================================================================
function isValid = checkPerpendicularPlane(model, normAxis, threshold)
isValid = checkPlane(model);
if isValid
    a = min(1, max(-1, (normAxis(1)*model(1) + normAxis(2)*model(2) + normAxis(3)*model(3)) ));
    angle = abs(acos(a));
    angle = min(angle, pi-angle);
    isValid = (angle < threshold);
end

% =========================================================================
% GPU codegen support flag
% =========================================================================
function flag = isGPUTarget()
flag = coder.gpu.internal.isGpuEnabled;


