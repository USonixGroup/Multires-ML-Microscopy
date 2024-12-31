% vision.internal.pc.ICPRegistrationGPU GPU implementation of ICP registration

% Copyright 2023-2024 The MathWorks, Inc.

%#codegen
classdef ICPRegistrationGPU < handle

    properties
        Moving              % movingPtCloud
        Fixed               % fixedPtCloud
        CovarianceMoving    % Covariance matrix of movingPtCloud
        CovarianceFixed     % Covariance matrix of fixedPtCloud
    end

    % Hyper parameters
    properties(Access = private)
        Metric
        UseAllMatches
        DoExtrapolate
        InlierRatio
        InlierDistance
        MaxIterations
        Tolerance
        InitialTransform
        Verbose

        UseDegree % flag to support backward compatibility in pcregrigid.
    end

    % Internal variables
    properties(Access = private)
        Rs
        Ts
        qs
        dq
        dTheta
        Err
        LocA
        StopIteration
        MaxNumInliersA
        UseInlierRatio
        InlierIndicesA
        InlierIndicesB
        Printer
    end

    %----------------------------------------------------------------------
    % Public APIs
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function this = ICPRegistrationGPU(unorgMoving, unorgFixed, options)
            this.Moving            = unorgMoving;
            this.Fixed             = unorgFixed;
            this.Metric            = options.metric;
            this.DoExtrapolate     = options.doExtrapolate;
            this.MaxIterations     = options.maxIterations;
            this.Tolerance         = options.tolerance;
            this.InitialTransform  = options.initTform;
            this.UseDegree         = options.useDegree;
            this.Verbose           = options.verbose;
            this.InlierDistance    = options.inlierDistance;
            this.CovarianceMoving  = coder.nullcopy(zeros(3,3,size(this.Moving,1),'like',this.Moving.Location));
            this.CovarianceFixed   = coder.nullcopy(zeros(3,3,size(this.Fixed,1),'like',this.Fixed.Location));
            if isempty(this.InlierDistance)
                this.UseInlierRatio = true;
                this.InlierRatio = options.inlierRatio;
                this.UseAllMatches = options.inlierRatio == 1;
            else % Use InlierDistance
                this.UseInlierRatio = false;
                this.UseAllMatches = false;
                this.InlierRatio = 1;
            end
        end

        %------------------------------------------------------------------
        function initializeRegistration(this)
            maxIter = this.MaxIterations;

            this.Rs = coder.nullcopy(zeros(3, 3, maxIter+1));
            this.Ts = coder.nullcopy(zeros(3, maxIter+1));

            % Quaternion and translation vector
            this.qs = [ones(1, maxIter+1); zeros(6, maxIter+1)];

            % The difference of quaternion and translation vector in consecutive
            % iterations
            this.dq = zeros(7, maxIter+1);

            % The angle between quaternion and translation vectors in consecutive
            % iterations
            this.dTheta = zeros(maxIter+1, 1);

            % Inlier RMSE
            this.Err = zeros(maxIter+1, 1);

            % Apply the initial condition.
            % We use pre-multiplication format in this algorithm.
            this.Rs(:,:,1) = this.InitialTransform.R;
            this.Ts(:,1)   = this.InitialTransform.Translation';
            this.qs(:,1)   = [vision.internal.quaternion.rotationToQuaternion(this.Rs(:,:,1)); this.Ts(:,1)];

            this.StopIteration = this.MaxIterations;

            if isSimMode()
                this.Printer = vision.internal.MessagePrinter.configure(this.Verbose);
            end

            this.LocA = this.Moving.Location;
            if this.qs(1) ~= 0 || any(this.qs(2:end,1))
                this.LocA = rigidTransform(this.Moving.Location, this.Rs(:,:,1), this.Ts(:,1));
            end

            numPointsA = this.Moving.Count;
            if this.UseInlierRatio
                this.MaxNumInliersA = max(1, round(this.InlierRatio * numPointsA));
            else
                this.MaxNumInliersA = numPointsA;
            end
            if this.Metric=="planeToPlane"
                % find covariance matrices
                this.CovarianceMoving = findCovarianceMatrix(this.Moving.Normal);
                this.CovarianceFixed  = findCovarianceMatrix(this.Fixed.Normal);
            end
        end

        %------------------------------------------------------------------
        function [tform, rmseInliers] = registerPointClouds(this)
           
            % Start ICP iterations
            for iter = 1 : this.MaxIterations

                % Find correspondences
                this.findCorrespondences(iter);
     
                % Estimate transformation given correspondences
                [R, T] = this.estimateTransformation;

                % Update transformation
                this.updateTotalTransformation(R, T, iter)

                % With extrapolation, we might be able to converge faster
                if this.DoExtrapolate
                    this.extrapolateInTransformSpace(iter);
                end

                % Stop ICP if it already converges
                if this.hasConverged(iter)
                    this.StopIteration = iter;
                    break;
                end
            end
            
            [tform, rmseInliersOut] = this.computeFinalTransformation;

            rmseInliers = cast(rmseInliersOut, 'like', this.Moving.Location);
        end
    end

    %----------------------------------------------------------------------
    % Methods related to each step in the iteration.
    %----------------------------------------------------------------------
    methods (Access = private)

        %------------------------------------------------------------------
        function findCorrespondences(this, iter)

            if this.Verbose && isSimMode()
                this.Printer.linebreak;
                this.Printer.print('--------------------------------------------\n');
                this.Printer.printMessage('vision:pointcloud:icpIteration',iter);
                this.Printer.printMessageNoReturn('vision:pointcloud:findCorrespondenceStart');
            end
           
            if this.UseAllMatches
                % Find correspondences
                [this.InlierIndicesB, inlierDists] = multiQueryKNNSearchImpl(this.Fixed, this.LocA, 1);
                this.InlierIndicesA = (1 : this.MaxNumInliersA).';
            else
                % Find correspondences
                [indices, dists] = multiQueryKNNSearchImpl(this.Fixed, this.LocA, 1);

                % Remove outliers
                [this.InlierIndicesA, this.InlierIndicesB, inlierDists] ...
                    = this.rejectOutliers(indices, dists);
            end

            coder.internal.errorIf(numel(this.InlierIndicesA) < 3, ...
                'vision:pointcloud:notEnoughPoints');

            if iter == 1
                this.Err(iter) = sqrt(sum(inlierDists(:))/numel(inlierDists));
            end

            if this.Verbose && isSimMode()
                this.Printer.printMessage('vision:pointcloud:stepCompleted');
                this.Printer.printMessageNoReturn('vision:pointcloud:estimateTransformStart');
            end
        end

        %------------------------------------------------------------------
        function [inliersA, inliersB, inlierDists] = rejectOutliers(this, indices, dists)

            coder.inline('never')
            coder.gpu.kernelfun;

            numPointsA = this.Moving.Count;
            keepInlierA = false(numPointsA, 1);

            if this.UseInlierRatio % Threshold inliers using inlier ratio
                if coder.gpu.internal.isGpuEnabled
                    [~,idxall] = gpucoder.sort(dists,'ascend');
                    idx = idxall(1:this.MaxNumInliersA);
                else
                    [~, idx] = mink(dists, this.MaxNumInliersA, 2);
                end

                keepInlierA(idx) = true;
                coder.internal.errorIf(nnz(keepInlierA) < 3, ...
                    'vision:pointcloud:increaseInlierRatio');
            else % Use maximum distance between inliers
                keepInlierA(dists <= this.InlierDistance^2) = true;
                coder.internal.errorIf(nnz(keepInlierA) < 3, ...
                    'vision:pointcloud:increaseInlierDistance');
            end


            inliersA = find(keepInlierA);
            if isSimMode()
                inliersB    = indices(keepInlierA);
                inlierDists = dists(keepInlierA);
            else
                inliersB    = indices(keepInlierA');
                inlierDists = dists(keepInlierA');
            end
        end

        %------------------------------------------------------------------
        function [R, T] = estimateTransformation(this)

            switch this.Metric
                case "pointToPoint"
                    [R, T] = vision.internal.calibration.computeRigidTransform(...
                        this.LocA(this.InlierIndicesA, :), ...
                        this.Fixed.Location(this.InlierIndicesB, :));
                case "pointToPlane"
                    [R, T] = pointToPlaneMetric(this.LocA(this.InlierIndicesA, :), ...
                        this.Fixed.Location(this.InlierIndicesB, :), ...
                        this.Fixed.Normal(this.InlierIndicesB, :));
                case "planeToPlane"
                    [R, T] = planeToPlaneMetric(this.LocA(this.InlierIndicesA,:), ...
                        this.Fixed.Location(this.InlierIndicesB,:), ...
                        this.CovarianceMoving(:,:,this.InlierIndicesA), ...
                        this.CovarianceFixed(:,:,this.InlierIndicesB));
                otherwise
                    R = cast([], 'like', this.LocA);
                    T = cast([], 'like', this.LocA);
            end
        end

        %------------------------------------------------------------------
        function updateTotalTransformation(this, R, T, iter)

            % Bad correspondence may lead to singular matrix
            coder.internal.errorIf(any(isnan(T), 'all') || any(isnan(R), 'all') , ...
                'vision:pointcloud:singularMatrix');

            % Update the total transformation
            Rnew = R * this.Rs(:,:,iter);
            Tnew = R * this.Ts(:,iter) + T;
            this.Rs(:,:,iter+1) = Rnew;
            this.Ts(:,iter+1)   = Tnew;

            % Inlier RMSE
            this.LocA = rigidTransform(this.Moving.Location, ...
                this.Rs(:,:,iter+1), this.Ts(:,iter+1));
            squaredError = sum((this.LocA(this.InlierIndicesA, :) - ...
                this.Fixed.Location(this.InlierIndicesB, :)).^2, 2);
            this.Err(iter+1) = sqrt(sum(squaredError)/numel(squaredError));

            % Convert to vector representation
            this.qs(:,iter+1) = [vision.internal.quaternion.rotationToQuaternion(Rnew); Tnew];

            if this.Verbose && isSimMode()
                this.Printer.printMessage('vision:pointcloud:stepCompleted');
            end
        end

        %------------------------------------------------------------------
        % Perform extrapolation: Besl, P., & McKay, N. (1992). A method for
        % registration of 3-D shapes. IEEE Transactions on pattern analysis
        % and machine intelligence, p245.
        %------------------------------------------------------------------
        function extrapolateInTransformSpace(this, iter)
            if this.Verbose && isSimMode()
                this.Printer.printMessageNoReturn('vision:pointcloud:updateTransformStart');
            end

            this.dq(:,iter+1) = this.qs(:,iter+1) - this.qs(:,iter);
            n1 = norm(this.dq(:,iter));
            n2 = norm(this.dq(:,iter+1));
            this.dTheta(iter+1) = (180/pi)*acos(dot(this.dq(:,iter),this.dq(:,iter+1))/(n1*n2));

            angleThreshold = 10;
            scaleFactor    = 25;
            if iter > 2 && this.dTheta(iter+1) < angleThreshold && this.dTheta(iter) < angleThreshold
                d = [this.Err(iter+1), this.Err(iter), this.Err(iter-1)];
                v = [0, -n2, -n1-n2];
                vmax = scaleFactor * n2;
                dv = extrapolate(v,d,vmax);
                if dv ~= 0
                    q = this.qs(:,iter+1) + dv * this.dq(:,iter+1)/n2;
                    q(1:4) = q(1:4)/norm(q(1:4));
                    % Update transformation and data
                    this.qs(:,iter+1) = q;
                    this.Rs(:,:,iter+1) = vision.internal.quaternion.quaternionToRotation(q(1:4));
                    this.Ts(:,iter+1) = q(5:7);
                    this.LocA = rigidTransform(this.Moving.Location, this.Rs(:,:,iter+1), this.Ts(:,iter+1));
                end
            end

            if this.Verbose && isSimMode()
                this.Printer.printMessage('vision:pointcloud:stepCompleted');
            end
        end

        %------------------------------------------------------------------
        function TF = hasConverged(this, iter)
            % Check convergence
            % Compute the mean difference in R/T from the recent three iterations.
            [dR, dT, rdiff, tdiff] = this.getChangesInTransformation(iter);

            if this.Verbose && isSimMode()
                % Convert from radians to degrees.
                if this.UseDegree
                    rdiff = rdiff*180/pi;
                end

                this.Printer.printMessage('vision:pointcloud:checkConverge', ...
                    num2str(tdiff), num2str(rdiff), num2str(this.Err(iter+1)));
            end

            TF = dT <= this.Tolerance(1) && dR <= this.Tolerance(2);
        end

        %------------------------------------------------------------------
        % Compute the changes in rotation and translation
        %------------------------------------------------------------------
        function [dR, dT, rdiff, tdiff] = getChangesInTransformation(this, iter)
            dR = 0;
            dT = 0;
            rdiff = 0;
            tdiff = 0;
            count = 0;
            for k = max(iter-2,1):iter

                % Rotation difference in radians
                q1 = this.qs(1:4,k);
                q2 = this.qs(1:4,k+1);
                cosTheta = dot(q1,q2)/(norm(q1)*norm(q2));
                rdiff = acos(min(1, max(-1, cosTheta)));

                % Euclidean difference
                t1 = this.Ts(:,k);
                t2 = this.Ts(:,k+1);
                tdiff = norm(t1-t2);

                % Accumulate differences
                dR = dR + rdiff;
                dT = dT + tdiff;

                count = count + 1;
            end
            dT = dT/count;
            dR = dR/count;
        end

        %------------------------------------------------------------------
        function [tform, rmseInliers] = computeFinalTransformation(this)

            % Make the R to be orthogonal as much as possible
            R = this.Rs(:,:,this.StopIteration+1)';
            [U, ~, V] = svd(R);
            R = V * U';

            rotation = cast(R, 'like', this.Moving.Location);
            translation = cast(this.Ts(:, this.StopIteration+1)', 'like', this.Moving.Location);
            tform = rigidtform3d(rotation, translation);

            rmseInliers = this.Err(this.StopIteration+1);

            if this.Verbose && isSimMode()
                this.Printer.linebreak;
                this.Printer.print('--------------------------------------------\n');
                this.Printer.printMessage('vision:pointcloud:icpSummary',...
                    this.StopIteration);
            end
        end
    end
end

%--------------------------------------------------------------------------
function B = rigidTransform(A, R, T)

    B = A * R' + T';
end

%--------------------------------------------------------------------------
% Extrapolation in quaternion space. Details are found in:
% Besl, P., & McKay, N. (1992). A method for registration of 3-D shapes.
% IEEE Transactions on pattern analysis and machine intelligence, 239-256.
%--------------------------------------------------------------------------
function dv = extrapolate(v,d,vmax)
    p1 = polyfit(v,d,1);    % linear fit
    p2 = polyfit(v,d,2);    % parabolic fit
    v1 = -p1(2)/p1(1);      % linear zero crossing point
    v2 = -p2(2)/(2*p2(1));  % polynomial top point

    if (issorted([0 v2 v1 vmax]) || issorted([0 v2 vmax v1]))
        % Parabolic update
        dv = v2;
    elseif (issorted([0 v1 v2 vmax]) || issorted([0 v1 vmax v2])...
            || (v2 < 0 && issorted([0 v1 vmax])))
        % Line update
        dv = v1;
    elseif (v1 > vmax && v2 > vmax)
        % Maximum update
        dv = vmax;
    else
        % No extrapolation
        dv = 0;
    end
end

%--------------------------------------------------------------------------
% Solve the following minimization problem:
%       min_{R, T} sum(|dot(R*p+T-q,nv)|^2)
%
% p, q, nv are all N-by-3 matrix, and nv is the unit normal at q
%
% Here the problem is solved by linear approximation to the rotation matrix
% when the angle is small.
%--------------------------------------------------------------------------
function [R, T] = pointToPlaneMetric(p, q, nv)

    % Set up the linear system
    cn = [cross(p,nv,2),nv];
    C = cn'*cn;
    qp = q-p;
    b =  [...
        iSum(qp .* cn(:,1) .* nv, 'all');
        iSum(qp .* cn(:,2) .* nv, 'all');
        iSum(qp .* cn(:,3) .* nv, 'all');
        iSum(qp .* cn(:,4) .* nv, 'all');
        iSum(qp .* cn(:,5) .* nv, 'all');
        iSum(qp .* cn(:,6) .* nv, 'all')];

    % X is [alpha, beta, gamma, Tx, Ty, Tz]
    X = C\b;

    [R,T] = convertTransformation(X);
end

%-----------------------------------------------------------------------
% planeToPlaneMetric function returns R and T given inlier correspondences
% p,q and their convariance matrices covp and covq.
% 
% Algorithm:
% 1. Calculate covariance matrices for each point (In initializeRegistration function).
% 2. Calculate difference in the point cloud locations.
% 3. Calculate the sum of the covariance matrices in M.
% 4. Find the inverse and square root of M in W.
% 5. Compute the Jacobians and form the linear system of equations.
% 6. Solve the linear system of equations and obtain the transformation.
%-----------------------------------------------------------------------
function [R, T] = planeToPlaneMetric(p, q, covp,covq)  
    coder.gpu.kernelfun;
    numInliers = coder.internal.indexInt(size(p,1));
    J = coder.nullcopy(zeros(3,6,numInliers,'like',p));
    W = coder.nullcopy(zeros(3,3,numInliers,'like',p));
    JTr = coder.nullcopy(zeros(numInliers,6,'like',p));
    JTJ = coder.nullcopy(zeros(6,6,numInliers,'like',p));
    pq = p-q;
    M = covp+covq;
    
    % Find inverse of the matrix M.
    % Assuming that matrices are symmetric and non singular.
    coder.gpu.kernel;
    for i=1:numInliers
        W(:,:,i) = M(:,:,i)\eye(3);
    end
    
    % Find square root of the matrix W.
    coder.gpu.kernel;
    for i=1:numInliers
        W(:,:,i) = real(iSqrtm(W(:,:,i)));
    end
    
    % Set up the linear system
    coder.gpu.kernel;
    for i=1:numInliers
        J(:,:,i) = [   0     p(i,3)  -p(i,2) 1 0 0;
                    -p(i,3)    0      p(i,1) 0 1 0;
                     p(i,2) -p(i,1)     0    0 0 1];

        J(:,:,i)   = W(:,:,i)*J(:,:,i);

        JTr(i,:)   = J(1,:,i)*sum(pq(i,:).*W(1,:,i)) + ... 
                     J(2,:,i)*sum(pq(i,:).*W(2,:,i)) + ...
                     J(3,:,i)*sum(pq(i,:).*W(3,:,i));
        
        JTJ(:,:,i) = J(1,:,i)'*J(1,:,i) + ...
                     J(2,:,i)'*J(2,:,i) + ...
                     J(3,:,i)'*J(3,:,i);
    end
   
    jtr = sum(JTr);
    jtj = sum(JTJ,3);
    
    % Solve the linear system
    % X is [alpha, beta, gamma, Tx, Ty, Tz]
    X = jtj\(-jtr)';

    [R,T] = convertTransformation(X);
    
end
%--------------------------------------------------------------------------
function covarianceMat = findCovarianceMatrix(Normal)
    coder.inline('never');
    coder.gpu.kernelfun;
    covarianceMat = coder.nullcopy(zeros(3,3,size(Normal,1),'like',Normal));
    % epsilon indicating the low covariance along the surface normal
    epsilon = 1e-3;
    C = cast([epsilon 0 0;0 1 0;0 0 1],'like',Normal);
    
    coder.gpu.kernel;
    for i = 1:size(Normal,1)
        % Obtain the Rotation matrix that transforms the basis vector e1
        % onto the input vector x
        Rx = getRotationToX(Normal(i,:));
        covarianceMat(:,:,i) = Rx*C*Rx';
    end
end
%--------------------------------------------------------------------------
function rot = getRotationToX(n)
    % Initialize the basis vector e1
    e1 = cast([1 0 0],'like',n);
    v = cross(e1,n);
    c = dot(e1,n);

    if c< -0.99
        % That means x and e1 are in the same direction
        rot = eye(3,'like',n);
    else
        % Skew matrix using vector v
        sv = [0 -v(3) v(2);
            v(3) 0 -v(1);
            -v(2) v(1) 0];
        factor = 1/(1+c);
        rot = eye(3)+sv+(sv*sv)*factor;
    end
end
%--------------------------------------------------------------------------
% convertTransformation function transforms the input X which is 
% [alpha, beta, gamma, Tx, Ty, Tz] and gives output Rotation matrix R and
% Translation vector T.
%--------------------------------------------------------------------------
function [R,T] = convertTransformation(X)
    cx = cos(X(1));
    cy = cos(X(2));
    cz = cos(X(3));
    sx = sin(X(1));
    sy = sin(X(2));
    sz = sin(X(3));

    R = [cy*cz, sx*sy*cz-cx*sz, cx*sy*cz+sx*sz;
        cy*sz, cx*cz+sx*sy*sz, cx*sy*sz-sx*cz;
        -sy,          sx*cy,          cx*cy];

    T = X(4:6);
end
%--------------------------------------------------------------------------
% Function for calculating sum of array
function result = iSum(A,varargin)
count = size(A,1);
result = zeros(1,1,'like',A);
for i = 1:count
    result = result + A(i,1)+A(i,2)+A(i,3);
end
end
%--------------------------------------------------------------------------
% Wrapper for sqrtm function
function mat = iSqrtm(M)
coder.inline('never')
mat = sqrtm(M);
end
%--------------------------------------------------------------------------
% Codegen support flag
%--------------------------------------------------------------------------
function flag = isSimMode()
flag = isempty(coder.target);
end