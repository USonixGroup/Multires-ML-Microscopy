classdef bundleAdjustmentValidation
    %
    % bundleAdjustmentValidation class consists of validation functions of bundleAdjustment
    % which are common for both codegen and simulation
    
    %  Copyright 2022 The MathWorks, Inc.
    %#codegen
    methods(Static)
        %--------------------------------------------------------------------------
        function returnType = validateMapPointSet(wpSet, funcName)
            validateattributes(wpSet, {'worldpointset'}, {'scalar', 'nonempty'}, funcName, 'wpSet');
            returnType = class(wpSet.WorldPoints);
        end

        %--------------------------------------------------------------------------
        function validateViewSet(vSet, funcName)
            validateattributes(vSet, {'imageviewset'}, {'scalar', 'nonempty'}, funcName, 'vSet');
        end

        %------------------------------------------------------------------
        function viewIds = validateViewIds(viewIds, funcName)
            viewIds = vision.internal.inputValidation.checkViewIds(viewIds, false, funcName, 'viewIds');
        end

        %--------------------------------------------------------------------------
        function checkIfViewIdsMissing(viewIds, wpSet, vSet)
            vision.internal.inputValidation.checkIfViewIsMissing(wpSet.ViewIds, viewIds);
            vision.internal.inputValidation.checkIfViewIsMissing(vSet.Views.ViewId, viewIds);
        end
        %--------------------------------------------------------------------------
        function validateSinglePose(pose, funcName)
            validateattributes(pose, {'rigidtform3d', 'rigid3d'}, {'scalar','nonempty'},...
                funcName, 'absolutePose');
        end
        %--------------------------------------------------------------------------
        function tf = validateFixedViewIDs(value, funcName)
            tf = true;
            if ~isempty(value)
                validateattributes(value,{'numeric'}, {'vector','integer','nonnegative'}, ...
                    funcName, 'FixedViewIDs');
            end
        end

        %--------------------------------------------------------------------------
        function validateMaxIterations(maxIter, funcName)
            validateattributes(maxIter,{'single', 'double'}, {'scalar','integer', 'positive'}, ...
                funcName, 'MaxIterations')
        end

        %--------------------------------------------------------------------------
        function validateTolerance(tol, funcName, argName)
            validateattributes(tol,{'single', 'double'}, {'real','nonnegative','scalar','finite'}, ...
                funcName, argName);
        end

        %--------------------------------------------------------------------------
        function validateSolver(solver, funcName, argName)
            % All available solvers
            validSolvers = {'preconditioned-conjugate-gradient', 'sparse-linear-algebra'};
            validatestring(solver, validSolvers, funcName, argName);
        end
    end
end