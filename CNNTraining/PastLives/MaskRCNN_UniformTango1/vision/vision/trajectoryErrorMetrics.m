classdef trajectoryErrorMetrics

% Copyright 2024 The MathWorks Inc.

%#codegen

    properties (SetAccess = protected)
        AbsoluteError

        RelativeError

        AbsoluteRMSE

        RelativeRMSE
    end

    properties (Hidden = true, SetAccess = protected)
        EstimatedTrajectory

        GroundTruthTrajectory
    end

    properties(Access = protected)
        Version = 1.0
    end

    methods (Hidden = true, Static = true)
        function this = create(estimationTranslations, groundTruthTranslation, ...
                absoluteRotationErrors, absoluteTranslationErrors, ...
                relativeRotationErrors, relativeTranslationErrors)
            arguments
                estimationTranslations {mustBeReal,mustBeNonempty,mustBeNonsparse,mustBeFinite,mustHaveThreeColumns(estimationTranslations)}
                groundTruthTranslation {mustBeReal,mustBeNonempty,mustBeNonsparse,mustBeFinite,mustHaveSameSize(groundTruthTranslation,estimationTranslations)}
                absoluteRotationErrors {mustBeReal,mustBeNonsparse,mustBeFinite,mustBeVector,mustHaveSameLength(absoluteRotationErrors,estimationTranslations)}
                absoluteTranslationErrors {mustBeReal,mustBeNonsparse,mustBeFinite,mustBeVector,mustHaveSameSize(absoluteTranslationErrors,absoluteRotationErrors)}
                relativeRotationErrors {mustBeReal,mustBeNonsparse,mustBeFinite,mustBeVector,mustHaveSameSize(relativeRotationErrors,absoluteRotationErrors)}
                relativeTranslationErrors {mustBeReal,mustBeNonsparse,mustBeFinite,mustBeVector,mustHaveSameSize(relativeTranslationErrors,absoluteRotationErrors)}
            end 


            this = trajectoryErrorMetrics(estimationTranslations, groundTruthTranslation, ...
                absoluteRotationErrors(:), absoluteTranslationErrors(:), ...
                relativeRotationErrors(:), relativeTranslationErrors(:));
        end
    end

    methods (Hidden = true, Access = protected)
        function this = trajectoryErrorMetrics(estimationTranslations, groundTruthTranslation, ...
                absoluteRotationErrors, absoluteTranslationErrors, ...
                relativeRotationErrors, relativeTranslationErrors)

            this.EstimatedTrajectory = estimationTranslations;
            this.GroundTruthTrajectory = groundTruthTranslation;

            this.AbsoluteError = [absoluteRotationErrors, absoluteTranslationErrors];
            this.RelativeError = [relativeRotationErrors, relativeTranslationErrors];

            this.AbsoluteRMSE  = [rms(absoluteRotationErrors), rms(absoluteTranslationErrors)];
            this.RelativeRMSE  = [rms(relativeRotationErrors), rms(relativeTranslationErrors)];
        end
    end

    methods
        function varargout = plot(this, errorMetric, args)
            arguments
                this
                errorMetric {mustBeMember(errorMetric,["absolute-translation","absolute-rotation","relative-translation","relative-rotation"])}
                args.Parent = [];
                args.ShowGroundTruth (1,1) {mustBeScalarOrEmpty, mustBeNonempty, mustBeNumericOrLogical}=true
                args.VerticalAxis (1,1) string {mustBeMember(args.VerticalAxis,["ZUp", "ZDown","XUp", "XDown", "YUp", "YDown"])} = "YDown";
            end
			
			nargoutchk(0,1);
            
            if ~isempty(args.Parent)
                uiaxesSupported = true;
                vision.internal.inputValidation.validateAxesHandle(args.Parent, uiaxesSupported);
            end

            ax = newplot(args.Parent);

            axis(ax, "equal");
            grid(ax, "on");

            if errorMetric=="absolute-translation"
                color=this.AbsoluteError(:,2);
                titleText = message("vision:compareTrajectories:absoluteTranslationTitle").getString;
            elseif errorMetric=="absolute-rotation"
                color=this.AbsoluteError(:,1);
                titleText = message("vision:compareTrajectories:absoluteRotationTitle").getString;
            elseif errorMetric=="relative-translation"
                color=this.RelativeError(:,2);
                titleText = message("vision:compareTrajectories:relativeTranslationTitle").getString;
            else
                color=this.RelativeError(:,1);
                titleText = message("vision:compareTrajectories:relativeRotationTitle").getString;
            end
            
            % Use the patch function to plot the trajectory colored by the
            % specified error metric.
            pEstimation = patch(ax,[this.EstimatedTrajectory(:,1); nan],... % nan to keep the 3-D line open
                [this.EstimatedTrajectory(:,2); nan],...
                [this.EstimatedTrajectory(:,3); nan],...
                [color; nan],...
                FaceColor="none",...
                EdgeColor="interp",LineWidth=2);
            colorbar(ax);

            legendText = {message("vision:compareTrajectories:estimatedTrajectoryLegend").getString};

            hold(ax, "on");

            if args.ShowGroundTruth
                pGroundTruth = plot3(ax, this.GroundTruthTrajectory(:,1),this.GroundTruthTrajectory(:,2),this.GroundTruthTrajectory(:,3), ...
                    '-', LineWidth=2, Color=[0.3, 0.3, 0.3]);
                legendText{2} = message("vision:compareTrajectories:groundTruthTrajectoryLegend").getString; %#ok<*NASGU>
                legend(ax,[pEstimation,pGroundTruth],legendText);
            else
                legend(ax,pEstimation,legendText);
            end

            cameraUpVector = vision.internal.pcviewer.getCamUpVectorByVertAxis(args.VerticalAxis);
            view(ax, 3);
            set(ax, CameraUpVector=cameraUpVector);

            xlabel(ax,"X");
            ylabel(ax,"Y");
            zlabel(ax,"Z");

            title(ax, titleText);

            hold(ax, "off");
			
			if nargout==1
                varargout{1} = ax;
            end
        end
    end

end

function mustHaveThreeColumns(value)
validateattributes(value, {'numeric'}, {'size',[NaN, 3]});
end

function mustHaveSameSize(value1, value2)
validateattributes(value1, {'numeric'}, {'size', size(value2)});
end

function mustHaveSameLength(value1, value2)
validateattributes(value1(:), {'numeric'}, {'size', size(value2(:,1))});
end