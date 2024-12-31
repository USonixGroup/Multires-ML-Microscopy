classdef VisualSLAMViewer < pcplayer
%VisualSLAMViewer show map points and camera trajectory

%   Copyright 2022-2023 The MathWorks, Inc.

    properties (GetAccess = ?matlab.unittest.TestCase)
        %CameraTrajectory
        %   Plot of the camera trajectory 
        CameraTrajectory
        
        %CurrentCamera
        %   Plot of the current camera
        CurrentCamera
    end

    properties (Access = private)
        AxesLimitsInitialized = false
    end

    methods (Access = public)
        function obj = VisualSLAMViewer(xyzPoints, camPoses, options)

            trajectory = vertcat(camPoses.Translation);
            
            axesLimitsInitialized = false;
            if isempty(xyzPoints)
                xLim =[-1, 1];
                yLim = xLim;
                zLim = xLim;
            else
                [xLim, yLim, zLim] = initializeAxesLimits([xyzPoints; trajectory], options.CameraSize);
                axesLimitsInitialized = true;
            end
            
            % Create a pcplayer object
            if isempty(options.Parent)
                parent = newplot(figure(Visible="off"));
            else
                parent = options.Parent;
            end
            obj@pcplayer(xLim, yLim, zLim, ...
                MarkerSize=options.MarkerSize, ...                
                VerticalAxis="y", ...
                VerticalAxisDir="down", ...
                Parent=parent);

            obj.AxesLimitsInitialized = axesLimitsInitialized;

            % Create plot components and plot map points and camera trajectory
            newPlot(obj, xyzPoints, camPoses, trajectory, options);
        end

        function newPlot(obj, xyzPoints, camPoses, trajectory, options)

            % Plot map points
            plotMapPoints(obj, xyzPoints, options);


            % Plot the current camera and trajectory
            if ~isempty(camPoses)
                hold(obj.Axes, 'on');
                initializeTrajectory(obj, trajectory, options);
                initializeCurrentCamera(obj, camPoses(end), options);
                hold(obj.Axes, 'off');
            end
        end

        function updatePlot(obj, xyzPoints, camPoses, isLoopClosed, options)

            if strcmpi(obj.Figure.Visible, 'off')
                figure(obj.Figure); % bring to front
            end

            if ~isempty(camPoses)
                currPose = camPoses(end);
                trajectory = vertcat(camPoses.Translation);

                % Update the map points
                if ~isempty(xyzPoints)
                    plotMapPoints(obj, xyzPoints, options);
                end

                if isempty(obj.CurrentCamera)
                    hold(obj.Axes, 'on');
                    initializeTrajectory(obj, trajectory, options);
                    initializeCurrentCamera(obj, currPose, options);
                    hold(obj.Axes, 'off');
                else
                    updateTrajectory(obj, trajectory, options);
                    updateCurrentCamera(obj, currPose, options);
                end

                % Update axis limits
                updateAxesLimits(obj, xyzPoints, currPose, trajectory, isLoopClosed, options);
            end

            drawnow('limitrate');
        end
    end

    methods (Access = private)
        function plotMapPoints(obj, xyzPoints, options)
            if isempty(options.MarkerColor)
                pointclouds.internal.pcui.setColorMapData(obj.Axes, options.ColorSource);
                obj.view(xyzPoints);
            else
                obj.view(xyzPoints, options.MarkerColor);
            end
        end

        function initializeTrajectory(obj, trajectory, options)
            obj.CameraTrajectory = plot3(obj.Axes, trajectory(:,1), trajectory(:,2), ...
                    trajectory(:,3), Color=options.CameraColor, LineWidth=2, ...
                    DisplayName="Camera trajectory");
        end

        function updateTrajectory(obj, trajectory, options)
            set(obj.CameraTrajectory, XData=trajectory(:,1), ...
                YData=trajectory(:,2), ...
                ZData=trajectory(:,3), ...
                Color=options.CameraColor);
        end

        function initializeCurrentCamera(obj, currPose, options)
            obj.CurrentCamera = plotCamera(AbsolutePose=currPose, ...
                    Parent=obj.Axes, Size=options.CameraSize, Color=options.CameraColor);
        end

        function updateCurrentCamera(obj, currPose, options)
            obj.CurrentCamera.AbsolutePose=currPose;
            obj.CurrentCamera.Size=options.CameraSize;
            obj.CurrentCamera.Color=options.CameraColor;
        end

        function updateAxesLimits(obj, xyzPoints, currPose, trajectory, isLoopClosed, options)
            if ~isempty(xyzPoints)
                if ~obj.AxesLimitsInitialized
                    [xLim, yLim, zLim] = initializeAxesLimits([xyzPoints; trajectory], options.CameraSize);
                    obj.AxesLimitsInitialized = true;
                elseif isLoopClosed % Camera poses changed after pose graph optimization
                    [xLim, yLim, zLim] = initializeAxesLimits(trajectory, options.CameraSize);
                else
                    [xLim, yLim, zLim] = computeAxesLimits(obj.Axes, currPose, options.CameraSize);
                end
                set(obj.Axes, XLim=xLim, YLim=yLim, ZLim=zLim);
            end
        end
    end
end

function [xLim, yLim, zLim] = initializeAxesLimits(xyz, cameraSize)
lo = min(xyz);
hi = max(xyz);
factor = 0.25;
nlo = lo - factor/2*(hi-lo) - cameraSize;
nhi = hi + factor/2*(hi-lo) + cameraSize;

xLim = [nlo(1), nhi(1)];
yLim = [nlo(2), nhi(2)];
zLim = [nlo(3), nhi(3)];
end

function [xLim, yLim, zLim] = computeAxesLimits(axes, currPose, cameraSize)
factor = 3; % The radius of the sphere enclosing a size-L camera is sqrt(6)*L
xLim=[min(axes.XLim(1), currPose.Translation(1)-factor*cameraSize), ...
      max(axes.XLim(2), currPose.Translation(1)+factor*cameraSize)];
yLim=[min(axes.YLim(1), currPose.Translation(2)-factor*cameraSize), ...
      max(axes.YLim(2), currPose.Translation(2)+factor*cameraSize)];
zLim=[min(axes.ZLim(1), currPose.Translation(3)-factor*cameraSize), ...
      max(axes.ZLim(2), currPose.Translation(3)+factor*cameraSize)];
end
