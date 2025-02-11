classdef TemporalInterpolator < vision.labeler.AutomationAlgorithm & vision.labeler.mixin.Temporal
    %TemporalInterpolator Automation algorithm to estimate ROIs using temporal interpolation.
    %   TemporalInterpolator is a temporal automation algorithm in the
    %   Video Labeler and the Ground Truth Labeler Apps.  Ground Truth
    %   Labeler App requires that you have the Automated Driving
    %   Toolbox(TM). The algorithm estimates the position of rectangle 
    %   or projected cuboid ROIs in the intermediate frames by interpolating 
    %   the position of the ROIs defined in the key frames. There must be 
    %   at least two key frames. The estimation is performed using linear 
    %   interpolation between two successive key frames.
    %
    %   See also videoLabler, vision.labeler.AutomationAlgorithm,
    %   vision.labeler.mixin.Temporal.

    % Copyright 2017-2023 The MathWorks, Inc.
    
    %----------------------------------------------------------------------
    % Algorithm Description
    %----------------------------------------------------------------------    
    properties(Constant)
        
        %Name Algorithm Name
        %   Character vector specifying name of algorithm.
        Name = vision.getMessage('vision:labeler:TemporalInterpolatorName');
        
        %Description Algorithm Description
        %   Character vector specifying short description of algorithm.
        Description = vision.getMessage('vision:labeler:TemporalInterpolatorDesc');
        
        %UserDirections Algorithm Usage Directions
        %   Cell array of character vectors specifying directions for
        %   algorithm users to follow in order to use algorithm.
        UserDirections = {...
            vision.getMessage('vision:labeler:TemporalInterpolatorROISelection'), ...
            vision.getMessage('vision:labeler:TemporalInterpolatorSetAttributes'), ...
            vision.getMessage('vision:labeler:TemporalInterpolatorRun'),...
            vision.labeler.AutomationAlgorithm.getDefaultUserDirections('review'),...
            vision.getMessage('vision:labeler:TemporalInterpolatorUndoRun'),...
            vision.labeler.AutomationAlgorithm.getDefaultUserDirections('accept')};
    end

    %----------------------------------------------------------------------
    % Properties
    %----------------------------------------------------------------------    
    properties
        
        %InitialLabels Set of labels at algorithm initialization
        %   Table with columns Name, Type and Position for labels marked at
        %   initialization.        
        InitialLabels;
        
        %prevROI Previous ROI position
        %   A vector of length 4 or 8
        prevROI;
       
        %delX change in ROI position in x-direction
        %   A vector of length N-1, where N is the number of ROIs defined
        %   manually
        delX;
        
        %delY change in ROI position in y-direction
        %   A vector of length N-1, where N is the number of ROIs defined
        %   manually        
        delY;
       
        %delW change in ROI width
        %   A vector of length N-1, where N is the number of ROIs defined
        %   manually        
        delW;
       
        %delH change in ROI height
        %   A vector of length N-1, where N is the number of ROIs defined
        %   manually        
        delH;

        %delA change in Rotated Rectangle ROI angle
        %   A vector of length N-1, where N is the number of ROIs defined
        %   manually
        delA;

        % Another set pf X,Y,W, and H for back face of projected cuboid.
        delX2;
        delY2;
        delW2;
        delH2;

        %ROIType ROI type
        %   A character array
        ROIType;
        
        %ROIName ROI name
        %   A character array
        ROIName;
        
        %Idx Index
        %   A scalar
        Idx;   

        % Flag to indicate presence of attributes.
        hasAttributes = 0;
        
        % Attribute Data
        %   struct array to store label attributes.
        AttribData;


    end
    
    properties (Access = private)
        
        %IsForwardAutomation Checks if automation is in forward direction     
        %   True if automation is in forward direction, False if automation
        %   is in reverse direction.
        IsForwardAutomation = true;        
    end
    
    %----------------------------------------------------------------------
    % Setup
    %----------------------------------------------------------------------
    methods
        
        function flag = supportsReverseAutomation(~)            
            flag = true;
        end  
        
        function isValid = checkLabelDefinition(~, labelDef)
            
            % Only Rectangular, Rotated Rectangle, and Projected Cuboid
            % ROI Label definitions are valid for the Temporal Interpolator.
            isValid = labelDef.Type == labelType.Rectangle || ...
                labelDef.Type == labelType.RotatedRectangle || ...
                labelDef.Type == labelType.ProjectedCuboid;
        end
        
        function isReady = checkSetup(algObj, labelsToAutomate)
            
            % Check-1: There must be at least two drawn ROIs
            numROIs = height(labelsToAutomate);
            assert(numROIs >= 2, ...
                vision.getMessage('vision:labeler:CreateAtLeastTwoROIsInDifferentKeyFrames'));            
            
            % Check-2: Each frame must contain at most 1 ROI
            frameTime = labelsToAutomate.Time;
            areTimestampsUnique = isequal(frameTime, unique(frameTime));
            
            repeatedTime = unique(frameTime(diff(frameTime)==0));
            msg = sprintf('Expected one ROI per video frame. Remove extra ROIs from frames at time: %s\n', num2str(repeatedTime(:)'));
            assert(areTimestampsUnique, msg);
            
            
            % Check-3: All ROIs must belong to the same label.
            hasSameLabels = (length(unique(labelsToAutomate.Name))==1);
            assert(hasSameLabels, ...
                'Expected all ROIs to belong to the same label definition.');


            % Check-4: Check if there are any attributes.
            if any(ismember(labelsToAutomate.Properties.VariableNames,'Attributes'))
               algObj.hasAttributes = 1;
            end

            isReady = true;
        end
    end
    
    %----------------------------------------------------------------------
    % Execution
    %----------------------------------------------------------------------
    methods
        function initialize(algObj, ~, labelsToAutomate)
             
            % Store the initial labels
            algObj.InitialLabels = labelsToAutomate;

            % Store type and name
            algObj.ROIType = labelsToAutomate.Type{1};
            algObj.ROIName = labelsToAutomate.Name{1};
            

            % Double check to ensure non-empty attributes.
            if (algObj.hasAttributes)
                if ~isempty(labelsToAutomate.Attributes{1}) 
                    algObj.hasAttributes = 1;
                else
                    algObj.hasAttributes = 0;
                end
            end

            % Initialize pointer index
            algObj.IsForwardAutomation = (strcmpi(algObj.AutomationDirection, 'forward'));
            if algObj.IsForwardAutomation
                algObj.Idx = 1;
            else
                algObj.Idx = size(algObj.InitialLabels,1);
            end

            % Compute sample time
            timeDuration = algObj.EndTime - algObj.StartTime;
            numFrames    = algObj.EndFrameIndex - algObj.StartFrameIndex;
            delT = timeDuration/numFrames;           
            
            % Compute interpolation parameters
            numROIs = height(labelsToAutomate);
            
            initROIPosition = labelsToAutomate.Position;
            initROITime = labelsToAutomate.Time;
            for n = 1 : numROIs-1
                startROIpos = initROIPosition(n,:);
                endROIpos   = initROIPosition(n+1,:);
                numFramesForInterp = (initROITime(n+1) - initROITime(n))/delT;
                
                algObj.delX(n) = (endROIpos(1) - startROIpos(1))/numFramesForInterp;
                algObj.delY(n) = (endROIpos(2) - startROIpos(2))/numFramesForInterp;
                algObj.delW(n) = (endROIpos(3) - startROIpos(3))/numFramesForInterp;
                algObj.delH(n) = (endROIpos(4) - startROIpos(4))/numFramesForInterp;
                if(algObj.ROIType == labelType.RotatedRectangle)
                    % Keep delta under 180 degrees and interpolate rotation
                    % direction instead of only going clockwise
                    % Rotation Directions by case:
                    %   CW (add abs(angleDelta) to prevAngle)
                    %   ----------------------------------------
                    %   angleDelta is positive
                    %   angleDeltaRev > -180 and < 0
                    %
                    %   OR
                    %
                    %   angleDelta is negative
                    %   angleDeltaRev >= 180
                    %
                    %   CCW (sub abs(angleDelta) to prevAngle)
                    %   ----------------------------------------
                    %   angleDelta is positive
                    %   angleDeltaRev <= -180
                    %
                    %   OR
                    %
                    %   angleDelta is negative
                    %   angleDeltaRev < 180 and >= 0

                    angleDelta = endROIpos(5) - startROIpos(5);
                    angleDeltaRev = startROIpos(5) - endROIpos(5);
                    if abs(angleDelta) > 180
                        angleDelta = sign(angleDelta)*(360 - sign(angleDelta)*angleDelta)/numFramesForInterp;
                    else
                        angleDelta = angleDelta/numFramesForInterp;
                    end
                    
                    % Determine if clockwise or counterclockwise rotation
                    if (angleDelta >= 0 && angleDeltaRev > -180 && angleDeltaRev < 0) || ...
                            (angleDelta < 0 && angleDeltaRev >= 180)
                        algObj.delA(n) = abs(angleDelta);
                    else
                        algObj.delA(n) = -abs(angleDelta);
                    end
                end
                if(algObj.ROIType == labelType.ProjectedCuboid)
                    algObj.delX2(n) = (endROIpos(5) - startROIpos(5))/numFramesForInterp;
                    algObj.delY2(n) = (endROIpos(6) - startROIpos(6))/numFramesForInterp;
                    algObj.delW2(n) = (endROIpos(7) - startROIpos(7))/numFramesForInterp;
                    algObj.delH2(n) = (endROIpos(8) - startROIpos(8))/numFramesForInterp; 
                end  

                % Store attributes in the following manner:
                % T1<T2  T1 has ROI R1 and T2 has ROI R2
                % Forward Automation: Interpolated ROIs between T1 and T2
                % have attributes of R1.
                % Reverse Automation: Interpolated ROIs between T1 and T2
                % have attributes of R2.
                if (algObj.hasAttributes) && (algObj.IsForwardAutomation)
                    algObj.AttribData{n} = labelsToAutomate.Attributes{n};
                elseif (algObj.hasAttributes) && ~(algObj.IsForwardAutomation)
                    algObj.AttribData{n} = labelsToAutomate.Attributes{n+1};
                end            
                
            end
            
            
        end
        
        function autoLabels = run(algObj, ~)
            
            beforeFirstROIDrawTime  = (algObj.CurrentTime < algObj.InitialLabels.Time(1));
            afterLastROIDrawTime    = (algObj.CurrentTime > algObj.InitialLabels.Time(end));
            
            % If the current image frame is before the first drawn label or
            % after the last drawn label, there are no automated ROIs.
            if beforeFirstROIDrawTime || afterLastROIDrawTime
                
                autoLabels  = [];
                newROI      = [];
                
            % If the current frame has an ROI, record position of ROI and
            % increment index, there are no automated ROIs.
            elseif (algObj.CurrentTime == algObj.InitialLabels.Time(algObj.Idx)) %at ROI draw time
                
                newROI = algObj.InitialLabels.Position(algObj.Idx, :);
                
                autoLabels = [];
                if algObj.IsForwardAutomation
                    algObj.Idx = algObj.Idx +  1;
                else
                    algObj.Idx = algObj.Idx -  1;
                end
            
            % Update the position of the ROI label and add an automated ROI
            % label
            else
                if algObj.IsForwardAutomation
                    idx = algObj.Idx - 1;
                    if(algObj.ROIType == labelType.Rectangle)
                        newROI = algObj.prevROI + ...
                            [algObj.delX(idx) algObj.delY(idx) algObj.delW(idx) algObj.delH(idx)]; 
                    elseif(algObj.ROIType == labelType.RotatedRectangle)
                        newROI = algObj.prevROI + ...
                            [algObj.delX(idx) algObj.delY(idx) algObj.delW(idx) ...
                            algObj.delH(idx) algObj.delA(idx)];
                        if newROI(5) <= -180
                            newROI(5) = newROI(5) + 360;
                        elseif newROI(5) > 180
                            newROI(5) = newROI(5) - 360;
                        end
                    else
                        newROI = algObj.prevROI + ...
                            [algObj.delX(idx) algObj.delY(idx) algObj.delW(idx) algObj.delH(idx) ...
                            algObj.delX2(idx) algObj.delY2(idx) algObj.delW2(idx) algObj.delH2(idx)]; 
                    end
                else
                    idx = algObj.Idx;
                    if(algObj.ROIType == labelType.Rectangle)
                        newROI = algObj.prevROI - ...
                            [algObj.delX(idx) algObj.delY(idx) algObj.delW(idx) algObj.delH(idx)];
                    elseif(algObj.ROIType == labelType.RotatedRectangle)
                        newROI = algObj.prevROI - ...
                            [algObj.delX(idx) algObj.delY(idx) algObj.delW(idx) ...
                            algObj.delH(idx) algObj.delA(idx)];
                        if newROI(5) <= -180
                            newROI(5) = newROI(5) + 360;
                        elseif newROI(5) > 180
                            newROI(5) = newROI(5) - 360;
                        end
                    else
                        newROI = algObj.prevROI - ...
                            [algObj.delX(idx) algObj.delY(idx) algObj.delW(idx) algObj.delH(idx) ...
                            algObj.delX2(idx) algObj.delY2(idx) algObj.delW2(idx) algObj.delH2(idx)]; 
                    end                    
                end                
                
                autoLabels.Name     = algObj.ROIName;
                autoLabels.Type     = algObj.ROIType;
                autoLabels.Position = newROI;
                
                if(algObj.hasAttributes) 
                    autoLabels.Attributes = algObj.AttribData{idx};
                end
            end
                            
            % Update for next frame
            algObj.prevROI = newROI;            
        end

        function terminate(algObj)
            
            % reset index           
            if algObj.IsForwardAutomation
                algObj.Idx = 1;
            else
                algObj.Idx = size(algObj.InitialLabels,1);
            end            
        end
    end
end

