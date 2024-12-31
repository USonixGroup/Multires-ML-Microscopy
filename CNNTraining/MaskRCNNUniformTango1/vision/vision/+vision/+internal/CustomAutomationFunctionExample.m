% This function defines a template for creating a custom automation
% algorithm for use in the Image Labeler, Video Labeler, and Ground Truth
% Labeler apps.
%
% To see an automation function example, open the following file:
%
%  >> edit exampleAutomationAlgorithmFunction.m
%
% To use your algorithm within the labeling apps, follow the steps outlined
% below:
%
%   1. On the app toolstrip, select Select Algorithm > Add Whole Image
%      Algorithm > Import Algorithm
%   2. Choose your function using the file browser.

% Copyright 2022 The MathWorks, Inc.
 
function autoLabels = myAutomationFunction(I)
% Your automation function is invoked on each image, I, chosen for
% automation in the labeler app. Implement your automation algorithm below
% and return automated labels in autoLabels. autoLabels must be a
% categorical matrix for automating pixel labeling. Otherwise, autoLabels
% must be a struct or table with fields Type, Name, Position and optionally
% Attributes. The Attributes field is valid only when labels with
% attributes are defined in the app.
%
% The fields of the autoLabels struct array are described below:
%    
%     Type        A labelType enumeration that defines the type of label.
%                 Type can have values Rectangle, Line, Polygon, Projected
%                 cuboid, or Scene.
% 
%     Name        A character vector specifying a label name. Only
%                 existing label names previously defined in the
%                 labeler app can be used.
% 
%     Position    Positions of the labels. The type of label determines
%                 the format of the position data. For more information,
%                 see the doc page for vision.labeler.AutomationAlgorithmFunction.
% 
%     Attributes  An array of structs representing the attributes
%                 contained by the automated labels. Each attribute
%                 is specified as a field of the struct, with the
%                 name of the field representing the name of the
%                 attribute and the value of the field representing
%                 the value of the attribute.
%
% Below is an example of how to specify an autoLabels structure for an
% algorithm that detects a car, finds a lane, and classifies the
% scene as sunny.
%
% % Rectangle labeled 'Car' positioned with top-left at (20,20)
% % with width and height equal to 50.
% autoLabels(1).Name      = 'Car';
% autoLabels(1).Type      = labelType('Rectangle');
% autoLabels(1).Position  = [20 20 50 50];
%
% % Line labeled 'LaneMarker' with 3 points.
% autoLabels(2).Name      = 'LaneMarker';
% autoLabels(2).Type      = labelType('Line');
% autoLabels(2).Position  = [100 100; 100 110; 110 120];
%
% % Scene labeled 'Sunny'
% autoLabels(3).Name      = 'Sunny';
% autoLabels(3).Type      = labelType('Scene');
% autoLabels(3).Position  = true;
 
%--------------------------------------------------------
% Place your algorithm code here
%--------------------------------------------------------