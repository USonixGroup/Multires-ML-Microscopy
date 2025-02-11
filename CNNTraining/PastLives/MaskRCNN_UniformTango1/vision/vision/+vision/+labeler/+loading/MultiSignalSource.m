%MultiSignalSource Interface for loading signal data to Ground Truth Labeler
%
%   MultiSignalSource specifies the interface for loading custom signals
%   from a data source in the Ground Truth Labeler app. The loaded signals
%   can then be labeled in the Ground Truth Labeler. Use of the Ground
%   Truth Labeler requires that you have the Automated Driving Toolbox(TM).
%
%   The MultiSignalSource class is an abstract class that defines the
%   signature for methods and properties that the Ground Truth Labeler app
%   uses for loading signal data. To define a custom data source loader,
%   you must construct a class that inherits from the
%   vision.labeler.loading.MultiSignalSource class. The class must specify
%   the following key components:
%
%   Loading a source 
%   ----------------
%   1. The Ground Truth Labeler has a loading dialog to load signals
%      from data sources. This class provides an interface to customize a
%      uipanel on the Loading dialog. This panel can be customized to
%      obtain all the information to load the source and make the object
%      ready for reading frames. For example, to load image data, this
%      uipanel may contain an edit box to specify the folder containing
%      images, and a drop-down to choose the image format (e.g. PNG, JPG)
%      to accept.
% 
%   2. An interface to load the source based on the information in
%      the loading dialog.
%
%   Reading frames
%   --------------
%   The class provides an interface to read frames. These frames are
%   rendered in the app for labeling.
%
%   In order to define a custom class to load a data source to the Ground
%   Truth Labeler app, follow these steps:
%
%   1. Construct a class that inherits from
%      vision.labeler.loading.MultiSignalSource class. Create this class in
%      +vision/+labeler/+loading folder within a folder that is on the
%      MATLAB path.
%
%   2. Define the abstract, constant properties.
%   
%       Name        - A string scalar specifying the type of the source.
%       Description - A string scalar describing the class.
%
%   3. Define the abstract methods to customize the uipanel on loading
%      dialog.
%
%       customizeLoadPanel - Method to customize the load panel
%       getLoadPanelData   - Method to obtain the data entered in the panel
%
%       Additionally you can use the following method to check how the
%       Loading dialog will look in the app, while developing these
%       methods.
%
%       loadPanelChecker   - Method to display the load panel outside the
%                            app.
%
%   4. Define the abstract method to load the source and populate the
%      information required by the app in the properties of the class.
%
%      loadSource - Method to load data source. This method is expected to
%                   populate the following properties:
%
%                   SignalName   - String identifiers for each signal in a
%                                  data source
%                   SignalType   - An array with each entry of type
%                                  vision.labeler.loading.SignalType
%                   Timestamp    - A vector or cell array of timestamps
%                                  for each signal
%                   SourceName   - The name of the data source 
%                   SourceParams - A struct containing fields with
%                                  information required to load the source.
%                                  The fields of the struct are custom to
%                                  the data sources.
%
%   5. Define the abstract method to read a frame from a signal
%
%      readFrame - A method to read frame from a particular signal in the
%                  source. The index to a particular timestamp in the
%                  Timestamp property is passed into this method
%   
%
%   A few examples of defining a custom class can be found here,
%
%                        Class                      | Data Source 
%   ----------------------------------------------------------------------- 
%   vision.labeler.loading.VideoSource              | Video file
%   vision.labeler.loading.ImageSequenceSource      | Image Sequence folder
%   vision.labeler.loading.VelodyneLidarSource      | Velodyne PCAP file
%   vision.labeler.loading.RosbagSource             | Rosbag file
%   vision.labeler.loading.PointCloudSequenceSource | Point cloud sequence folder
%   vision.labeler.loading.CustomImageSource        | Custom image format
%
%   See also groundTruthLabeler, groundTruthMultisignal

% Copyright 2019-2020 The MathWorks, Inc.

classdef MultiSignalSource < handle & matlab.mixin.Heterogeneous
    
    properties (Abstract)
        %Name A string scalar specifying the type of the source that will
        %be loaded by this class
        Name
        
        %Description A string scalar describing the functionality of the
        %class
        Description
    end
    
    
    %----------------------------------------------------------------------
    % Properties with information about the source
    %----------------------------------------------------------------------
    properties (GetAccess = public, SetAccess = protected)
        %SourceName A string specifying the name of the source. In most
        %cases, this is the name of the file from which the signals are
        %being read from.
        SourceName string
        
        %SourceParams A struct which is the output of the getLoadPanelData
        %method. The fields of the struct contains values required to load
        %the signal using the loadSource method.
        SourceParams          
    end
    
    %----------------------------------------------------------------------
    % Properties with information about the signals
    %----------------------------------------------------------------------
    properties (GetAccess = public, SetAccess = protected)
        %SignalName A string array containing a string identifier for each
        %signal
        SignalName string
        
        %SignalType An array of vision.labeler.loading.SignalType, for each
        %signal
        SignalType
        
        %Timestamp A cell array of vectors containing the
        %timestamps for each signal. Timestamps are of type duration or
        %datetime.
        Timestamp {}
    end

    properties (Dependent)
        %NumSignals Number of signals that can be read from. The number
        %should be equal to the number of elements in the SignalName array
        NumSignals
    end
    
    properties (Hidden)
        %IsLoaded A flag indicating that the loadSource call when loading a
        %saved object was successful
        IsLoaded
    end

    %----------------------------------------------------------------------
    % Methods to customize the uipanel in the Loading Dialog of Ground Truth
    % Labeler app
    %----------------------------------------------------------------------
    methods (Abstract)

        %customizeLoadPanel
        %   customizeLoadPanel is invoked when a MultiSignalSource is
        %   selected from the 'Source Type' dropdown menu of the loading
        %   dialog of the Ground Truth Labeler app.
        customizeLoadPanel(this, panel)
        
        %getLoadPanelData 
        %   getLoadPanelData is invoked when 'Add Source' button in the
        %   loading dialog is clicked in the Ground Truth Labeler App. This
        %   method obtains the data from the load panel and provides the
        %   information to load the source. sourceName is a string
        %   capturing the name of the source and sourceParams is a struct
        %   with fields containing parameters required to load the source.
        %   Both these outputs are passed to loadSource method
        [sourceName, sourceParams] = getLoadPanelData(this)
    end
    
    %----------------------------------------------------------------------
    % Method to load source and populate all the properties that will be
    % queried by the app
    %----------------------------------------------------------------------    
    methods (Abstract)
        %loadSource
        %   loadSource is invoked when 'Add Source' button in the loading
        %   dialog is clicked and getLoadPanelData is executed
        %   successfully. Additionally, load source is also invoked when
        %   the MultiSignalSource object is loaded into the workspace. When
        %   a MultiSignalSource object is loaded, it is expected that it
        %   has the right SourceName and SourceParams to be able to load
        %   the source and read from it.
        loadSource(sourceName, sourceParams)
        
    end
    
    %----------------------------------------------------------------------
    % Method to read frame
    %----------------------------------------------------------------------  
    methods (Abstract)
        %readFrame
        %   Read frame for a signal by timestamp index. The index should be
        %   in bounds of the length of the timestamps for a signal
        frame  = readFrame(this, signalName, index)
    end
    
    methods (Static)
        function loadPanelChecker()
            %loadPanelChecker 
            %   Check the load panel in signal loading dialog of Ground
            %   Truth Labeler. loadPanelChecker brings up a dialog similiar
            %   to the loading dialog when you click on 'Open -> Add
            %   Signal' in Ground Truth Labeler app. This method can be
            %   used to see how the load panel is populated by
            %   customizeLoadPanel method
            
            import vision.internal.videoLabeler.tool.signalLoading.view.*
            display = SignalLoadViewMulti();
            display.open([]);
        end
    end
    
    methods
        function numSignals = get.NumSignals(this)
            numSignals = numel(string(this.SignalName));
        end
    end
    
    methods(Hidden)
        function setTimestamps(this, timestamps)
            
            if ~iscell(timestamps)
                timestamps = {timestamps};
            end
            
            for idx = 1:numel(timestamps)
                if size(timestamps{idx}, 2)  ~= 1
                    timestamps{idx} = timestamps{idx}';
                end
            end
            
            this.Timestamp = timestamps;
        end
        
        function that = saveobj(this)
            that.SourceName = this.SourceName;
            that.SourceParams = this.SourceParams;
            that.SignalName = this.SignalName;
            that.SignalType = this.SignalType;
            that.Timestamp = this.Timestamp;
            that.SourceClass = class(this);
        end
        
        function fixSourceLoading(this, alternateSourceName, alternatePaths)
            try
                signalNames = this.SignalName;
                signalTypes = this.SignalType;
                timestamps = this.Timestamp;
                this.loadSource(alternateSourceName, this.SourceParams);
                this.IsLoaded = true;
                
                this.SignalName = signalNames;
                this.SignalType = signalTypes;
                this.Timestamp = timestamps;
            catch
                this.IsLoaded = false;
            end
        end
        
        function sourceParams = fixSourceParams(this, alternatePaths)
            sourceParams = this.SourceParams;
        end
        
        function validName = makeValidName(~, name, prefix)
            if ~isvarname(name)
                validName = matlab.lang.makeValidName(prefix + name);
            else
                validName = name;
            end
        end

        function isPending = checkPendingChanges(this)
            %checkPendingChanges Checks for pending changes in load panel
            %   checkPendingChanges returns a boolean flag isPending, if
            %   there are any pending changes in the load panel.
            isPending = false;
        end        

        function info = getFileReaderInfo(this)
            %getFileReaderInfo Get file reader information specific to this source.
            % An empty stub method. See VideoSource for an implementation.
            % Used by writeVideoScenes.
            info = [];
        end
    end
    
    methods (Hidden, Static)
        function this = loadobj(that)
            this = eval(that.SourceClass);
            
            this.SourceName = that.SourceName;
            this.SourceParams = that.SourceParams;
            
            try
                this.loadSource(this.SourceName, this.SourceParams);
                this.IsLoaded = true;
            catch
                this.IsLoaded = false;
            end
            
            this.SignalName = that.SignalName;
            this.SignalType = that.SignalType;
            this.Timestamp = that.Timestamp;
        end
    end 
end
