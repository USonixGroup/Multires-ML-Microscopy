% Computer Vision Toolbox
% Version 24.2 (R2024b) 12-Jul-2024
%
% Video Display
%   vision.DeployableVideoPlayer  - Display video (Windows and Linux)
%   vision.VideoPlayer            - Play video or display image
%   implay                        - View video from files, the MATLAB workspace, or Simulink signals
%
% Video File I/O
%   VideoReader                   - Read video frames from video file
%   vision.VideoFileWriter        - Write video frames and audio samples to video file
%   vision.BinaryFileReader       - Read binary video data from files
%   vision.BinaryFileWriter       - Write binary video data to files
%
% Feature Detection, Extraction and Matching
%   detectHarrisFeatures          - Find corners using the Harris-Stephens algorithm
%   detectMinEigenFeatures        - Find corners using the minimum eigenvalue algorithm
%   detectFASTFeatures            - Find corners using the FAST algorithm
%   detectSURFFeatures            - Find SURF features
%   detectSIFTFeatures            - Find SIFT features
%   detectKAZEFeatures            - Find KAZE features
%   detectORBFeatures             - Find ORB features
%   detectMSERFeatures            - Find MSER features
%   detectBRISKFeatures           - Find BRISK features
%   extractFeatures               - Extract feature vectors from image
%   extractHOGFeatures            - Extract HOG features
%   extractLBPFeatures            - Extract LBP features
%   matchFeatures                 - Find matching features
%   matchFeaturesInRadius         - Find matching features within a radius
%   showMatchedFeatures           - Display corresponding feature points
%   cornerPoints                  - Object for storing corner points
%   SURFPoints                    - Object for storing SURF interest points
%   SIFTPoints                    - Object for storing SIFT interest points
%   KAZEPoints                    - object for storing KAZE interest points
%   ORBPoints                     - Object for storing ORB interest points
%   MSERRegions                   - Object for storing MSER regions
%   BRISKPoints                   - Object for storing BRISK interest points
%   binaryFeatures                - Object for storing binary feature vectors
%
% Object Detection and Recognition
%   ocr                                 - Recognize text using Optical Character Recognition
%   ocrText                             - Object for storing OCR results
%   ocrTrainingData                     - Create training data for OCR from groundTruth
%   ocrTrainingOptions                  - Options for training an OCR model
%   trainOCR                            - Train an OCR model to recognize image text
%   evaluateOCR                         - Evaluate OCR results against ground truth
%   quantizeOCR                         - Quantize OCR model
%   detectTextCRAFT                     - Detect text in an image
%   readAprilTag                        - Read and estimate pose for AprilTags in images
%   readArucoMarker                     - Read and estimate pose for ArUco markers in images
%   generateArucoMarker                 - Generate ArUco marker images
%   readBarcode                         - Read 1D and 2D barcodes
%   vision.CascadeObjectDetector        - Detect objects using the Viola-Jones algorithm
%   vision.PeopleDetector               - Detect upright people using HOG features
%   peopleDetectorACF                   - Detect upright people using ACF features
%   peopleDetector                      - Detect people using RTMDet object detector 
%   acfObjectDetector                   - Detect objects using ACF features
%   rcnnObjectDetector                  - Detect objects using R-CNN deep learning detector
%   fastRCNNObjectDetector              - Detect objects using Fast R-CNN deep learning detector
%   fasterRCNNObjectDetector            - Detect objects using Faster R-CNN deep learning detector
%   yolov2ObjectDetector                - Detect objects using YOLO v2 deep learning detector
%   yolov3ObjectDetector                - Detect objects using YOLO v3 deep learning detector
%   yolov4ObjectDetector                - Detect objects using YOLO v4 deep learning detector
%   rtmdetObjectDetector                - Detect objects using RTMDet deep learning detector
%   hrnetObjectKeypointDetector         - Detect Object Keypoints using HRNet deep learning detector
%   trainCascadeObjectDetector          - Train a model for a cascade object detector
%   trainACFObjectDetector              - Train a model for an ACF object detector
%   trainRCNNObjectDetector             - Train an R-CNN deep learning object detector
%   trainFastRCNNObjectDetector         - Train a Fast R-CNN deep learning object detector
%   trainFasterRCNNObjectDetector       - Train a Faster R-CNN deep learning object detector
%   trainYOLOv2ObjectDetector           - Train a YOLO v2 deep learning object detector
%   trainYOLOv3ObjectDetector           - Train a YOLO v3 deep learning object detector
%   trainYOLOv4ObjectDetector           - Train a YOLO v4 deep learning object detector
%   trainSSDObjectDetector              - Train an SSD deep learning object detector
%   trainHRNetObjectKeypointDetector    - Train a HRNet deep learning object Keypoint detector
%   roiMaxPooling2dLayer                - ROI max pooling layer for Fast and Faster R-CNN
%   rcnnBoxRegressionLayer              - Box regression layer for Fast and Faster R-CNN 
%   rpnSoftmaxLayer                     - Softmax layer for region proposal network (RPN)
%   rpnClassificationLayer              - Classification layer for region proposal network (RPN)
%   regionProposalLayer                 - Region proposal layer for Faster R-CNN
%   roiAlignLayer                       - ROI align layer for Mask R-CNN
%   roiInputLayer                       - ROI input layer for Fast R-CNN
%   fasterRCNNLayers                    - Create a Faster R-CNN object detection network
%   yolov2Layers                        - Create a YOLO v2 object detection network
%   yolov2TransformLayer                - YOLO v2 Transform layer
%   yolov2OutputLayer                   - YOLO v2 Output layer
%   evaluateDetectionPrecision          - Evaluate the precision metric for object detection
%   evaluateDetectionMissRate           - Evaluate the miss rate metric for object detection
%   evaluateDetectionAOS                - Evaluate the average orientation similarity metric for object detection
%   estimateAnchorBoxes                 - Estimate anchor boxes for deep learning object detectors
%   balanceBoxLabels                    - Balance bounding box labels for object detection
%   blockLocationSet                    - List of block locations in large images
%   boxLabelDatastore                   - Datastore for bounding box labels
%   selectStrongestBbox                 - Select strongest bounding boxes from overlapping clusters
%   selectStrongestBboxMulticlass       - Select strongest multiclass bounding boxes from overlapping clusters
%   bagOfFeatures                       - Create bag of visual features
%   trainImageCategoryClassifier        - Train bag of features based image category classifier
%   imageCategoryClassifier             - Predict image category
%   indexImages                         - Create an index for image search
%   retrieveImages                      - Search for similar images
%   invertedImageIndex                  - Search index that maps visual words to images
%   evaluateImageRetrieval              - Evaluate image search results
%   anchorBoxLayer                      - Stores anchor boxes for object detection
%   focalLossLayer                      - Focal cross-entropy classification layer for imbalanced classes
%   ssdMergeLayer                       - SSD Merge layer
%   ssdLayers                           - Create a SSD object detection network
%   ssdObjectDetector                   - Detect objects using an SSD deep learning detector
%   focalCrossEntropy                   - Focal cross-entropy loss for imbalanced classes
%   patchEmbeddingLayer                 - Extract patches from an image and map each patch to a vector
%   slowFastVideoClassifier             - SlowFast video classifier
%   r2plus1dVideoClassifier             - R(2+1)D video classifier
%   inflated3dVideoClassifier           - Inflated-3D video classifier
%   reidentificationNetwork             - Create an object reidentification network
%   trainReidentificationNetwork        - Train a reidentification network
%   evaluateReidentificationNetwork     - Evaluate a reidentification network
%
% Semantic Segmentation
%   semanticseg                         - Semantic image segmentation using deep learning
%   evaluateSemanticSegmentation        - Evaluate semantic segmentation data set against ground truth
%   deeplabv3plus                       - Create DeepLab v3+ network for semantic segmentation using deep learning
%   unet                                - Create U-Net network for semantic segmentation using deep learning
%   unet3d                              - Create U-Net network for 3-D semantic segmentation using deep learning
%   labeloverlay                        - Overlay semantic segmentation results on an image
%   volshow                             - Display semantic segmentation results on a volume
%   pixelLabelDatastore                 - Create a PixelLabelDatastore to work with collections of pixel label data
%   pixelClassificationLayer            - Pixel classification layer for semantic segmentation
%   dicePixelClassificationLayer        - Pixel classification layer using generalized dice loss for semantic segmentation
%   balancePixelLabels                  - Balance pixel labels by oversampling block locations in large images
%   segmentationConfusionMatrix         - Confusion matrix of semantic segmentation
%   generalizedDice                     - Generalized dice similarity metric for semantic segmentation
%
% Ground Truth Labeling
%   imageLabeler                        - App for labeling ground truth data in a collection of images
%   videoLabeler                        - App for labeling ground truth data in video or image sequence
%   groundTruth                         - Object for storing ground truth labels
%   groundTruthDataSource               - Create ground truth data source
%   labelDefinitionCreator              - Define labels, sublabels and attributes
%   labelType                           - Enumeration of ground truth label types
%   attributeType                       - Enumeration of ground truth attribute types
%   vision.labeler.AutomationAlgorithm  - Interface for automated labeling
%   vision.labeler.mixin.Temporal       - Mixin for adding temporal context to automated labeling
%   objectDetectorTrainingData          - Create training data for an object detector from groundTruth
%   pixelLabelTrainingData              - Create training data for semantic segmentation from groundTruth
%   groundTruthToOpenLabel              - Convert groundTruth object to OpenLABEL JSON file
%   groundTruthFromOpenLabel            - Convert OpenLABEL JSON file to groundTruth object
%
% Motion Analysis and Tracking
%   assignDetectionsToTracks      - Assign detections to tracks for multi-object tracking
%   vision.BlockMatcher           - Estimate motion between images or video frames
%   vision.ForegroundDetector     - Detect foreground using Gaussian Mixture Models
%   vision.HistogramBasedTracker  - Track object in video based on histogram
%   configureKalmanFilter         - Create a Kalman filter for object tracking
%   vision.KalmanFilter           - Kalman filter
%   opticalFlow                   - Object for storing optical flow
%   opticalFlowFarneback          - Estimate object velocities using Farneback algorithm
%   opticalFlowHS                 - Estimate object velocities using Horn-Schunck algorithm
%   opticalFlowLK                 - Estimate object velocities using Lucas-Kanade algorithm
%   opticalFlowLKDoG              - Estimate object velocities using modified Lucas-Kanade algorithm
%   opticalFlowRAFT               - Estimate object velocities using RAFT deep learning network
%   vision.PointTracker           - Track points in video using Kanade-Lucas-Tomasi (KLT) algorithm
%   vision.TemplateMatcher        - Locate template in image
%
% Camera Calibration
%   cameraCalibrator              - Single camera calibration app
%   stereoCameraCalibrator        - Stereo camera calibration app
%   estimateCameraParameters      - Calibrate a single camera or a stereo camera
%   estimateFisheyeParameters     - Calibrate a fisheye camera
%   estimateStereoBaseline        - Estimate baseline of a stereo camera
%   detectCheckerboardPoints      - Detect a checkerboard pattern in images
%   detectCircleGridPoints        - Detect a circle grid pattern in images
%   detectCharucoBoardPoints      - Detect a ChArUco board pattern in images
%   detectAprilGridPoints         - Detect an AprilGrid pattern in images
%   patternWorldPoints            - Generate world point locations of a calibration pattern
%   generateCharucoBoard          - Generate ChArUco board image
%   showExtrinsics                - Visualize extrinsic camera parameters
%   showReprojectionErrors        - Visualize calibration errors
%   cameraParameters              - Object for storing camera parameters
%   stereoParameters              - Object for storing parameters of a stereo camera system
%   fisheyeParameters             - Object for storing fisheye camera parameters
%   cameraIntrinsics              - Object for storing intrinsic camera parameters
%   fisheyeIntrinsics             - Object for storing intrinsic fisheye camera parameters
%   cameraCalibrationErrors       - Object for storing standard errors of estimated camera parameters
%   stereoCalibrationErrors       - Object for storing standard errors of estimated stereo parameters
%   fisheyeCalibrationErrors      - Object for storing standard errors of estimated fisheye camera parameters
%   undistortImage                - Correct image for lens distortion
%   undistortPoints               - Correct point coordinates for lens distortion
%   undistortFisheyeImage         - Correct fisheye image for lens distortion
%   undistortFisheyePoints        - Correct point coordinates for fisheye lens distortion
%   img2world2d                   - Determine world coordinates of image points
%   world2img                     - Project world points into the image
%   cameraIntrinsicsFromOpenCV    - Convert camera intrinsics from OpenCV to MATLAB
%   cameraIntrinsicsToOpenCV      - Convert camera intrinsics to OpenCV from MATLAB
%   stereoParametersFromOpenCV    - Convert stereo camera parameters from OpenCV to MATLAB
%   stereoParametersToOpenCV      - Convert stereo camera parameters to OpenCV from MATLAB
%
% Stereo Vision
%   disparityBM                       - Compute disparity map using Block Matching
%   disparitySGM                      - Compute disparity map using Semi-Global Matching
%   epipolarLine                      - Compute epipolar lines for stereo images
%   estimateStereoRectification       - Uncalibrated stereo rectification
%   isEpipoleInImage                  - Determine whether the epipole is inside the image
%   lineToBorderPoints                - Compute the intersection points of lines and image border
%   reconstructScene                  - Reconstructs a 3-D scene from a disparity map
%   rectifyStereoImages               - Rectifies a pair of stereo images
%   stereoAnaglyph                    - Create a red-cyan anaglyph from a stereo pair of images
%
% Multiple View Geometry
%   bundleAdjustment             - Refine camera poses and 3-D points
%   bundleAdjustmentMotion       - Refine a camera pose using motion-only bundle adjustment
%   bundleAdjustmentStructure    - Refine 3-D points using structure-only bundle adjustment
%   cameraProjection             - Compute camera projection matrix
%   estimateCameraProjection     - Estimate camera projection matrix
%   pose2extr                    - Convert camera pose to extrinsics
%   estimateEssentialMatrix      - Estimate the essential matrix
%   estimateFundamentalMatrix    - Estimate the fundamental matrix
%   estworldpose                 - Estimate camera pose from 3-D to 2-D point correspondences
%   estimateExtrinsics           - Estimate location of a calibrated camera
%   extr2pose                    - Convert extrinsics into camera pose
%   plotCamera                   - Plot a camera in 3-D coordinates
%   pointTrack                   - Object for storing matching points from multiple views
%   estrelpose                   - Estimate relative up-to-scale pose of calibrated camera
%   rotmat2vec3d                 - Convert a 3-D rotation matrix into a rotation vector
%   rotvec2mat3d                 - Convert a 3-D rotation vector into a rotation matrix
%   triangulate                  - Find 3-D locations of matching points between pairs of images
%   triangulateMultiview         - Triangulate 3-D locations of points matched across multiple views
%   imageviewset                 - Object for managing image data for structure-from-motion, visual odometry and visual SLAM
%   imageviewset/createPoseGraph - Create pose graph from an image view set
%   imageviewset/optimizePoses   - Optimize image view set poses
%   worldpointset                - Object for managing 3-D to 2-D point correspondences
%   monovslam                    - Perform monocular visual SLAM
%   stereovslam                  - Perform visual SLAM with a stereo camera
%   rgbdvslam                    - Perform visual SLAM with an RGB-D camera
%   compareTrajectories          - Compare estimated trajectory against ground truth
%   trajectoryErrorMetrics       - Object for storing trajectory accuracy metrics
%
% Point Cloud Processing
%   pointCloud                   - Object for storing a 3-D point cloud
%   velodyneFileReader           - Creates a Velodyne PCAP file reader object
%   pcbin                        - Spatially bin points in a point cloud
%   pcdenoise                    - Remove noise from a 3-D point cloud
%   pcdownsample                 - Downsample a 3-D point cloud
%   pcsegdist                    - Segment a point cloud into clusters based on Euclidean distance
%   pcnormals                    - Estimate normal vectors for a point cloud
%   pccat                        - Concatenate 3-D point clouds
%   pcmerge                      - Merge two 3-D point clouds
%   pcregistercpd                - Register two point clouds using CPD algorithm
%   pcregistericp                - Register two point clouds using ICP algorithm
%   pcregisterndt                - Register two point clouds using NDT algorithm
%   pcregistercorr               - Register two point clouds using phase correlation algorithm
%   pcalign                      - Align an array of point clouds
%   pctransform                  - Transform a 3-D point cloud
%   pcfitplane                   - Fit plane to a 3-D point cloud
%   pcfitsphere                  - Fit sphere to a 3-D point cloud
%   pcfitcylinder                - Fit cylinder to a 3-D point cloud
%   pcviewer                     - Visualize and inspect point cloud
%   pcshow                       - Plot 3-D point cloud
%   pcshowpair                   - Visualize differences between point clouds
%   pcplayer                     - Player for visualizing streaming 3-D point cloud data
%   pcread                       - Read a 3-D point cloud from PLY file
%   pcwrite                      - Write a 3-D point cloud to PLY file
%   pcfromkinect                 - Get point cloud from Kinect for Windows
%   pcfromdepth                  - Convert depth image to point cloud
%   planeModel                   - Object for storing a parametric plane model
%   sphereModel                  - Object for storing a parametric sphere model
%   cylinderModel                - Object for storing a parametric cylinder model
%   segmentGroundFromLidarData   - Segment ground points from organized Lidar data
%   segmentLidarData             - Segment organized 3-D range data into clusters
%   pcviewset                    - Object for managing data for point cloud-based visual odometry and SLAM
%   pcviewset/createPoseGraph    - Create pose graph from a point cloud view set
%   pcviewset/optimizePoses      - Optimize point cloud view set poses
%   scanContextDescriptor        - Extract scan context descriptor from a point cloud
%   scanContextDistance          - Distance between scan context descriptors
%   scanContextLoopDetector      - Detect loop closures using scan context descriptors
%   pcmapndt                     - Create a localization map using Normal Distributions Transform
%   pcmapndt/findPose            - Localize a point cloud using Normal Distributions Transform
%
% Rotation Utilities
%   quaternion                   - Quaternion arrays from parts or other rotation representations
%   quaternion/rotmat            - Convert quaternions to rotation matrices
%   quaternion/euler             - Convert quaternions to Euler or Tait-Bryan angles (radians)
%   quaternion/eulerd            - Convert quaternions to Euler or Tait-Bryan angles (degrees)
%   quaternion/rotvec            - Convert quaternions to rotation vectors (radians)
%   quaternion/rotvecd           - Convert quaternions to rotation vectors (degrees)
%   quaternion/compact           - Arrays from quaternions
%   quaternion/parts             - Extract four parts of a quaternion
%   quaternion/rotateframe       - Rotate coordinate frame using quaternions
%   quaternion/rotatepoint       - Rotate 3D points using quaternions
%   quaternion/norm              - Quaternion norm
%   quaternion/normalize         - Convert to a unit quaternion
%   quaternion/dist              - Distance between two quaternions
%   quaternion/slerp             - Spherical linear interpolation of quaternions
%   quaternion/meanrot           - Average rotation of quaternions
%   randrot                      - Uniformly distributed random rotations
%
% Enhancement
%   vision.Deinterlacer           - Remove motion artifacts by deinterlacing input video signal
%
% Conversions
%   vision.ChromaResampler        - Downsample or upsample chrominance components of images
%   vision.GammaCorrector         - Gamma correction
%
% Filtering
%   isfilterseparable             - Check filter separability
%   integralImage                 - Compute integral image
%   integralFilter                - Filter using integral image
%   integralKernel                - Define filter for use with integral images
%
% Geometric Transformations
%   estgeotform2d                 - Estimate 2-D geometric transformation from matching point pairs
%   estgeotform3d                 - Estimate 3-D geometric transformation from matching point pairs
%   affinetform2d                 - 2-D affine geometric transformation
%   affinetform3d                 - 3-D affine geometric transformation
%   projtform2d                   - 2-D projective geometric transformation
%   rigidtform2d                  - 2-D rigid geometric transformation
%   rigidtform3d                  - 3-D rigid geometric transformation
%   simtform2d                    - 2-D similarity geometric transform
%   simtform3d                    - 3-D similarity geometric transform
%
% Statistics
%   vision.BlobAnalysis         - Properties of connected regions
%
% Text and Graphics
%   insertObjectAnnotation      - Insert annotation in image or video stream
%   insertObjectKeypoints       - Insert object keypoints in image or video stream.
%   insertObjectMask            - Insert masks in image or video stream
%   insertMarker                - Insert markers in image or video stream
%   insertShape                 - Insert shapes in image or video stream
%   insertText                  - Insert text in image or video stream
%   listTrueTypeFonts           - List available TrueType fonts
%   showShape                   - Show shapes in an image, video, or point cloud
%
% Utilities
%   bbox2points                 - Convert a rectangle into a list of points
%   bboxcrop                    - Crop bounding boxes
%   bboxOverlapRatio            - Compute bounding box overlap ratio
%   bboxPrecisionRecall         - Compute bounding box precision and recall against ground truth
%   bboxresize                  - Resize bounding boxes
%   bboxwarp                    - Apply geometric transformation to bounding boxes
%   cuboid2img                  - Project cuboids from world to image
%   ransac                      - Fit a model to noisy data 
%   fitPolynomialRANSAC         - Fit polynomial to [x,y] data using RANSAC
%   visionSupportPackages       - Launches the support package installer
%
% Examples
%   visiondemos                 - Index of Computer Vision Toolbox examples
%
% Simulink functionality
%   <a href="matlab:visionlib">visionlib</a>                   - Open Computer Vision Toolbox Simulink library
%
% See also images

%   Copyright 2004-2024 The MathWorks, Inc.
