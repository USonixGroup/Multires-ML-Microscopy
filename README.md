# Multiresolution Machine Learning for Segmentation and Characterisation of Microscopy Images
Microscopy is fundamental in medical research and practice, allowing observation of cellular states and responses to various stimuli. Analyzing these microscopy images is therefore a vital part of medical research. This process typically involves identifying cell boundaries to extract and analyze cellular parameters like size and shape, and to track their changes over time.

Multires-ML-Microscopy is an open-source library written in MATLAB, featurin a fully integrated pipeline for the segmentation, analysis, and tracking of cells in microscopy images. The library addresses a gap in biomedical imaging by combining advanced machine learning techniques with accessible design and customisability. Using wavelet-based pre-processing enhanced image quality under noise, a Mask R-CNN segmentation model with a custom EfficientNet-inspired backbone delivered high-precision cell masks, and the tracking module improved temporal consistency in live-cell imaging. This functionality is encapsulated within a user-friendly GUI, making the tool accessible to researchers without coding expertise.

Designed for use in biological and medical research environments, the pipeline enables high-throughput and reproducible analysis of cellular morphology, supporting tasks such as drug testing, disease modelling, and cell behaviour studies.

## HSMR25 Submission & DOI
This repository accompanies our submission to The 17th Hamlyn Symposium on Medical Robotics (HSMR25), held at the Royal Geographical Society on 27th June 2025. Our work is presented as part of the session "Healing Through Collaboration: Open-Source Software in Surgical, Biomedical and AI Technologies", highlighting the importance of community-driven development in medical imaging. The full submission is archived with Zenodo and can be accessed via DOI: [10.5281/zenodo.15727799](https://doi.org/10.5281/zenodo.15727799).

## Getting help
Enquiries about the library and questions should be asked on the discussion page of the GitHub or directed to the team of developers on the GitHub or via email:nikhil.dhulashia.21@ucl.ac.uk, or yigit.dogan.21@ucl.ac.uk

## Citation
If you use Multires-ML-Microscopy in your work, please cite it as follows:

**APA**
```
N Dhulashia, Y Doğan, D Rajpara1, X Shi, L Casamitjana Ortega, R Haqshenas (2025), Multires-ML-Microscopy: Multiresolution Machine Learning based Segmentation and Characterisation of Microscopy Images, https://github.com/TechAvi-eng/Multires-ML-Microscopy
```

**BibTeX**
```
@software{optimuslib,
author = {N Dhulashia, Y Doğan, D Rajpara, X Shi1, L Casamitjana Ortega, R Haqshenas },
title = {Multires-ML-Microscopy: Multiresolution Machine Learning based Segmentation and Characterisation of Microscopy Images},
url = {https://github.com/TechAvi-eng/Multires-ML-Microscopy},
version = {0.1}
}
```
## License
Multires-ML-Microscopy is licensed under an MIT licence.
Copyright (c) 2025.

## UPDATES: Addition of Sobel + Watershed segmentation algorithm 
- A segmentSobelWatershed.m file was added with the Sobel and marker-controlled Watershed cell segmenter, which does NOT use Machine Learning. [Link to the standalone MATLAB code](https://github.com/rustamtoshov23/Sobel-Watershed-segmenter).
- The app code was modified, old versions (including the installer and the .mlapp file) were deleted, a new Multires_ML_Microscopy.m file was uploaded. Updates:
  - Sobel + Watershed input panel was added in Settings
  - All button images now show up
  - smartResize now takes place ONLY for visualisation. All segmentation algorithms work on full image resolution, however the final image with masks is then resized to fit into the designed Display window. When exporting results, those will be at the original image resolution as well!
  - The code is presented in .m format, rather than .mlapp as it was previously. The only change this creates is that we lose drag-and-drop functionality if we ever want to edit the GUI in App Designer. GUI will have to be hardcoded. Nevertheless, the app works in the same exact way as it used to.
  - Cleaned up, got rid of dead code. 
- MATLAB Toolbox dependencies are specified (everything was run on MATLAB v25.2):
  - Image Processing Toolbox
  - Deep Learning Toolbox
  - Computer Vision Toolbox
  - Statistics and Machine Learning Toolbox
  - Wavelet Toolbox
  - Parallel Computing Toolbox
- Added all the raw images (Base image (and its Ground Truth) + 6 Fibroblast cell images + 2 Microglial cell images) to Demo Images folder. I have used these images to analyse and develop the Sobel + Watershed algorithm. To see the specific settings I used for each image, please refer to [Sobel + Watershed project's GitHub page](https://github.com/rustamtoshov23/Sobel-Watershed-segmenter).
- Added a folder called **Post-Processing**. It contains 4 little MATLAB codes (in all, files have to be loaded in manually in the code):
  - JSON_to_TIFF.mlx &rarr; can be used to convert the output JSON segmentation mask of this app into a more useful .tiff version.
  - Image_Character.mlx &rarr; calculates the RMS contrast (the higher the value, the higher the contrast is in the image) and Laplacian Variance of the image (the higher the value, the sharper is the image). Be careful as there is no set definitive value for a high contrast or high sharpness image. These metrics are used to compare image quality within the same dataset.
  - Segm_Metrics.mlx &rarr; if a ground truth of an image is present, can be used to calculate the segmentation metrics: F1 score, Boundary F1 score, 95% Hausdorff Distance, ASSD.
  - Overlay.mlx &rarr; allows the user to overlay the segmentation mask over the original image (segmentation will be overlaid in red). 
- **TO RUN THE APP:** you only really need the Multires_ML_Microscopy folder and the Multires_ML_Microscopy.m file. Make sure all files are downloaded from GitHub properly (it is recommended to download big MATLAB files individually), add all files to the same path in MATLAB, make sure you have downloaded all the Toolboxes listed above and then just run the Multires_ML_Microscopy.m file!
- If you have any questions with regards to these modifications, please contact the developer at 88.rustam.toshov@gmail.com


## UPDATES: Addition of 3D Microglia Segmentation and Classification
This update extends the Multires-ML-Microscopy application with a pipeline for the segmentation and morphological classification of microglia directly from 3D microscopy image stacks.

The added workflow performs 3D image loading and calibration, preprocessing, segmentation, removal of objects touching the lateral image boundaries, soma detection, separation of potentially merged cells, extraction of 3D morphological features, and machine-learning classification.

### 3D Microglia Pipeline

The following MATLAB functions were added in the `Multires_ML_Microscopy/Microglia_3D` folder:

- `loadMicroglia3D.m` – loads 3D TIFF/TIF and LSM microscopy image stacks and obtains voxel calibration where available.
- `preprocessMicroglia3D.m` – performs intensity normalisation, 3D Gaussian smoothing, background estimation and background subtraction.
- `segmentMicroglia3D.m` – performs Otsu-based hysteresis thresholding and 3D morphological reconstruction using 26-connectivity.
- `removeXYBorderObjects3D.m` – removes partially visible objects touching the X or Y image boundaries while retaining objects touching the Z boundaries.
- `detectMicrogliaSomas3D.m` – detects soma candidates within segmented microglial objects.
- `separateMicroglia3D.m` – separates objects containing multiple detected somas using soma-guided 3D geodesic region growing.
- `extractMicrogliaFeatures3D.m` – calculates morphological measurements from individual segmented microglia.
- `classifyMicroglia3D.m` – applies the trained machine-learning classifier to the extracted morphological features.

The trained classifier is provided as:

- `Microglia_Classifier.mat`

### Morphological Classification

Nine 3D morphological measurements are used as predictor variables:

1. Volume
2. Surface area
3. Equivalent diameter
4. Sphericity
5. Major axis length
6. Intermediate axis length
7. Minor axis length
8. Elongation
9. Flatness

Each analysed microglial cell is classified into one of three morphological classes:

- **Amoeboid**
- **Activated**
- **Ramified**

### Integration into the Application

The main `Multires_ML_Microscopy.m` application was modified to integrate the 3D microglia workflow into the existing graphical user interface.

The 3D microglia functionality allows a microscopy image stack to be processed through segmentation, individual-cell analysis and morphological classification within the application. Relevant processing parameters can be adjusted through the application interface.

### Demo Image

A demonstration 3D microscopy stack is included at:

`Demo Images/Microglia/TrialControlZip.tif`

The demonstration image originates from the **3DMorph** dataset developed for 3D analysis of microglial morphology:

York, E. M., LeDue, J. M., Bernier, L.-P., and MacVicar, B. A. (2018). *3DMorph Automatic Analysis of Microglial Morphology in Three Dimensions from Ex Vivo and In Vivo Imaging*. eNeuro, 5(6), ENEURO.0266-18.2018.

Original 3DMorph repository:
https://github.com/ElisaYork/3DMorph

Article:
https://doi.org/10.1523/ENEURO.0266-18.2018

### Full Development Pipeline

The complete standalone development repository, including the segmentation and classification pipeline, training-data preparation, classifier development, figures and results, is available here:

https://github.com/maulenar/Microglia-3D-Segmentation-Classification

The standalone repository contains more detailed documentation of the individual processing stages and machine-learning workflow.

### Contact

For questions regarding the 3D microglia segmentation and classification extension, please contact the developer at maulen.a0602@gmail.com.
