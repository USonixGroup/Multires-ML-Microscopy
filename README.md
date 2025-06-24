# Multiresolution Machine Learning for Segmentation and Characterisation of Microscopy Images
Microscopy is fundamental in medical research and practice, allowing observation of cellular states and responses to various stimuli. Analyzing these microscopy images is therefore a vital part of medical research. This process typically involves identifying cell boundaries to extract and analyze cellular parameters like size and shape, and to track their changes over time.

Multires-ML-Microscopy is an open-source library written in MATLAB, featurin a fully integrated pipeline for the segmentation, analysis, and tracking of cells in microscopy images. The library addresses a gap in biomedical imaging by combining advanced machine learning techniques with accessible design and customisability. Using wavelet-based pre-processing enhanced image quality under noise, a Mask R-CNN segmentation model with a custom EfficientNet-inspired backbone delivered high-precision cell masks, and the tracking module improved temporal consistency in live-cell imaging. This functionality is encapsulated within a user-friendly GUI, making the tool accessible to researchers without coding expertise.

Designed for use in biological and medical research environments, the pipeline enables high-throughput and reproducible analysis of cellular morphology, supporting tasks such as drug testing, disease modelling, and cell behaviour studies.

## HSMR25 Submission & DOI
This repository accompanies our submission to The 17th Hamlyn Symposium on Medical Robotics (HSMR25), held at the Royal Geographical Society on 27th June 2025. Our work is presented as part of the session "Healing Through Collaboration: Open-Source Software in Surgical, Biomedical and AI Technologies", highlighting the importance of community-driven development in medical imaging. The full submission is archived with Zenodo and can be accessed via DOI: [10.5281/zenodo.15727799](https://doi.org/10.5281/zenodo.15727799).

## Getting help
Enquiries about the library and questions should be asked on the discxussion page of the GitHub or directed to the team of developers on the GitHub or via email:nikhil.dhulashia.21@ucl.ac.uk, or yigit.dogan.21@ucl.ac.uk

## Citation
If you use Multires-ML-Microscopy in your work, please cite it as follows:

**APA**
```
N Dhulashia1, Y Doğan1, D Rajpara1, X Shi1, L Casamitjana Ortega1, R Haqshenas (2025), Multires-ML-Microscopy: Multiresolution Machine Learning based Segmentation and Characterisation of Microscopy Images, https://github.com/USonixGroup/Multires-ML-Microscopy
```

**BibTeX**
```
@software{optimuslib,
author = {N Dhulashia1, Y Doğan1, D Rajpara1, X Shi1, L Casamitjana Ortega1, R Haqshenas },
title = {Multires-ML-Microscopy: Multiresolution Machine Learning based Segmentation and Characterisation of Microscopy Images},
url = {https://github.com/USonixGroup/Multires-ML-Microscopy},
version = {0.1}
}
```
## License
Multires-ML-Microscopy s licensed under an MIT licence.
Copyright (c) 2025.
