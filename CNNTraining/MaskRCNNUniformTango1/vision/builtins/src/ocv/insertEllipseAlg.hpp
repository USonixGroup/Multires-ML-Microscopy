/* Copyright 2024 The MathWorks, Inc. */

/* Algorithm to insert ellipse shape used in both simulation and codegen */
#include "opencv2/opencv.hpp"

//////////////////////////////////////////////////////////////////////////////
//Algorithmic operations for drawing ellipses using OpenCV
//////////////////////////////////////////////////////////////////////////////
void drawMultipleEllipses(cv::Mat img, int32_T* pts, int32_T* axes, double*
        angleIn, int row, double startAngle, double endAngle, double*
        lineColor, double opacity, int thickness, int lineType, int shift) {
    // Ellipse location and size
    int xCenter = 0, yCenter = 0;
    int semiMajorAxisLength = 0, semiMinorAxisLength = 0;
    int longSemiAxisLength = 0;
    double angle = 0; 
    
    // Fixed to floating point conversion factor. 
    const float factor = std::pow(2.f,(float)-shift);

    for (int i = 0; i < row; ++i) 
    { 
        xCenter = pts[i]; 
        yCenter = pts[row+i];
        semiMajorAxisLength = axes[i]; 
        semiMinorAxisLength = axes[row+i];
        angle = angleIn[i]; 
       
        cv::Scalar ellipseColor(lineColor[2*row+i],
                lineColor[row+i], lineColor[i]); // ellipseColor(B, G, R);

        if (thickness == -1) {
            // When thickness is set to -1, cv::ellipse draws a filled ellipse.

            cv::Mat imgPrev; 
            cv::Rect roiRect; 

            // Create a copy of the image.
            imgPrev = img.clone(); 

            // Draw ellipse on the image
            cv::ellipse(img, cv::Point(xCenter, yCenter),
                    cv::Size(semiMajorAxisLength, semiMinorAxisLength), angle,
                    startAngle, endAngle, ellipseColor, thickness, lineType,
                    shift);

            if (shift > 0) 
            {
                // Ellipse floating point center and axes lengths are encoded
                // as fixed point integers. Convert to back to floating point using
                // x = xfxp * 2^-shift.
                xCenter = static_cast<int>((float)xCenter * factor);
                yCenter = static_cast<int>((float)yCenter * factor);
                semiMajorAxisLength = static_cast<int>((float)semiMajorAxisLength * factor);
                semiMinorAxisLength = static_cast<int>((float)semiMinorAxisLength * factor);
            }
      
            longSemiAxisLength = std::max(semiMajorAxisLength, semiMinorAxisLength);
       
            // Determine top-left corner of ROI. Pad the ROI by 2 pixels to
            // account for the round-off erros in the fixpt-float conversion.
            auto xTopLeft = xCenter-longSemiAxisLength-2; 
            auto yTopLeft = yCenter-longSemiAxisLength-2;
            auto sideLength = 2*longSemiAxisLength+4;

            // Add opacity to the rectangle by blending the region of interest
            // around the ellipse. 
            roiRect = cv::Rect(xTopLeft, yTopLeft, sideLength, sideLength);
            roiRect = roiRect & cv::Rect(0, 0, img.cols, img.rows);

            // Control the opacity (alpha) of the ellipse using addWeighted
            cv::addWeighted(img(roiRect), opacity, imgPrev(roiRect), 1.0 - opacity, 0, img(roiRect));
        } 
        else 
        {
            // Draw ellipse on the image
            cv::ellipse(img, cv::Point(xCenter, yCenter), cv::Size(semiMajorAxisLength, semiMinorAxisLength), angle, 
                        startAngle, endAngle, ellipseColor, thickness, lineType, shift);
        }
    }
}
