///////////////////////////////////////////////////////////////////////////
//
//  readBarCodeCore contains
//  implementation of readBarcode_published_c_api.
//  Detects message ,location and format detected, for bar codes in the
//  given image.
//  Copyright 2021-2024 The MathWorks, Inc.
//
///////////////////////////////////////////////////////////////////////////
#ifdef COMPILE_FOR_VISION_BUILTINS
#include <barcode/vision_defines.h>
#include <ZXing/BarcodeFormat.h>
#include <ZXing/DecodeHints.h>
#include <ZXing/MultiFormatReader.h>
#include <ZXing/GlobalHistogramBinarizer.h>
#include <ZXing/Result.h>
#include <ZXing/TextUtfEncoding.h>
#include <ZXing/DecodeStatus.h>
#include <barcode/ImageLuminanceSource.h>
#include <barcode/readBarcode_published_c_api.hpp>
#else
#include "BarcodeFormat.h"
#include "DecodeHints.h"
#include "MultiFormatReader.h"
#include "GlobalHistogramBinarizer.h"
#include "Result.h"
#include "TextUtfEncoding.h"
#include "DecodeStatus.h"
#include "ImageLuminanceSource.h"
#include "readBarcode_published_c_api.hpp"
#endif

#include <algorithm>// for std::transform - applies given function to a range
#include <vector>
#include <string>
#include <cstring>

////////////////////////////////////////////////////////////////////////////////
// Get supported barcode formats in a std::map
////////////////////////////////////////////////////////////////////////////////
std::map<std::string, ZXing::BarcodeFormat> getFormats()
{
    std::map<std::string, ZXing::BarcodeFormat> formatMap = {
        {"AZTEC",ZXing::BarcodeFormat::AZTEC},
        {"CODABAR",ZXing::BarcodeFormat::CODABAR},
        {"CODE-39",ZXing::BarcodeFormat::CODE_39},
        {"CODE-93",ZXing::BarcodeFormat::CODE_93},
        {"CODE-128",ZXing::BarcodeFormat::CODE_128},
        {"DATA-MATRIX",ZXing::BarcodeFormat::DATA_MATRIX},
        {"EAN-8",ZXing::BarcodeFormat::EAN_8},
        {"EAN-13",ZXing::BarcodeFormat::EAN_13},
        {"ITF",ZXing::BarcodeFormat::ITF},
        {"PDF-417",ZXing::BarcodeFormat::PDF_417},
        {"QR-CODE",ZXing::BarcodeFormat::QR_CODE},
        {"RSS-14",ZXing::BarcodeFormat::RSS_14},
        {"RSS-EXPANDED",ZXing::BarcodeFormat::RSS_EXPANDED},
        {"UPC-A",ZXing::BarcodeFormat::UPC_A},
        {"UPC-E",ZXing::BarcodeFormat::UPC_E}};

    return formatMap;
}

////////////////////////////////////////////////////////////////////////////////
// Parse the input formats and return vector of Zxing:Barcode format objects
////////////////////////////////////////////////////////////////////////////////

std::vector<ZXing::BarcodeFormat> parseFormats(std::string input, const int nFormats, const int* formatLengths) {

    auto formatMap = getFormats();
    std::vector<ZXing::BarcodeFormat> formats;

    if (nFormats == 1) {        
        int len = formatLengths[0];
        std::string tmpFormat = "";

        for (int j = 0; j < len; j++)
            tmpFormat = tmpFormat + input[j];

        if (!strcmp("all",tmpFormat.c_str())) {
            for (auto& it : formatMap)
                formats.push_back(it.second);
        }
        else {
            formats.push_back(formatMap.at(tmpFormat));
        }
    }
    else {
        // Use a user-specified set of families

        std::string format;
        std::string tmpFormat;
        int start = 0;
        for (int i = 0; i< nFormats; i++) {

            // extract length of each individual family
            int len = formatLengths[i];
            tmpFormat = "";
            for (int j = 0; j < len; j++)
                // add each family to string
                tmpFormat = tmpFormat + input[start+j];

            formats.push_back(formatMap.at(tmpFormat));
            start = start + len;

        }
    }
    return formats;
}
////////////////////////////////////////////////////////////////////////////////
// Executes single instance of 'MultiFormatReader'
////////////////////////////////////////////////////////////////////////////////

void singleMultiFormatReader(const std::vector<ZXing::BarcodeFormat>& formats,
                             const ZXing::GlobalHistogramBinarizer& binImage,
                             const bool robustRowScan,
                             ZXing::Result& result){

    // Hints initialized to search for all standards that the library supports
    ZXing::DecodeHints hints;

    hints.setPossibleFormats(formats);
    hints.setShouldTryHarder(robustRowScan);
    hints.setShouldTryRotate(robustRowScan);

    // Instantiate the 'MultiFormatReader' object
    ZXing::MultiFormatReader reader(hints);
    result = reader.read(binImage);
}

////////////////////////////////////////////////////////////////////////////////
// Executes two instances of 'MultiFormatReader'
////////////////////////////////////////////////////////////////////////////////

void dualMultiFormatReader(std::vector<ZXing::BarcodeFormat>& formats,
                           const ZXing::GlobalHistogramBinarizer& binImage,
                           const bool robustRowScan,
                           const bool robustPatternScan,
                           std::vector<ZXing::BarcodeFormat>::iterator& iterQR,
                           ZXing::Result& result){

    std::vector<ZXing::BarcodeFormat> formats2DHarder;

    // Separate workflow for QR and DATA_MATRIX
    if (iterQR != formats.end()){
        formats2DHarder.push_back(ZXing::BarcodeFormat::QR_CODE);
        formats.erase(iterQR);
    }

    auto iterDataMatrix = std::find(formats.begin(), formats.end(), ZXing::BarcodeFormat::DATA_MATRIX);
    if (iterDataMatrix != formats.end()){
        formats2DHarder.push_back(ZXing::BarcodeFormat::DATA_MATRIX);
        formats.erase(iterDataMatrix);
    }

    // For QR and DATA_MATRIX
    if (!formats2DHarder.empty()){

        ZXing::DecodeHints hints2DHarder;
        hints2DHarder.setPossibleFormats(formats2DHarder);
        hints2DHarder.setShouldTryHarder(robustPatternScan);
        hints2DHarder.setShouldTryRotate(robustPatternScan);

        ZXing::MultiFormatReader readers2DHarder(hints2DHarder);
        result = readers2DHarder.read(binImage);

        if (result.isValid())
            return;
    }
    else{ // For all other formats

        ZXing::DecodeHints hintsOthers;
        hintsOthers.setPossibleFormats(formats);
        hintsOthers.setShouldTryHarder(robustRowScan);
        hintsOthers.setShouldTryRotate(robustRowScan);

        ZXing::MultiFormatReader readersOthers(hintsOthers);
        result = readersOthers.read(binImage);
    }
}

////////////////////////////////////////////////////////////////////////////////
// Perform 1D/2D barcode detection and decoding using ZXing functions
////////////////////////////////////////////////////////////////////////////////

void zxingMultiFormatDetectDecode(const uint8_T * imgData, const size_t width, const size_t height,
                                  std::vector<ZXing::BarcodeFormat>& formats, const bool robustRowScan, const bool robustPatternScan, ZXing::Result& result){

    ImageLuminanceSource source(imgData, width, height);

    // Instantiate a Global Histogram binarizer (works well as a general purposed binarizer)
    ZXing::GlobalHistogramBinarizer binImage(std::shared_ptr<ZXing::LuminanceSource>(&source, [](void*) {}));

    // Only QR and DATA_MATRIX codes are affected by setting "TryHarder"
    auto iterQR = std::find(formats.begin(), formats.end(), ZXing::BarcodeFormat::QR_CODE);
    auto iterDataMatrix = std::find(formats.begin(), formats.end(), ZXing::BarcodeFormat::DATA_MATRIX);

    // Depending on the hints provided, one or two instances of MultiFormatReader are executed.
    if (robustRowScan == robustPatternScan || (iterQR == formats.end() &&
                                               iterDataMatrix == formats.end())){
        singleMultiFormatReader(formats, binImage, robustRowScan, result);
    }
    else {
        dualMultiFormatReader(formats, binImage, robustRowScan, robustPatternScan, iterQR, result);
    }

}

////////////////////////////////////////////////////////////////////////////////
// Returns the detected bar code format as a string
////////////////////////////////////////////////////////////////////////////////

std::string getBarcodeFormat(const ZXing::BarcodeFormat&  format) {

    auto formatMap = getFormats();
    std::string formatOut;
    for( auto& it : formatMap) {
        if(it.second == format) {
            formatOut = it.first;
            return formatOut;
        }
    }
    return "";
}

////////////////////////////////////////////////////////////////////////////////
// Initializes the locations of the barcode in the image
////////////////////////////////////////////////////////////////////////////////

void getBarcodeLocations(double* &locs, const std::vector<ZXing::ResultPoint>& resultPoints) {

    int idx = 0;
    for( auto& it : resultPoints) {
        locs[idx] = it.x();
        locs[idx + resultPoints.size()] = it.y();
        idx++;
    }
}

////////////////////////////////////////////////////////////////////////////////
// API that takes required input from MATLAB
////////////////////////////////////////////////////////////////////////////////

void multiFormatDetectDecode(void* mImgData, const int32_t width,
                             const int32_t height, void* mFormatData,
                             const int nFormats,  const int* formatLengths,
                             const bool robustRowScan,
                             const bool robustPatternScan, void** resultObj,
                             int* locSize, int* msgLen, int* formatLen) {

    uint8_T* imgData = static_cast<uint8_T*>(mImgData);
    const std::string inputFormat = static_cast<const char*>(mFormatData);

    // parse formats to get vector of Zxing:BarcodeFormat objects
    std::vector<ZXing::BarcodeFormat> formats = parseFormats(inputFormat, nFormats, formatLengths);

    // initialize ZXing:Result object
    ZXing::Result *result = new ZXing::Result(ZXing::DecodeStatus::NoError);

    // resultObj is a reference to the ZXing:Result object
    *resultObj = (void*)result;

    ZXing::Result &res = *result;

    // Perform multi format detect decode for 1D/2D families
    zxingMultiFormatDetectDecode(imgData, width, height, formats, robustRowScan, robustPatternScan, res);

    // initialize length/sizes of message, locations and detected formats
    std::string data;
    if (res.isValid()) {
        data = ZXing::TextUtfEncoding::ToUtf8(res.text());
    }
    *locSize = (int)res.resultPoints().size();
    *msgLen = (int)data.length();
    *formatLen = (int)getBarcodeFormat(res.format()).length();
}

////////////////////////////////////////////////////////////////////////////////
// Function to initialize all the required outputs - message, locations and
// detected format
////////////////////////////////////////////////////////////////////////////////

void initializeOutput(void* msg, void* loc, void* format, void* resultObj) {

    char* dataStr = static_cast<char*>(msg);
    double* location = static_cast<double*>(loc);
    char* detectedFormat = static_cast<char*>(format);
    ZXing::Result &res = *((ZXing::Result*)resultObj);

    if (res.isValid()) {
        std::string tempMessage = ZXing::TextUtfEncoding::ToUtf8(res.text());
        strcpy(dataStr, tempMessage.c_str());
    }

    std::string tempFormat = getBarcodeFormat(res.format());
    strcpy(detectedFormat, tempFormat.c_str());
    getBarcodeLocations(location, res.resultPoints());
}

///////////////////////////////////////////////////////////////////////////
// Function to delete the MATLAB pointer
///////////////////////////////////////////////////////////////////////////
void deleteResultPtr(void* ptrObj) {
    if (ptrObj != NULL) {
        delete((ZXing::Result*)ptrObj);
        ptrObj = NULL;
    }
}
