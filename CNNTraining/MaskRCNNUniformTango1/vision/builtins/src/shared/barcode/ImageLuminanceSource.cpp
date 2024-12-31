////////////////////////////////////////////////////////////////////////////////
// Implements a custom class that inherits from the abstract 'LuminanceSource'
// class for storing the image data passed in from MATLAB. Construction performs
// a shallow copy of the image data and expects the image to be row-major.
//
// The class provides methods to access a row, or a matrix from the image
// and utility functions to rotate and crop the image.
//
// Copyright 2019-2024 The MathWorks, Inc.
////////////////////////////////////////////////////////////////////////////////
#ifdef COMPILE_FOR_VISION_BUILTINS
#include <barcode/ImageLuminanceSource.h>
#include <ZXing/ByteArray.h>
#else
#include "ImageLuminanceSource.h"
#include "ByteArray.h"
#endif
#include <algorithm>
#include <stdexcept>
#include <assert.h>

////////////////////////////////////////////////////////////////////////////////
// Convenience function for copying ImageLuminanceSource object
////////////////////////////////////////////////////////////////////////////////
inline std::shared_ptr<ZXing::ByteArray> MakeCopy(const uint8_t* pixels, const size_t rowBytes, const size_t left, const size_t top, const size_t width, const size_t height)
{
    auto result = std::make_shared<ZXing::ByteArray>();
    result->resize(width * height);
    const uint8_t* srcRow = pixels + top * rowBytes + left;
    uint8_t* destRow = result->data();
    for (size_t y = 0; y < height; ++y, srcRow += rowBytes, destRow += width) {
        std::copy_n(srcRow, width, destRow);
    }
    return result;
}

////////////////////////////////////////////////////////////////////////////////
// Convenience function for getting rotation angle in range: [0,360)
////////////////////////////////////////////////////////////////////////////////
inline int modTo360(const int degreeCW){
    return (degreeCW + 360) % 360;
}

////////////////////////////////////////////////////////////////////////////////
// Constructor for copying image data as a shared_ptr for rotation workflows
////////////////////////////////////////////////////////////////////////////////
ImageLuminanceSource::ImageLuminanceSource(std::shared_ptr<const ZXing::ByteArray> pixelsArray, const size_t width, const size_t height):
    _pixelsArray(pixelsArray), _left(0), _top(0), _width(width), _height(height), _rowBytes(width)
{
    _pixels = _pixelsArray->data();
};

////////////////////////////////////////////////////////////////////////////////
// Getter for _width of image
////////////////////////////////////////////////////////////////////////////////
int ImageLuminanceSource::width() const
{
    return static_cast<int>(_width);
}

////////////////////////////////////////////////////////////////////////////////
// Getter for _height of image
////////////////////////////////////////////////////////////////////////////////

int ImageLuminanceSource::height() const
{
    return static_cast<int>(_height);
}

////////////////////////////////////////////////////////////////////////////////
// Access row in image (pointer to the first element)
////////////////////////////////////////////////////////////////////////////////
const uint8_t * ImageLuminanceSource::getRow(int y, ZXing::ByteArray& buffer, bool forceCopy) const
{
    const uint8_t* row = _pixels + (y + _top)*_rowBytes + _left;
    if (!forceCopy) {
        return row;
    }
    else {
        buffer.resize(_width);
        std::copy_n(row, _width, buffer.begin());
        return buffer.data();
    }
}

////////////////////////////////////////////////////////////////////////////////
// Access matrix (contiguous rows in image)
////////////////////////////////////////////////////////////////////////////////
const uint8_t * ImageLuminanceSource::getMatrix(ZXing::ByteArray& buffer, int& outRowBytes, bool forceCopy) const
{
    const uint8_t* row = _pixels + _top*_rowBytes + _left;
    if (!forceCopy) {
        outRowBytes = static_cast<int>(_width);
        return row;
    }
    else{
        outRowBytes = static_cast<int>(_width);
        buffer.resize(_width * _height);
        uint8_t* dest = buffer.data();
        for (size_t y = 0; y < _height; ++y, row += _rowBytes, dest += _width) {
            std::copy_n(row, _width, dest);
        }
        return buffer.data();
    }

}

////////////////////////////////////////////////////////////////////////////////
// Return true if the class provides crop support
////////////////////////////////////////////////////////////////////////////////
bool ImageLuminanceSource::canCrop() const
{
    return true;
}

////////////////////////////////////////////////////////////////////////////////
// Access ROI in image
////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<ZXing::LuminanceSource> ImageLuminanceSource::cropped(int left, int top, int width, int height) const
{
    return std::make_shared<ImageLuminanceSource>(_left + left, _top + top, width, height, _pixels, _rowBytes);
}

////////////////////////////////////////////////////////////////////////////////
// Return true if class supports rotation
////////////////////////////////////////////////////////////////////////////////
bool ImageLuminanceSource::canRotate() const
{
    return true;
}

////////////////////////////////////////////////////////////////////////////////
// Rotate image by 90 degrees
////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<ZXing::ByteArray> ImageLuminanceSource::rotate90() const
{
    auto pixels = std::make_shared<ZXing::ByteArray>(static_cast<int>(_width) * static_cast<int>(_height));
    const uint8_t* srcRow = _pixels + _top * _rowBytes + _left;
    uint8_t* dest = pixels->data();
    for (size_t y = 0; y < _height; ++y, srcRow += _rowBytes) {
        for (size_t x = 0; x < _width; ++x) {
            dest[x * _height + (_height - y - 1)] = srcRow[x];
        }
    }
    return pixels;
}

////////////////////////////////////////////////////////////////////////////////
// Rotate image by 180 degrees
////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<ZXing::ByteArray> ImageLuminanceSource::rotate180() const
{
    auto pixels = MakeCopy(_pixels, _rowBytes, _left, _top, _width, _height);
    std::reverse(pixels->begin(), pixels->end());
    return pixels;
}

////////////////////////////////////////////////////////////////////////////////
// Rotate image by 270 degrees
////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<ZXing::ByteArray> ImageLuminanceSource::rotate270() const
{
    auto pixels = std::make_shared<ZXing::ByteArray>(static_cast<int>(_width) * static_cast<int>(_height));
    const uint8_t* srcRow = _pixels + _top * _rowBytes + _left;
    uint8_t* dest = pixels->data();
    for (size_t y = 0; y < _height; ++y, srcRow += _rowBytes) {
        for (size_t x = 0; x < _width; ++x) {
            dest[(_width - x - 1) * _height + y] = srcRow[x];
        }
    }
    return pixels;
}

////////////////////////////////////////////////////////////////////////////////
// Access rotated image (supports only predefined rotations)
////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<ZXing::LuminanceSource> ImageLuminanceSource::rotated(int degreeCW) const
{
    assert((degreeCW == 90) || (degreeCW == 180) || (degreeCW == 270) || (degreeCW == 0)); 

    degreeCW = modTo360(degreeCW + 360) % 360;
    if (degreeCW == 90)
    {
        auto pixels = rotate90();
        return std::make_shared<ImageLuminanceSource>(pixels, _height, _width);
    }
    else if (degreeCW == 180) {
        // Same as a vertical flip followed a horizonal flip
        auto pixels = rotate180();
        return std::make_shared<ImageLuminanceSource>(pixels, _width, _height);
    }
    else if (degreeCW == 270) {
        auto pixels = rotate270();
        return std::make_shared<ImageLuminanceSource>(pixels, _height, _width);
    }
    else {
        return std::make_shared<ImageLuminanceSource>(0, 0, _width, _height, _pixels, _width);
    }
}
