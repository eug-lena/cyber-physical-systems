#include "ImageDenoiser.hpp"
#include <opencv2/imgproc.hpp>

void ImageDenoiser::denoiseImage(cv::Mat &originalImage, cv::Mat &colorMask, cv::Mat &processedImage, int thresholdValue, int maxValue)
{
    // Create a copy of the colorMask to preserve the original data
    cv::Mat colorMaskCopy;
    colorMask.copyTo(colorMaskCopy);

    // Apply Gaussian Blur to the color mask to reduce noise
    cv::GaussianBlur(colorMaskCopy, colorMaskCopy, cv::Size(5, 5), 0);

    // Apply Closing operation to the color mask to improve quality
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(colorMaskCopy, colorMaskCopy, cv::MORPH_CLOSE, element);

    // Create a mask to ignore the car at the bottom-center of the image
    cv::Mat mask = cv::Mat::ones(originalImage.size(), CV_8U);
    cv::Rect ignoreRegion(mask.cols / 4, 3 * mask.rows / 4, mask.cols / 2, mask.rows / 4);
    mask(ignoreRegion) = 0;

    // Apply the mask to the colorMaskCopy
    colorMaskCopy = colorMaskCopy & mask;

    // Perform a Bitwise And operation to extract the color information from the original image
    cv::bitwise_and(originalImage, originalImage, processedImage, colorMaskCopy);

    // Convert the processed image to Grayscale
    cv::cvtColor(processedImage, processedImage, cv::COLOR_BGR2GRAY);

    // Apply a Threshold to the processed image
    cv::threshold(processedImage, processedImage, thresholdValue, maxValue, cv::THRESH_BINARY);
}