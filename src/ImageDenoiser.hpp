#ifndef IMAGE_DENOISER_HPP
#define IMAGE_DENOISER_HPP

#include <opencv2/core.hpp>

class ImageDenoiser {
    public:
        static void denoiseImage(cv::Mat &originalImage, cv::Mat &colorMask, cv::Mat &processedImage, int thresholdValue, int maxValue);
};

#endif // IMAGE_DENOISER_HPP
