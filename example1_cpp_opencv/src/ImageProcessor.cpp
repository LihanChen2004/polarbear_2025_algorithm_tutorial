#include "ImageProcessor.hpp"

#include <iostream>
#include <opencv2/opencv.hpp>

// 构造函数，读取图像
ImageProcessor::ImageProcessor(const std::string & imagePath)
{
  image = cv::imread(imagePath);  // 使用 OpenCV 读取图像
  if (image.empty()) {
    std::cerr << "Error: Could not open or find the image!" << '\n';
  }
}

// 转换为灰度图像
void ImageProcessor::toGray()
{
  if (!image.empty()) {
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
  }
}

// 调整图像大小
void ImageProcessor::resizeImage(int width, int height)
{
  if (!image.empty()) {
    cv::resize(image, image, cv::Size(width, height));
  }
}

// 应用高斯模糊
void ImageProcessor::applyGaussianBlur(int kernelSize)
{
  if (!image.empty()) {
    cv::GaussianBlur(image, image, cv::Size(kernelSize, kernelSize), 0);
  }
}

// 显示图像
void ImageProcessor::showImage(const std::string & windowName)
{
  if (!image.empty()) {
    cv::imshow(windowName, image);
    cv::waitKey(0);  // 等待按键
  }
}

// 保存图像
void ImageProcessor::saveImage(const std::string & outputPath)
{
  if (!image.empty()) {
    cv::imwrite(outputPath, image);
  }
}
