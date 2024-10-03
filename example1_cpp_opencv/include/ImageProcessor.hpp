#ifndef IMAGE_PROCESSOR_HPP
#define IMAGE_PROCESSOR_HPP

#include <opencv2/opencv.hpp>

class ImageProcessor
{
public:
  // 构造函数，读取图像
  ImageProcessor(const std::string & imagePath);

  // 转换为灰度图像
  void toGray();

  // 调整图像大小
  void resizeImage(int width, int height);

  // 应用高斯模糊
  void applyGaussianBlur(int kernelSize);

  // 显示图像
  void showImage(const std::string & windowName);

  // 保存图像
  void saveImage(const std::string & outputPath);

private:
  cv::Mat image;  // 内部保存处理的图像
};

#endif  // IMAGE_PROCESSOR_HPP
