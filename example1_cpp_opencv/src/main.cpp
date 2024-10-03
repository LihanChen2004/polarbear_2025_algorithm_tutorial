#include "ImageProcessor.hpp"

int main()
{
  // 创建 ImageProcessor 对象，并读取图像
  ImageProcessor processor("/home/lihanchen/Downloads/smbu.jpg");

  // 转换为灰度图像
  processor.toGray();

  // 调整图像大小
  processor.resizeImage(1920, 1080);

  // 应用高斯模糊
  processor.applyGaussianBlur(7);

  // 显示处理后的图像
  processor.showImage("Processed Image");

  // 保存处理后的图像
  processor.saveImage("output.jpg");

  return 0;
}
