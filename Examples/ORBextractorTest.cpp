#include "ORBextractor.hpp"
#include <opencv2/opencv.hpp>

int main() {
    // 初始化 ORBextractor 参数
    int nfeatures = 500; // 特征点数量
    float scaleFactor = 1.2; // 尺度因子
    int nlevels = 8; // 金字塔层数
    int iniThFAST = 20; // 初始 FAST 阈值
    int minThFAST = 7; // 最小 FAST 阈值

    // 创建 ORBextractor 对象
    ORB_SLAM3::ORBextractor orbExtractor(nfeatures, scaleFactor, nlevels, iniThFAST, minThFAST);

    // 读取图像
    cv::Mat image = cv::imread("test.jpg", cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        std::cerr << "Could not open or find the image" << std::endl;
        return -1;
    }

    // 存储特征点和描述符的容器
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::vector<int> vLappingArea;
    vLappingArea.resize(2);
    vLappingArea[0] = 0;
    vLappingArea[1] = image.cols;
    std::cout << "图像已成功保存到1111111ma "  << std::endl;
    // 提取特征点和描述符
    int monoIndex = orbExtractor(image, cv::noArray(), keypoints, descriptors, vLappingArea);
     

    // 绘制特征点
    cv::Mat imageWithKeypoints;
    cv::drawKeypoints(image, keypoints, imageWithKeypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

    // 显示图像
    //cv::imshow("Image with Keypoints", imageWithKeypoints);
    //cv::waitKey(0);
    std::string filename = "image_with_keypoints.png";
    // 使用cv::imwrite函数保存图像
    bool saved = cv::imwrite(filename, imageWithKeypoints);

    if (saved) {
        std::cout << "图像已成功保存到 " << filename << std::endl;
    } else {
        std::cerr << "保存图像时出错，请检查图像数据或权限。" << std::endl;
    }

    return 0;
}