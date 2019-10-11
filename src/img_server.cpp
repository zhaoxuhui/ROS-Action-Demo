#include<ros/ros.h>
#include<actionlib/server/simple_action_server.h>   // 服务端需要包含的头文件
#include<image_process/ProcImgAction.h> // 自定义Action所要包含的头文件
#include<opencv2/opencv.hpp>    // OpenCV基础包要包含的头文件

// 利用typedef提前定义好类型
typedef actionlib::SimpleActionServer <image_process::ProcImgAction> Server;

// 自定义的用于将OpenCV的Mat类型转成Message传输的vector<uint8_t>类型的函数
std::vector<uint8_t> cvtImg2Array(cv::Mat img) {
    // array的数据格式是B、G、R波段像素依次存储
    std::vector<uint8_t> array;
    for (int c = 0; c < img.channels(); c++) {
        for (int i = 0; i < img.size().height; i++) {
            for (int j = 0; j < img.size().width; j++) {
                array.push_back(img.at<cv::Vec3b>(i, j)[c]);
            }
        }
    }
    return array;
}

// 服务端在收到客户端发来的Action后调用的回调函数
void executeCb(const image_process::ProcImgGoalConstPtr &goal, Server *as) {
    image_process::ProcImgFeedback feedback;

    // 根据客户端发来的图片路径读取并转换图片数据
    ROS_INFO("Reading image...");
    cv::Mat img = cv::imread(goal->img_path);
    std::vector<uint8_t> array = cvtImg2Array(img);
    int img_height = img.size().height;
    int img_width = img.size().width;

    ROS_INFO("Server is processing image with %d x %d pixels...", img_height, img_width);

    // 新建一个result_array用于存放处理后的结果
    std::vector<uint8_t> result_array(img_height * img_width * 3);

    // 这里依次遍历像素，实现将彩色图像转换为灰度图像的功能
    // 为了便于功能扩展，最后的灰度图也设置成了三个通道，各通道灰度值相同
    for (int i = 0; i < img_height * img_width; i++) {
        int b = array[i];
        int g = array[i + img_height * img_width];
        int r = array[i + img_height * img_width * 2];
        uint8_t gray = uint8_t((r * 30 + g * 59 + b * 11 + 50) / 100);
        result_array[i] = gray;
        result_array[i + img_height * img_width] = gray;
        result_array[i + img_height * img_width * 2] = gray;

        // 每执行10次向客户端发布一次进度Feedback
        if (i % 10 == 0) {
            feedback.complete_percent = (i * 100.0) / (img_height * img_width);
            as->publishFeedback(feedback);
        }
    }

    // 当Action完成后，向客户端返回结果
    ROS_INFO("Server finished processing image.");
    image_process::ProcImgResult result;
    // 虽然在Message里定义的processed_img是数组，但是可以直接将vector赋值给它的
    // 因为在自动生成的头文件中uint8[]就是通过std::vector<uint8_t>实现的
    result.processed_img = result_array;
    result.img_height = img_height;
    result.img_width = img_width;
    // 最后在setSucceeded函数里设置返回值，将结果返回给客户端
    as->setSucceeded(result);
}

// 中断回调函数
void preemptCb(Server *as) {
    if (as->isActive()) {
        // 强制中断
        as->setPreempted();
    }
}


int main(int argc, char *argv[]) {
    // 初始化并指定节点名称
    ros::init(argc, argv, "img_server");
    // 创建句柄
    ros::NodeHandle nh;

    // 创建服务端
    // 第一个参数是节点句柄
    // 第二个参数是新建的Server的名称
    // 第三个参数是处理的回调函数，这里使用了boost库的bind函数，使得函数传入的参数类型可以不确定
    // _1是占位符，格式为：boost::bind(函数名, 参数1，参数2，...)
    // 更多可参考这篇博客https://blog.csdn.net/jack_20/article/details/78862402
    // 第四个参数是表示服务是否自动开始，详细可参考SimpleActionServer的构造函数文档
    Server server(nh, "img_process", boost::bind(&executeCb, _1, &server), false);
    //注册抢占回调函数
    server.registerPreemptCallback(boost::bind(&preemptCb, &server));
    // 开始服务
    server.start();
    ROS_INFO("Service is ready to work.");
    // 循环执行
    ros::spin();
    return 0;
}