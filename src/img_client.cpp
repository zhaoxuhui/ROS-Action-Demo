#include<ros/ros.h>
#include<actionlib/client/simple_action_client.h>   // 客户端需要包含的头文件
#include<image_process/ProcImgAction.h> // 自定义Action所要包含的头文件
#include<opencv2/opencv.hpp>    // OpenCV基础包要包含的头文件

// 利用typedef提前定义好类型
typedef actionlib::SimpleActionClient <image_process::ProcImgAction> Client;

// 自定义函数，用于将Message传输的vector<uint8_t>类型数据转换成OpenCV的Mat类型
cv::Mat cvtArray2Img(std::vector<uint8_t> array, int img_height, int img_width) {
    cv::Mat restore_img(img_height, img_width, CV_8UC3);
    for (int i = 0; i < img_height; i++) {
        for (int j = 0; j < img_width; j++) {
            int index = i * img_width + j;
            uint8_t b = array[index];
            uint8_t g = array[index + img_height * img_width];
            uint8_t r = array[index + img_height * img_width * 2];
            restore_img.at<cv::Vec3b>(i, j) = cv::Vec3b(b, g, r);
        }
    }
    return restore_img;
}


// 当Action完成后的回调函数
void doneCb(const actionlib::SimpleClientGoalState &state,
            const image_process::ProcImgResultConstPtr &result) {
    // 新建一个Mat并将传输的数据转换成OpenCV能识别的Mat类型
    cv::Mat processed_img;
    processed_img = cvtArray2Img(result->processed_img, result->img_height, result->img_width);

    // 如果原图太大，将它缩小，这样利于展示
    if (result->img_width > 1200) {
        float ratio = 1200.0 / result->img_width;
        cv::resize(processed_img, processed_img, cv::Size(0, 0), ratio, ratio);
    }

    ROS_INFO("Finished processing image with %d x %d pixels...", result->img_height, result->img_width);

    // 展示图片
    cv::imshow("processed_img", processed_img);
    cv::waitKey(0);

    // 关闭ROS
    ros::shutdown();
}

// 当Action启动后的回调函数
void activeCb() {
    ROS_INFO("Contected to server,start processing image...");
}

// 当收到Feedback后的回调函数
void feedbackCb(const image_process::ProcImgFeedbackConstPtr &feedback) {
    ROS_INFO("percent complete:%.2f%%", feedback->complete_percent);
}

int main(int argc, char *argv[]) {
    // 用户输入图片路径
    std::string img_path;
    std::cout << "Input image path:\n";
    std::cin >> img_path;

    // 初始化并指定节点名称
    ros::init(argc, argv, "img_client");
    // 创建客户端并指定要连接到的服务端名称
    Client client("img_process", true);

    // 等待服务端
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    // 创建Action的goal，传入的是图片路径字符串
    image_process::ProcImgGoal img_goal;
    img_goal.img_path = img_path;

    // 将goal发送给服务端，并设置回调函数
    client.sendGoal(img_goal, &doneCb, &activeCb, &feedbackCb);

    // 循环执行
    ros::spin();
    return 0;
}