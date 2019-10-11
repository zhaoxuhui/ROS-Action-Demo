# coding=utf-8
import rospy
import roslib
roslib.load_manifest('image_process')
import actionlib
import cv2
import numpy as np

from image_process.msg import ProcImgAction, ProcImgGoal


# 自定义函数，用于将字符串形式的数据转换成uint8数字表示的NDArray
# 在Python中Message的uint8数组以字符串形式传输的，这点需要注意
def cvtArray2Img(array, img_height, img_width):
    restore_img = np.zeros([img_height, img_width, 3], np.uint8)
    print "receiving and converting image data..."
    for i in range(img_height):
        for j in range(img_width):
            index = i*img_width+j
            # 利用ord()函数将单个字母转换成对应的字符编码
            b = ord(array[index])
            g = ord(array[index+img_height*img_width])
            r = ord(array[index+img_height*img_width*2])
            restore_img[i, j, :] = [b, g, r]
    return restore_img

# 当Action完成后的回调函数
def doneCb(state, result):
    # 新建一个Mat并将传输的字符串数据转换成OpenCV能识别的Mat类型
    processed_img = cvtArray2Img(
        result.processed_img, result.img_height, result.img_width)

    # 如果原图太大，将它缩小，这样利于展示
    if processed_img.shape[0] > 1200:
        ratio = 1200.0/result.img_width
        processed_img = cv2.resize(processed_img, None, None, ratio, ratio)

    rospy.loginfo("Finished processing image with %d x %d pixels...",
                  result.img_height, result.img_width)

    # 展示图片
    cv2.imshow("processed_img", processed_img)
    cv2.waitKey(0)

# 当Action启动后的回调函数
def activeCb():
    rospy.loginfo("Contected to server,start processing image...")

# 当收到Feedback后的回调函数
def feedbackCb(feedback):
    rospy.loginfo("percent complete:%.2f%%", feedback.complete_percent)


if __name__ == "__main__":
    # 用户输入图片路径
    img_path = raw_input("Input image path:\n")

    # 初始化并指定节点名称
    rospy.init_node("img_client")
    # 创建客户端并指定要连接到的服务端名称
    client = actionlib.SimpleActionClient("img_process", ProcImgAction)

    # 等待服务端
    rospy.loginfo("Waiting for action server to start.")
    client.wait_for_server()
    rospy.loginfo("Action server started, sending goal.")

    # 创建Action的goal，传入的是图片路径字符串
    img_goal = ProcImgGoal()
    img_goal.img_path = img_path

    # 将goal发送给服务端，并设置回调函数
    client.send_goal(img_goal, doneCb, activeCb, feedbackCb)

    # 循环执行
    rospy.spin()
