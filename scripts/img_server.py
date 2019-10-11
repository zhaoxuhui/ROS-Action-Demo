# coding=utf-8
import rospy
import roslib
roslib.load_manifest("image_process")
import actionlib
import cv2

from image_process.msg import ProcImgAction,ProcImgFeedback,ProcImgResult


# 自定义函数，用于将OpenCV的NDArray类型的数据转换成一维的list
def cvtImg2Array(img):
    # array的数据格式是B、G、R波段像素依次存储
    array = []
    img_height = img.shape[0]
    img_width = img.shape[1]
    img_channels = img.shape[2]

    for c in range(img_channels):
        for i in range(img_height):
            for j in range(img_width):
                array.append(img[i,j,c])
    return array

class ImgProcessServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("img_process",ProcImgAction,self.exectueCb,False)
        self.server.register_preempt_callback(self.preemptCb)
        self.server.start()
        rospy.loginfo("Service is ready to work.")
    
    # 服务端在收到客户端发来的Action后调用的回调函数
    def exectueCb(self,goal):
        feedback = ProcImgFeedback()

        # 根据客户端发来的图片路径读取并转换图片数据
        img = cv2.imread(goal.img_path)
        array = cvtImg2Array(img)
        img_height = img.shape[0]
        img_width = img.shape[1]
        
        rospy.loginfo("Server is processing image with %d x %d pixels...", img_height, img_width)
        
        # 新建一个result_array用于存放处理后的结果
        result_array = [0]*(img_height * img_width * 3)
        
        # 这里依次遍历像素，实现将彩色图像转换为灰度图像的功能
        # 为了便于功能扩展，最后的灰度图也设置成了三个通道，各通道灰度值相同
        for i in range(img_height*img_width):
            b = array[i]
            g = array[i + img_height * img_width]
            r = array[i + img_height * img_width * 2]
            gray = int((r * 30 + g * 59 + b * 11 + 50) / 100)
            result_array[i]=gray
            result_array[i + img_height * img_width] = gray
            result_array[i + img_height * img_width * 2] = gray
            
            # 每执行10次向客户端发布一次进度Feedback
            if i%10==0:
                feedback.complete_percent = (i * 100.0) / (img_height * img_width)
                self.server.publish_feedback(feedback)
        
        # 当Action完成后，向客户端返回结果
        rospy.loginfo("Server finished processing image.")
        result = ProcImgResult()
        result.processed_img = result_array
        result.img_height = img_height
        result.img_width = img_width
        # 最后在set_succeeded函数里设置返回值，将结果返回给客户端
        self.server.set_succeeded(result)
    
    # 中断回调函数
    def preemptCb(self):
        if self.server.is_active():
            # 强制中断
            self.server.set_preempted()

if __name__ == "__main__":
    # 初始化并指定节点名称
    rospy.init_node("img_server")
    # 调用服务类启动服务
    server = ImgProcessServer()
    # 循环执行
    rospy.spin()