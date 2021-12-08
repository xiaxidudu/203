# coding=utf-8
import sys
import time
import cv2
import numpy as np
import vision_definitions
from PIL import Image
import math
import socket
from naoqi import ALProxy


IP = "192.168.2.102"#nao机器人的ip地址，需要及时更换
PORT = 9559 #默认连接机器人聆听端口

motion = ALProxy("ALMotion", IP, PORT)#调用运动模块
camera = ALProxy("ALVideoDevice", IP, PORT)#调用视觉模块

red_ball = ALProxy("ALRedBallDetection", IP, PORT)
landmark = ALProxy("ALLandMarkDetection", IP, PORT)

memory = ALProxy("ALMemory", IP, PORT)
tts = ALProxy("ALTextToSpeech", IP, PORT)

posture = ALProxy("ALRobotPosture", IP, PORT)#调用初始模块
auto_life = ALProxy("ALAutonomousLife", IP, PORT)#调用自主生活模块

i_can_speak=ALProxy("ALTextToSpeech", IP, PORT)

max_step_x = 0.8  # 最大正向平移沿X（米）0.001-0.08
max_step_y = 0.14  # 沿Y方向的绝对最大横向平移（米）0.101-0.16
max_step_theta = 0.4  # 绕Z轴绝对最大旋转（弧度）0.001-0.524
max_step_frequency = 0.88  # 最大步进频率（标准化，无单位）1
step_height = 0.02  # Z轴高程（米）0.005-0.040
tor_sow_x = 0.0  # X周围峰值躯干旋转（弧度）
tor_sow_y = 0.0  # Y（弧度）附近的峰值躯干旋转
moveConfig = [["MaxStepX", max_step_x],
              ["MaxStepY", max_step_y],
              ["MaxStepTheta", max_step_theta],
              ["MaxStepFrequency", max_step_frequency],
              ["StepHeight", step_height],
              ["TorsoWx", tor_sow_x],
              ["TorsoWy", tor_sow_y]]


def Automatic_Identification():
    flag=0
    began_lock=time.time()
    while (1):
        naoImage = camera.getImageRemote(nameId)
        imageWidth = naoImage[0]
        imageHeight = naoImage[1]
        array = naoImage[6]
        im = Image.fromstring("RGB", (imageWidth, imageHeight), array)
        frame = np.asarray(im)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lowera = np.array([0, 0, 221])
        uppera = np.array([180, 30, 255])
        mask1 = cv2.inRange(hsv, lowera, uppera)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask1, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        newlines = cv2.HoughLines(mask, 1, np.pi / 180, 80)
        if newlines is None:
            newlines1 = [[0, 0]]
        else:
            newlines1 = newlines[:, 0, :]
        single_line_value = newlines1[0]
        if single_line_value[0] != 0:
            if single_line_value[1] <= 1.04:
                motion.move(1, 0, -0.2, moveConfig)
            elif single_line_value[1] >= 2.09:
                motion.move(1, 0, 0.2, moveConfig)
            else:

                if single_line_value[1] <= 1.57 and single_line_value[1] > 1.04:
                    print 'shuzhi',single_line_value[1]
                    if flag == 0 :
                        began = time.time()
                        print 'lock1', began - began_lock
                        if began-began_lock>=29.5:
                            flag = 1
                        print 'end'
                    motion.move(1, 0, 0.2, moveConfig)

                if single_line_value[1] <= 2.09 and single_line_value[1] > 1.57:
                    print 'shuzhi', single_line_value[1]
                    if flag == 0:
                        began = time.time()

                        print 'lock1',began - began_lock
                        if began - began_lock >= 29.5:
                            flag = 1
                        print 'end'
                    motion.move(1, 0, -0.2, moveConfig)
        else:
            motion.move(1, 0.0, 0.0, moveConfig)
        if newlines is None and flag==1:
            end = time.time()
            print 'time',end - began
            if 5 >= end - began >= 4.0:
                motion.move(1, 0.0, 0.0, moveConfig)
                break
            if end - began >= 5.5:
                break


def ready():
    global nameId
    while True:
        touch_sign=memory.getData("FrontTactilTouched")
        if touch_sign==1:
            break
    try:
        auto_life.setState("safeguard")  # 关闭自主生活模式
        posture.goToPosture("StandInit", 0.4)  # 启用一种站立姿态。
        motion.moveInit()  # 初始化移动过程。 检查机器人姿势并采取正确姿势。 这是阻塞调用。
        motion.setAngles('HeadPitch', 10 * math.pi / 180.0, 0.2)  # 调用头部，并且调整设置角度，最后一个参数是用于调整速度。
        camera.setActiveCamera(1)  # 设置当前默认活动相机，0是机器人下端摄像头 。
        resolution = vision_definitions.kQVGA  # 代表着最低分辨率1
        colorSpace = vision_definitions.kBGRColorSpace  # 代表着最小色彩空间
        fps = 30  # 设置帧数
        nameId = camera.subscribe("python_GVM", resolution, colorSpace, fps)
        # subscribe(模块名, 请求的分辨率, 请求的色彩空间, 帧数)
        # 设置曝光度模式
        camera.setCamerasParameter(nameId, 22, 2)  # 为模块当前活动摄像机设置特定参数值。
        naoImage = camera.getImageRemote(nameId)
        imageWidth = naoImage[0]
        imageHeight = naoImage[1]
        array = naoImage[6]
        im = Image.fromstring("RGB", (imageWidth, imageHeight), array)
        frame = np.asarray(im)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 由RGB空间转化为hsv，hsv图像适合用于图像分割，

    except:
        for x in range(10):
            time.sleep(1)
            i_can_speak.say("please restart, my system taking is wronging")


def robot_socket_broadcast(port, host, Message="111"):
    """
    发送端
    :param port:port目的端口
    :param host: host目的ip
    :param Message: 发送的消息
    :return: 成功返回1
    """
    address = (host, port)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # 创建一个udp——socket
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    msg = Message
    s.sendto(msg, address)
    time.sleep(0.1)
    s.close()
    return 1


def robot_socket_client(port, host=""):
    """
    接收端
    :param port:port接收端自己的端口
    :param host: host接收端自己的ip
    :return: data
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    address = (host, port)
    s.bind(address)
    data, addr = s.recvfrom(2048)
    s.close()
    return data, addr

if __name__ == '__main__':
    ready()
    data, Drr = robot_socket_client(7788,"192.168.2.102")
    print Drr
    Automatic_Identification()
    motion.rest()
