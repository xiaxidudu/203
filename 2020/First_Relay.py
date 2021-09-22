# coding=utf-8
import sys
import time
import cv2
import numpy as np
from PIL import Image
import math
import socket
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule
from optparse import OptionParser
import threading

IP = "192.168.2.102"
PORT = 9559
message = [0, 0, 0]
flag = 0
Y = 0.0
Z = 0.0
times = 5
dev = 0.0
red_ball = ALProxy("ALRedBallDetection", IP, PORT)
landmark = ALProxy("ALLandMarkDetection", IP, PORT)
motion = ALProxy("ALMotion", IP, PORT)
memory = ALProxy("ALMemory", IP, PORT)
tts = ALProxy("ALTextToSpeech", IP, PORT)
camera = ALProxy("ALVideoDevice", IP, PORT)
posture = ALProxy("ALRobotPosture", IP, PORT)
auto_life = ALProxy("ALAutonomousLife", IP, PORT)

max_step_x = 0.075  # 最大正向平移沿X（米）0.035
max_step_y = 0.142  # 沿Y方向的绝对最大横向平移（米）0.12
max_step_theta = 0.2  # 绕Z轴绝对最大旋转（弧度）0.4
max_step_frequency = 0.8  # 最大步进频率（标准化，无单位）0.7
step_height = 0.026  # Z轴高程（米）0.027
tor_sow_x = 0.0  # X周围峰值躯干旋转（弧度）
tor_sow_y = 0.0  # Y（弧度）附近的峰值躯干旋转

ts = [1, 0]  # 摄像头

# Global variable to store the HumanGreeter module instance
HumanGreeter = None
memory = None
motion.setMoveArmsEnabled(True, True)


# 听觉
def straight_move(x, y=0.0, z=0.0):
    """
    # posture.goToPosture 是进入走路准备姿势
    # motion.setMoveArmsEnabled 是设置走路时不摆臂，这个你们自己也搜得到
    # motion.moveTo就是走路的函数，分别是向前的距离x, 横移的距离y, 和角度
    # 下面定义的就是走路的参数，这个要自己更具实际情况设定
    :return: 无返回
    """
    print "Y:" + str(Y) + " y:" + str(y)
    print "Z:" + str(z)
    # posture.goToPosture("StandInit", 0.25)
    motion.moveTo(x, y, z,
                  [["MaxStepX", max_step_x],
                   ["MaxStepY", max_step_y],
                   ["MaxStepTheta", max_step_theta],
                   ["MaxStepFrequency", max_step_frequency],
                   ["StepHeight", step_height],
                   ["TorsoWx", tor_sow_x],
                   ["TorsoWy", tor_sow_y]])


def find_white_line(img):
    """
    视觉部分，对白线进行分析
    传入一张图片img，返回三个白线的分析结果
    :param img: 传入的类型为np.array
    :return:返回三个元组
    三个元组分别对应机器人识别到的前、左、右各方向的白线，元组中数据分别为：线段两端点的 X、Y 坐标，分别为 X1、Y1、X2、Y2。最后一个数据为线段长度
    """
    # path = "/home/pipibao/Pictures/ProjectImage/WhiteLine/1.png"
    # img = cv2.imread(path)
    # cv2.imshow("11", img)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur_gray = cv2.GaussianBlur(gray, (5, 5), 0)
    hsv_image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    low_range = np.array([0, 0, 221])
    high_range = np.array([180, 30, 255])
    hsv_range = cv2.inRange(hsv_image, low_range, high_range)
    kernel = np.ones((4, 5), np.uint8)
    closed = cv2.morphologyEx(hsv_range, cv2.MORPH_CLOSE, kernel)
    # cv2.imshow("closed", closed)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    minLineLength = 20
    maxLineGap = 5
    lines = cv2.HoughLinesP(closed, 1, np.pi / 180.0, minLineLength, maxLineGap)
    print lines
    if lines==None:
        lines=[]
    # 三个元组分别对应机器人识别到的前、左、右各方向的白线，元组中数据分别为：线段两端点的 X、Y 坐标，分别为 X1、Y1、X2、Y2。最后一个数据为线段长度
    front_line = (0, 0, 0, 0, 0.0)
    left_line = (0, 0, 0, 0, 0.0)
    right_line = (0, 0, 0, 0, 0.0)
    X1 = 0
    Y1 = 1
    X2 = 2
    Y2 = 3
    LENGTH = 4
    head_pitch = 1 * math.pi / 180.0
    bottom_camera_height = 46.0
    cnt = 0
    for line in lines:
        cnt += 1
        for x1, y1, x2, y2 in line:
            length = math.sqrt(float((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)))
            if (math.fabs(x2 - x1) / 320.0 - math.fabs(y2 - y1) / 240.0 > math.fabs(x2 - x1) / 320.0 * 0.4):
                if (length > front_line[LENGTH]):
                    front_line = (x1, y1, x2, y2, length)
            elif (y2 > y1 and length > right_line[LENGTH]):
                right_line = (x1, y1, x2, y2, length)
            elif (y2 < y1 and length > left_line[LENGTH]):
                left_line = (x1, y1, x2, y2, length)
    if ((front_line[X2] < left_line[X2] and left_line[LENGTH] != 0.0) or (
            front_line[X1] > right_line[X1] and right_line[LENGTH] != 0.0)):
        front_line = (0, 0, 0, 0, 0.0)
    if (left_line[Y1] < front_line[Y1]):
        left_line = (0, 0, 0, 0, 0.0)
    if (right_line[Y2] < front_line[Y2]):
        right_line = (0, 0, 0, 0, 0.0)
    if (front_line[LENGTH] != 0.0):
        front_angle_y1 = math.pi / 2.0 - 0.692896 - (front_line[Y1] - 120.0) / 240.0 * 0.831475 - head_pitch
        front_distance_y1 = int(bottom_camera_height * math.tan(front_angle_y1))
        front_angle_x1 = (front_line[X1] - 120.0) / 240.0 * 1.064127
        front_distance_x1 = int(bottom_camera_height / math.cos(front_angle_y1) * math.tan(front_angle_x1))
        front_angle_y2 = math.pi / 2.0 - 0.692896 - (front_line[Y2] - 120.0) / 240.0 * 0.831475 - head_pitch
        front_distance_y2 = int(bottom_camera_height * math.tan(front_angle_y2))
        front_angle_x2 = (front_line[X2] - 120.0) / 240.0 * 1.064127
        front_distance_x2 = int(bottom_camera_height / math.cos(front_angle_y2) * math.tan(front_angle_x2))
    else:
        front_distance_x1 = front_distance_y1 = front_distance_x2 = front_distance_y2 = 0

    if (left_line[LENGTH] != 0.0):
        left_angle_y1 = math.pi / 2.0 - 0.692896 - (left_line[Y1] - 120.0) / 240.0 * 0.831475 - head_pitch
        left_distance_y1 = int(bottom_camera_height * math.tan(left_angle_y1))
        left_angle_x1 = (left_line[X1] - 120.0) / 240.0 * 1.064127
        left_distance_x1 = int(bottom_camera_height / math.cos(left_angle_y1) * math.tan(left_angle_x1))
        left_angle_y2 = math.pi / 2.0 - 0.692896 - (left_line[Y2] - 120.0) / 240.0 * 0.831475 - head_pitch
        left_distance_y2 = int(bottom_camera_height * math.tan(left_angle_y2))
        left_angle_x2 = (left_line[X2] - 120.0) / 240.0 * 1.064127
        left_distance_x2 = int(bottom_camera_height / math.cos(left_angle_y2) * math.tan(left_angle_x2))
    else:
        left_distance_x1 = left_distance_y1 = left_distance_x2 = left_distance_y2 = 0

    if (right_line[LENGTH] != 0.0):
        right_angle_y1 = math.pi / 2.0 - 0.692896 - (right_line[Y1] - 120.0) / 240.0 * 0.831475 - head_pitch
        right_distance_y1 = int(bottom_camera_height * math.tan(right_angle_y1))
        right_angle_x1 = (right_line[X1] - 120.0) / 240.0 * 1.064127
        right_distance_x1 = int(bottom_camera_height / math.cos(right_angle_y1) * math.tan(right_angle_x1))
        right_angle_y2 = math.pi / 2.0 - 0.692896 - (right_line[Y2] - 120.0) / 240.0 * 0.831475 - head_pitch
        right_distance_y2 = int(bottom_camera_height * math.tan(right_angle_y2))
        right_angle_x2 = (right_line[X2] - 120.0) / 240.0 * 1.064127
        right_distance_x2 = int(bottom_camera_height / math.cos(right_angle_y2) * math.tan(right_angle_x2))
    else:
        right_distance_x1 = right_distance_y1 = right_distance_x2 = right_distance_y2 = 0

    front_line2 = (front_distance_x1, front_distance_y1, front_distance_x2, front_distance_y2, front_line[LENGTH])
    left_line2 = (left_distance_x1, left_distance_y1, left_distance_x2, left_distance_y2, left_line[LENGTH])
    right_line2 = (right_distance_x1, right_distance_y1, right_distance_x2, right_distance_y2, right_line[LENGTH])

    return front_line[LENGTH], left_line[LENGTH], right_line[LENGTH]


def show_nao_image(ip=IP, port=9559, p0=1):  # 这个函数是从官方文档复制下来的，用于获取拍的照片  你们可以去官网搜  Retrieving images
    """
    调用机器人的api拍摄一张照片并返回
    :param ip: 当前机器人的ip
    :param port: 调用端口
    :param p0: 0是下摄像头，1是上摄像头
    :return: 返回一个图像数组
    """
    # print ip
    # print port
    time.sleep(times)
    if p0 == 0:  # 0是下摄像头，1是上摄像头。
        camera.setActiveCamera(0)
    elif p0 == 1:
        camera.setActiveCamera(1)
    resolution = 2  # VGA
    color_space = 11  # RGB

    video_client = camera.subscribe("python_client", resolution, color_space, 5)
    t0 = time.time()
    # Get a camera image.
    # image[6] contains the image data passed as an array of ASCII chars.
    nao_image = camera.getImageRemote(video_client)

    t1 = time.time()

    # Time the image transfer.
    print "acquisition delay ", t1 - t0

    camera.unsubscribe(video_client)

    # Get the image size and pixel array.
    image_width = nao_image[0]
    image_height = nao_image[1]
    array = nao_image[6]

    # Create a PIL Image from our pixel array.装换为PIL图片格式
    im = Image.fromstring("RGB", (image_width, image_height), array)  # 返回类型？
    # print type(im)
    Im = cv2.cvtColor(np.asarray(im), cv2.COLOR_RGB2BGR)
    # PTH = "/home/pipibao/Desktop/" + str(flag) + ".jpg"
    # cv2.imwrite(PTH, Im)
    return Im
    # Save the image.
    # im.save("tt2.bmp", "BMP")


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


def ready():
    auto_life.setState("safeguard")  # 关闭自主生活模式
    # posture.goToPosture("Stand", 0.25)
    posture.goToPosture("StandInit", 0.4)


def say(M):
    tts.say(M)


def rest():
    motion.rest()


def analyze():
    global message, Y, flag, Z
    print "flag:" + str(flag)
    Y = 0.0
    Z = 0.0 + dev
    if message[1] is 0.0 and message[2] is 0.0:
        return
    elif message[1] > message[2]:
        if message[2] < 20.0 and message[1] > 40.0:
            Z = Z - 0.1
            Y = Y - 0.03
            if message[1] < 200 or message[2] == 0:
                Y = Y - 0.06
                Z = Z - 0.1
        return
        Y = Y - 0.04
        if message[1] > message[2] + 40 or message[1] > message[2] * 1.5:
            Y = Y - 0.04
            Z = Z - 0.1
            if message[1] > message[2] + 80 or message[1] > message[2] * 2:
                Y = Y - 0.04
                Z = Z - 0.1
                if message[1] > message[2] + 150:
                    Y = Y - 0.04
                    Z = Z - 0.1
    elif message[2] > message[1]:
        Y = Y + 0.01
        Z = Z + 0.1
        if message[1] < 20.0 and message[2] > 40.0:
            Z = Z + 0.3
            if message[1] < 5.0:
                Y = Y + 0.01
                Z = Z + 0.2
            Y = Y + 0.03
            return
        if message[2] > message[1] + 80 or message[2] > message[1] * 2:
            Y = Y + 0.01
            Z = Z + 0.1
            if message[2] > message[1] + 150 or message[2] > message[1] * 2.5:
                Y = Y + 0.005
                Z = Z + 0.1
                if message[2] > message[1] + 250:
                    Y = Y + 0.005
                    Z = Z + 0.1


def run_thread():
    """
    调用全局变量message,flag
    message为find_white_line的返回值
    flag为1则线程结束
    :return: 无返回
    """
    global message
    message[0] = 0
    message[1] = 0
    message[2] = 0
    I = show_nao_image()
    message[0], message[1], message[2] = find_white_line(I)
    analyze()
    print "run_thread Z:" + str(Z)
    print "run_thread Y:" + str(Y)
    print "run_thread:" + str(message)


def main():
    global flag, Y
    # say("I can listen")
    print ("I can listen")
    for i in range(4):
        flag = flag + 1
        if Z != 0.0:
            print "Z:" + str(Z)
            straight_move(0.05, 0.0, Z)
        added_thread = threading.Thread(target=run_thread)
        added_thread.start()
        if i is 6:
            straight_move(x=1.35, y=Y)
        else:
            straight_move(x=1.5, y=Y,z=-0.02)

        # added_thread.join()
        print "message" + str(message)
    #
    ip = "192.168.2.102"
    if robot_socket_broadcast(7788, ip, "ok") is 1:
        say("信号发送成功")
    #
    rest()


def run_again():
    print "我摔倒了"
    global flag
    for i in range(2 - flag):
        added_thread = threading.Thread(target=run_thread)
        if flag:
            added_thread.start()
        else:
            flag = flag + 1
        straight_move(x=1, y=Y, z=0)
        # added_thread.join()
        print "message" + str(message)
    ip = "192.168.2.102"
    if robot_socket_broadcast(7788, ip, "ok") == 1:
        say("信号发送成功")
    rest()


def restore():
    # Create proxy to ALMemory
    # memoryProxy = ALProxy("ALMemory", IP, PORT)
    while True:
        time.sleep(1)
        footContact = True
        footContact = memory.getData("footContact")
        # return footContact
        if footContact is True:
            auto_life.setState("solitary")
            time.sleep(2)
            ready()
            run_again()


class HumanGreeterModule(ALModule):
    """ A simple module able to react
    to facedetection events

    """

    def __init__(self, name):
        ALModule.__init__(self, name)
        # No need for IP and port here because
        # we have our Python broker connected to NAOqi broker

        # Create a proxy to ALTextToSpeech for later use
        self.sound = ALProxy("ALSoundDetection")
        self.tts = ALProxy("ALTextToSpeech")
        # Subscribe to the FaceDetected event:
        global memory
        memory = ALProxy("ALMemory")
        self.sound.setParameter("Sensitivity", 0.4)
        memory.subscribeToEvent("SoundDetected",
                                "HumanGreeter",
                                "onSoundDetected")

    def onSoundDetected(self, *_args):
        """ This will be called each time a face is
        detected.

        """
        # Unsubscribe to the event when talking,
        # to avoid repetitions
        memory.unsubscribeToEvent("SoundDetected",
                                  "HumanGreeter")

        # self.tts.say("Hello, you")
        main()
        # Subscribe again to the event
        # memory.subscribeToEvent("SoundDetected",
        #                         "HumanGreeter",
        #                         "onSoundDetected")


def sound_peak():
    """ Main entry point

    """
    parser = OptionParser()
    parser.add_option("--pip",
                      help="Parent broker port. The IP address or your robot",
                      dest="pip")
    parser.add_option("--pport",
                      help="Parent broker port. The port NAOqi is listening to",
                      dest="pport",
                      type="int")
    parser.set_defaults(
        pip=IP,
        pport=9559)

    (opts, args_) = parser.parse_args()
    pip = opts.pip
    pport = opts.pport

    # We need this broker to be able to construct
    # NAOqi modules and subscribe to other modules
    # The broker must stay alive until the program exists
    myBroker = ALBroker("myBroker",
                        "0.0.0.0",  # listen to anyone
                        0,  # find a free port and use it
                        pip,  # parent broker IP
                        pport)  # parent broker port

    # Warning: HumanGreeter must be a global variable
    # The name given to the constructor must be the name of the
    # variable
    global HumanGreeter
    HumanGreeter = HumanGreeterModule("HumanGreeter")
    # 由它重复sleep进行，是的事件不断进行。
    try:
        while True:
            time.sleep(1)
            # print "sleep"
    except KeyboardInterrupt:
        print "Interrupted by user, shutting down"
        myBroker.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    ready()
    sound_peak()
