# coding=utf-8
import sys,time,cv2,math,socket
import numpy as np
import vision_definitions
from PIL import Image
from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule
from optparse import OptionParser


IP = "192.168.2.103"
PORT = 9559
IP2="192.168.2.105"

max_step_x = 0.100  # 最大正向平移沿X（米）0.035
max_step_y = 0.142  # 沿Y方向的绝对最大横向平移（米）0.12
max_step_theta = 0.2  # 绕Z轴绝对最大旋转（弧度）0.4
max_step_frequency = 0.9 # 最大步进频率（标准化，无单位）0.7
step_height = 0.021  # Z轴高程（米）0.027
tor_sow_x = 0.0  # X周围峰值躯干旋转（弧度）
tor_sow_y = 0.0  # Y（弧度）附近的峰值躯干旋转


moveConfig = [["MaxStepX", max_step_x],
              ["MaxStepY", max_step_y],
              ["MaxStepTheta", max_step_theta],
              ["MaxStepFrequency", max_step_frequency],
              ["StepHeight", step_height],
              ["TorsoWx", tor_sow_x],
              ["TorsoWy", tor_sow_y]]


motion = ALProxy("ALMotion", IP, PORT)
camera = ALProxy("ALVideoDevice", IP, PORT)
red_ball = ALProxy("ALRedBallDetection", IP, PORT)
landmark = ALProxy("ALLandMarkDetection", IP, PORT)
memory = ALProxy("ALMemory", IP, PORT)
tts = ALProxy("ALTextToSpeech", IP, PORT)
posture = ALProxy("ALRobotPosture", IP, PORT)
auto_life = ALProxy("ALAutonomousLife", IP, PORT)
i_can_speak=ALProxy("ALTextToSpeech", IP, PORT)

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
            if 5 >= end - began >= 3.5:
                motion.move(1, 0.0, 0.0, moveConfig)
                break
            if end - began >= 5.0:
                break

def ready():
    global nameId
    try:
        auto_life.setState("safeguard")
        posture.goToPosture("StandInit", 0.6)
        motion.moveInit()
        motion.setAngles('HeadPitch',10 * math.pi / 180.0, 0.2)
        camera.setActiveCamera(1)
        resolution = vision_definitions.kQVGA
        colorSpace = vision_definitions.kBGRColorSpace
        fps = 30
        nameId = camera.subscribe("python_GVM", resolution, colorSpace, fps)
        camera.setCamerasParameter(nameId, 22, 2)
        naoImage = camera.getImageRemote(nameId)
        imageWidth = naoImage[0]
        imageHeight = naoImage[1]
        array = naoImage[6]
        im = Image.fromstring("RGB", (imageWidth, imageHeight), array)
        frame = np.asarray(im)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    except:
        for x in range(3):
            i_can_speak.say("plaese resart, my system taking is wronging")
    while True:
        touch_sign=memory.getData("FrontTactilTouched")
        if touch_sign==1:
            i_can_speak.say("i am ready")
            break

def robot_socket_broadcast(port, host, Message="111"):
    address = (host, port)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    msg = Message
    s.sendto(msg, address)
    time.sleep(0.1)
    s.close()
    return 1

def main():
    Automatic_Identification()
    if robot_socket_broadcast(7788, "192.168.2.105", "ok") is 1:
        print 'ok'
    motion.rest()
class HumanGreeterModule(ALModule):
    def __init__(self, name):
        ALModule.__init__(self, name)
        self.sound = ALProxy("ALSoundDetection")
        global memory
        memory = ALProxy("ALMemory")
        self.sound.setParameter("Sensitivity", 0.4)
        memory.subscribeToEvent("SoundDetected",
                                "HumanGreeter",
                                "onSoundDetected")
    def onSoundDetected(self, *_args):
        memory.unsubscribeToEvent("SoundDetected",
                                  "HumanGreeter")
        main()
def listen():
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

    myBroker = ALBroker("myBroker",
                        "0.0.0.0",
                        0,
                        pip,
                        pport)

    global HumanGreeter
    HumanGreeter = HumanGreeterModule("HumanGreeter")
    try:
        while True:
            time.sleep(0.01)
    except KeyboardInterrupt:
        print "Interrupted by user, shutting down"
        myBroker.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    ready()
    listen()