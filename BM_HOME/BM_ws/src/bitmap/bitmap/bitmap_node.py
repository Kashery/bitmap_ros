import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from random import random
import json


from geometry_msgs.msg import Pose2D

class Settings:
    def __init__(self, JSON_PATH):
        json_file = open(JSON_PATH)
        self.file = json.load(json_file)
    
    def GetBitmapPath(self):
        return self.file['bitmap_file']
    
    def GetScaling(self):
        return self.file['scaling']['x'], self.file['scaling']['y']

    def GetDockPosition(self):
        return self.file['dock1']['x'], self.file['dock1']['y'], self.file['dock2']['x'], self.file['dock2']['y']
    
    def GetDockOrientation(self):
        return self.file['dock1']['theta'], self.file['dock2']['theta']
    
    def GetDockLenght(self):
        return self.file['dock1']['length'], self.file['dock2']['length']

    def GetDockWidth(self):
        return self.file['dock1']['width'], self.file['dock2']['width']
    
    def GetTailLimit(self):
        return self.file['tail']['samples_limit']

    def GetTailVisibility(self):
        return self.file['tail']['visible']


JSON_PATH = "/home/ros/BM_ws/src/bitmap/bitmap/bitmap_setup.json"
jConfig = Settings(JSON_PATH)


# XSCALING_FACTOR = 400/103
# YSCALING_FACTOR = XSCALING_FACwwwwwwdddddTOR * 100/96.5
# DOCK_WIDTH = 18 * XSCALING_FACTOR
# DOCK_LENGTH = 22 * YSCALING_FACTOR

XSCALING_FACTOR, YSCALING_FACTOR = jConfig.GetScaling()
DOCK_WIDTH = XSCALING_FACTOR * jConfig.GetDockWidth()[1]
DOCK_LENGTH = YSCALING_FACTOR * jConfig.GetDockLenght()[1]
docks = 2
def draw_tail(im, xL, yL):
    xp = None
    yp = None
    for x,y in zip(xL ,yL):
        if xp is not None:
            cv2.line(im, (int(x),int(y)), (int(xp),int(yp)), (255,0,0), 2)
        xp = x
        yp = y
    return im

def create_dock(im,x,y,theta):
    p1 = (int((DOCK_LENGTH/2)*np.cos(theta) - (DOCK_WIDTH/2)*np.sin(theta) + x), int((DOCK_WIDTH/2)*np.cos(theta) + (DOCK_LENGTH/2)*np.sin(theta) + y))
    p2 = (int((DOCK_LENGTH/2)*np.cos(theta) - (-DOCK_WIDTH/2)*np.sin(theta) + x),  int((-DOCK_WIDTH/2)*np.cos(theta) + (DOCK_LENGTH/2)*np.sin(theta) + y))
    p3 = (int((-DOCK_LENGTH/2)*np.cos(theta) - (-DOCK_WIDTH/2)*np.sin(theta) + x), int((-DOCK_WIDTH/2)*np.cos(theta) + (-DOCK_LENGTH/2)*np.sin(theta) + y))
    p4 = (int((-DOCK_LENGTH/2)*np.cos(theta) - (DOCK_WIDTH/2)*np.sin(theta) + x), int((DOCK_WIDTH/2)*np.cos(theta) + (-DOCK_LENGTH/2)*np.sin(theta) + y))
    #print(f'p1 {p1}\np2 {p2}\np3 {p3}\np4 {p4}')
    #print(f'width {DOCK_WIDTH}\nheight {DOCK_LENGTH}')
    #print(f'{np.sin(theta)/2}')
    cv2.circle(im, (int(x), int(y)), 3, (255,0,0), -1)
    cv2.line(im, p1, p4, (255,0,0), 10)
    cv2.line(im, p2, p3, (255,0,0), 10)
    cv2.line(im, p3, p4, (255,0,0), 10)
    
    return im

def draw_grid(im):
    x_off = 50 * XSCALING_FACTOR
    y_off = 50 * YSCALING_FACTOR
    x =0
    y=0
    while(x < 1920):
        cv2.line(im, (int(x),0), (int(x), 1054),(255,0,0), 1)
        x += x_off
    while(y < 1054):
        cv2.line(im, (0,int(y)), (1920, int(y)),(255,0,0), 1)
        y += y_off
    return im
class Image_Publisher(Node):
    def __init__(self):
        super().__init__("image_tutorial")
        self.add_msg_to_info_logger("initializing node")
        self.image_publisher_ = self.create_publisher(Image, 'image', 10)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Pose2D,
            '/pos_track',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info("test")

    def publish(self, cv2_image):
        try:
             msg = self.bridge.cv2_to_imgmsg(cv2_image, "bgr8")
             self.image_publisher_.publish(msg)
        except CvBridgeError as e:
             self.add_msg_to_info_logger(e)
        self.add_msg_to_info_logger("sending_image")
        

    def add_msg_to_info_logger(self, msg):
        self.get_logger().info(msg)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('pos_track')
        self.subscription = self.create_subscription(
            Pose2D,
            'pos_track',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info("test")


def main():  

    xL = []
    yL = []
    xDock = [None] * 2
    yDock = [None] * 2
    BITMAP_PATH = jConfig.GetBitmapPath()
    xDock[0], yDock[0], xDock[1], yDock[1]  = jConfig.GetDockPosition()
    

    xDock[0] = xDock[0] * XSCALING_FACTOR
    yDock[0] = yDock[0] * YSCALING_FACTOR
    xDock[1] = xDock[1] * XSCALING_FACTOR
    yDock[1] = yDock[1] * YSCALING_FACTOR
    thetaDock1, thetaDock2 = jConfig.GetDockOrientation()

    theta = 0.0
    # x1, y1 = [250, 100] 
    # x2, y2 = [350, 200]

    rclpy.init()
    image_publisher = Image_Publisher()
    minimal_subscriber = MinimalSubscriber()
    

    image_publisher.add_msg_to_info_logger("initializing camera")
    cv2.namedWindow("image", cv2.WINDOW_NORMAL)
    cv2.setWindowProperty("image", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.moveWindow("image", 1920, 0) 
    # font 
    font = cv2.FONT_HERSHEY_SIMPLEX 

    text = "test: test\ntest:test"
    
    # org 
    
    
    # fontScale 
    fontScale = 1.2
    
    # Red color in BGR 
    color = (0, 0, 0) 
    
    # Line thickness of 2 px 
    thickness = 2
    dy = 35

    toggle = 0
    grid = True
    negative = True
    while True:
        text = f"Dock 1 x: {xDock[0]:.2f} y: {yDock[0]:.2f} theta: {thetaDock1:.2f}\n Dock 2: x: {xDock[1]:.2f} y: {yDock[1]:.2f} theta: {thetaDock1:.2f}"
        org = (00, 50) 
        im = cv2.imread(BITMAP_PATH)
        if jConfig.GetTailVisibility():
            xL.insert(0, xDock[0])
            if len(xL) > jConfig.GetTailLimit():
                xL.pop()
            if len(yL) > jConfig.GetTailLimit():
                yL.pop()
            yL.insert(0, yDock[0])
            #cv2.rectangle(im,(int(XSCALING_FACTOR*x1), int(YSCALING_FACTOR*y1)), (int(XSCALING_FACTOR*x2), int(YSCALING_FACTOR*y2)), (255, 0, 0), -1)
            for i in range(0,docks):
                im = create_dock(im, xDock[i], yDock[i], thetaDock1)
            im = draw_tail(im, xL, yL)
        else:
            for i in range(0,docks):
                im = create_dock(im, xDock[i], yDock[i], thetaDock1)
        for i, line in enumerate(text.split('\n')):
            org= (org[0],org[1] + dy)
            im = cv2.putText(im, line, org, font, fontScale, 
                  color, thickness, cv2.LINE_AA, False)
        if im is None:
            image_publisher.add_msg_to_info_logger("jpeg?")
            break
        if grid:
            im = draw_grid(im)
        if negative:
            im = abs(255-im)
        cv2.imshow("image", im)
        pressed = cv2.waitKey(1)
        if pressed == ord('d'):
            yDock[toggle] = yDock[toggle] + 10 * np.cos(thetaDock1)
            xDock[toggle] = xDock[toggle] - 10 * np.sin(thetaDock1)
        if pressed == ord('w'):
            xDock[toggle] = xDock[toggle] - 10 * np.cos(thetaDock1)
            yDock[toggle] = yDock[toggle] - 10 * np.sin(thetaDock1)
        if pressed == ord('a'):
            yDock[toggle] = yDock[toggle] - 10 * np.cos(thetaDock1)
            xDock[toggle] = xDock[toggle] + 10 * np.sin(thetaDock1)
        if pressed == ord('s'):
            xDock[toggle] = xDock[toggle] + 10 * np.cos(thetaDock1)
            yDock[toggle] = yDock[toggle] + 10 * np.sin(thetaDock1)
        if pressed == 27 or pressed == ord('q'):
        # closing application if esc or q pressed
            image_publisher.add_msg_to_info_logger("closing button pressed, closing")
            break
        if pressed == ord('1'):
            toggle = 0
        if pressed == ord('2'):
            toggle = 1
        if pressed == ord('['):
            grid = not grid
        if pressed == ord(']'):
            negative = not negative
            

        elif pressed == 32:
        # publishing image if space pressed
            image_publisher.publish(im)
    # closing  
    minimal_subscriber.destroy_node()

    image_publisher.destroy_node()
    rclpy.shutdown()   
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()