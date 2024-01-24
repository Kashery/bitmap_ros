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
        return self.file['dock1']['x']*100, self.file['dock1']['y']*100, self.file['dock2']['x']*100, self.file['dock2']['y']*100
    
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


class BitmapNode(Node):

    def __init__(self):
        super().__init__('bitmap_node')
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.subscription = self.create_subscription(
            Pose2D,
            'pos_track',
            self.listener_callback,
            10)
        self.add_msg_to_info_logger("initializing node")
        self.image_publisher = self.create_publisher(Image, 'image', 10)
        self.JSON_PATH = "/home/ros/BM_ws/src/bitmap/bitmap/bitmap_setup.json"
        self.jConfig = Settings(self.JSON_PATH)
        self.XSCALING_FACTOR, self.YSCALING_FACTOR = self.jConfig.GetScaling()
        self.DOCK_WIDTH = self.XSCALING_FACTOR * self.jConfig.GetDockWidth()[1]
        self.DOCK_LENGTH = self.YSCALING_FACTOR * self.jConfig.GetDockLenght()[1]
        self.docks = 2
        self.bridge = CvBridge()
        self.xL = []
        self.yL = []
        self.xDock = [None] * 2
        self.yDock = [None] * 2
        self.thetaDock = [None] * 2
        self.BITMAP_PATH = self.jConfig.GetBitmapPath()
        self.xDock[0], self.yDock[0], self.xDock[1], self.yDock[1]  = self.jConfig.GetDockPosition()

        self.xDock[0] = self.xDock[0] * self.XSCALING_FACTOR
        self.yDock[0] = self.yDock[0] * self.YSCALING_FACTOR
        self.xDock[1] = self.xDock[1] * self.XSCALING_FACTOR
        self.yDock[1] = self.yDock[1] * self.YSCALING_FACTOR
        self.thetaDock[0], self.thetaDock[1] = float(self.jConfig.GetDockOrientation()[0])*np.pi/180, float(self.jConfig.GetDockOrientation()[1])*np.pi/180
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        cv2.setWindowProperty("image", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.moveWindow("image", 1920, 0)
        self.theta = 0.0
        self.font = cv2.FONT_HERSHEY_SIMPLEX 

        self.text = "test: test\ntest:test"
        
        # org 
        
        
        # fontScale 
        self.fontScale = 1.2
        
        # Red color in BGR 
        self.color = (0, 0, 0) 
        
        # Line thickness of 2 px 
        self.thickness = 2
        self.dy = 35

        self.toggle = 0
        self.grid = True
        self.negative = True

    def draw_tail(self, im, xL, yL):
        xp = None
        yp = None
        for x,y in zip(xL ,yL):
            if xp is not None:
                cv2.line(im, (int(x),int(y)), (int(xp),int(yp)), (255,0,0), 2)
            xp = x
            yp = y
        return im
    def create_dock(self, im,x,y,theta):
        p1 = (int((self.DOCK_LENGTH/2)*np.cos(theta) - (self.DOCK_WIDTH/2)*np.sin(theta) + x), int((self.DOCK_WIDTH/2)*np.cos(theta) + (self.DOCK_LENGTH/2)*np.sin(theta) + y))
        p2 = (int((self.DOCK_LENGTH/2)*np.cos(theta) - (-self.DOCK_WIDTH/2)*np.sin(theta) + x),  int((-self.DOCK_WIDTH/2)*np.cos(theta) + (self.DOCK_LENGTH/2)*np.sin(theta) + y))
        p3 = (int((-self.DOCK_LENGTH/2)*np.cos(theta) - (-self.DOCK_WIDTH/2)*np.sin(theta) + x), int((-self.DOCK_WIDTH/2)*np.cos(theta) + (-self.DOCK_LENGTH/2)*np.sin(theta) + y))
        p4 = (int((-self.DOCK_LENGTH/2)*np.cos(theta) - (self.DOCK_WIDTH/2)*np.sin(theta) + x), int((self.DOCK_WIDTH/2)*np.cos(theta) + (-self.DOCK_LENGTH/2)*np.sin(theta) + y))
        #print(f'p1 {p1}\np2 {p2}\np3 {p3}\np4 {p4}')
        #print(f'width {DOCK_WIDTH}\nheight {DOCK_LENGTH}')
        #print(f'{np.sin(theta)/2}')
        cv2.circle(im, (int(x), int(y)), 3, (255,0,0), -1)
        cv2.line(im, p1, p4, (255,0,0), 10)
        cv2.line(im, p2, p3, (255,0,0), 10)
        cv2.line(im, p3, p4, (255,0,0), 10)
        return im
    
    def draw_grid(self, im):
        x_off = 50 * self.XSCALING_FACTOR
        y_off = 50 * self.YSCALING_FACTOR
        x =0
        y=0
        while(x < 1920):
            cv2.line(im, (int(x),0), (int(x), 1054),(255,0,0), 1)
            x += x_off
        while(y < 1054):
            cv2.line(im, (0,int(y)), (1920, int(y)),(255,0,0), 1)
            y += y_off
        return im
    
    def publish(self, cv2_image):
        try:
             msg = self.bridge.cv2_to_imgmsg(cv2_image, "bgr8")
             self.image_publisher_.publish(msg)
        except CvBridgeError as e:
             self.add_msg_to_info_logger(e)
        self.add_msg_to_info_logger("sending_image")
        

    def add_msg_to_info_logger(self, msg):
        self.get_logger().info(msg)
    def listener_callback(self, msg):
        
        self.yL.insert(0, msg.y)
        self.xL.insert(0, msg.x)
        self.get_logger().info(f"{self.xL}")

    def timer_callback(self):
        text = f"Dock 1 x: {self.xDock[0]/100/ self.XSCALING_FACTOR:.2f} y: {self.yDock[0]/100/ self.YSCALING_FACTOR:.2f} theta: {self.thetaDock[0]*180/np.pi:.2f}\n Dock 2: x: {self.xDock[1]/100/ self.XSCALING_FACTOR:.2f} y: {self.yDock[1]/100/ self.YSCALING_FACTOR:.2f} theta: {self.thetaDock[1]*180/np.pi:.2f}"
        org = (00, 50) 
        im = cv2.imread(self.BITMAP_PATH)
        if self.jConfig.GetTailVisibility():
            
            if len(self.xL) > self.jConfig.GetTailLimit():
                self.xL.pop()
            if len(self.yL) > self.jConfig.GetTailLimit():
                self.yL.pop()
            
            #cv2.rectangle(im,(int(XSCALING_FACTOR*x1), int(YSCALING_FACTOR*y1)), (int(XSCALING_FACTOR*x2), int(YSCALING_FACTOR*y2)), (255, 0, 0), -1)
            for i in range(0,self.docks):
                im = self.create_dock(im, self.xDock[i], self.yDock[i], self.thetaDock[i])
            im = self.draw_tail(im, self.xL, self.yL)
        else:
            for i in range(0,self.docks):
                im = self.create_dock(im, self.xDock[i], self.yDock[i], self.thetaDock[i])
        for i, line in enumerate(text.split('\n')):
            org= (org[0],org[1] + self.dy)
            im = cv2.putText(im, line, org, self.font, self.fontScale, 
                  self.color, self.thickness, cv2.LINE_AA, False)
        if im is None:
            self.image_publisher.add_msg_to_info_logger("jpeg?")
            return
        if self.grid:
            im = self.draw_grid(im)
        if self.negative:
            im = abs(255-im)
        cv2.imshow("image", im)
        pressed = cv2.waitKey(1)
        if pressed == ord('d'):
            self.yDock[self.toggle] = self.yDock[self.toggle] + 10 * np.cos(self.thetaDock[self.toggle])
            self.xDock[self.toggle] = self.xDock[self.toggle] - 10 * np.sin(self.thetaDock[self.toggle])
        if pressed == ord('w'):
            self.xDock[self.toggle] = self.xDock[self.toggle] - 10 * np.cos(self.thetaDock[self.toggle])
            self.yDock[self.toggle] = self.yDock[self.toggle] - 10 * np.sin(self.thetaDock[self.toggle])
        if pressed == ord('a'):
            self.yDock[self.toggle] = self.yDock[self.toggle] - 10 * np.cos(self.thetaDock[self.toggle])
            self.xDock[self.toggle] = self.xDock[self.toggle] + 10 * np.sin(self.thetaDock[self.toggle])
        if pressed == ord('s'):
            self.xDock[self.toggle] = self.xDock[self.toggle] + 10 * np.cos(self.thetaDock[self.toggle])
            self.yDock[self.toggle] = self.yDock[self.toggle] + 10 * np.sin(self.thetaDock[self.toggle])
        if pressed == ord(','):
            self.thetaDock[self.toggle] = self.thetaDock[self.toggle] + 1/180 * np.pi
            if self.thetaDock[self.toggle] >= 360/180 * np.pi:
                self.thetaDock[self.toggle] = 0
        if pressed == ord('.'):
            self.thetaDock[self.toggle] = self.thetaDock[self.toggle] - 1/180 * np.pi
            if self.thetaDock[self.toggle] < 0:
                self.thetaDock[self.toggle] = 359/180 * np.pi
        if pressed == 27 or pressed == ord('q'):
        # closing application if esc or q pressed
            self.image_publisher.add_msg_to_info_logger("closing button pressed, closing")
            return
        if pressed == ord('1'):
            self.toggle = 0
        if pressed == ord('2'):
            self.toggle = 1
        if pressed == ord('['):
            self.grid = not self.grid
        if pressed == ord(']'):
            self.negative = not self.negative
            

        elif pressed == 32:
        # publishing image if space pressed
            self.image_publisher.publish(im)


def main(args=None):
    rclpy.init(args=args)

    bitmap_node = BitmapNode()

    rclpy.spin(bitmap_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bitmap_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()