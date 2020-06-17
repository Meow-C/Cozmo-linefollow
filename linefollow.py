import rospy, cv2, cv_bridge, numpy, math, datetime
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class Follower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        # cv2.namedWindow('window', 1)

        self.image_sub = rospy.Subscriber('/cozmo_camera/image', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.lift_height_pub = rospy.Publisher('/lift_height', Float64, queue_size=1)

        self.twist = Twist()
        self.lift_height = Float64(data=20)
        self.err = 0
        self.cnt = 0

        self.lift_height_pub.publish(20)


    def image_callback(self, msg):
        # To get the image from the camera and convert it to binary image by using opencv
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        cv2.imshow('window_img',img)
        cv2.waitKey(3)

        # ROI
        # cropped = img[y0:y1, x0:x1]
        cropped = img[160:210, 20:340]

        cv2.imshow('window_cropped',cropped)
        cv2.waitKey(3)

        # Binarization
        gray = cv2.cvtColor(cropped, cv2.COLOR_RGB2GRAY)
        ret, binimg = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)

        cv2.imshow('window_BIN', binimg)
        cv2.waitKey(3)

        # calculate center of bounding box
        y, x = (binimg < 125).nonzero()

        # Judge center of line
        x0 = 160
        if x.shape[0] <> 0:
            y0 = (max(y) + min(y))/2
            x0 = (max(x) + min(x))/2
            # print '(x0,y0):', x0, y0

        # print 'x.shape[0]:',  x.shape[0]
        if x.shape[0] < 100:
            self.cnt  += 1
            if self.cnt > 30:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                print 'Stop!'
        else:    
            self.cnt = 0
            # control
            err = x0 - 160
            print 'err:', err
            self.twist.linear.x = 0.01
            self.twist.angular.z = 0.0
            if abs(err) > 5:
                self.twist.angular.z = 0.74 * -err * 0.006
                print 'angular.z', self.twist.angular.z
                print 'Turn'
            else:
                print 'Go ahead!'
        self.cmd_vel_pub.publish(self.twist)
        self.lift_height_pub.publish(20)


if __name__ == '__main__':
    rospy.init_node('follow')
    follow = Follower()
    rospy.spin()
