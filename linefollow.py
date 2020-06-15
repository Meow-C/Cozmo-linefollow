import rospy, cv2, cv_bridge, numpy, math, datetime
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Follower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        # cv2.namedWindow('window', 1)

        # subscrib the topic from ros to get the image 
        self.image_sub = rospy.Subscriber('/cozmo_camera/image', Image, self.image_callback)
        # publish the topic to control the cozmo
        # Twist is the data format commonly used in ROS to control the motion of robots
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.twist = Twist()
        self.err = 0
        self.turn_cmd = Twist()

        
    def image_callback(self, msg):
        # To get the image from the camera and convert it to binary image by using opencv
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        '''
        use Twist to control Cozmo
        linear.x go ahead, m/s
        angular.z turn the cozmo, radian
        '''
        # self.twist.linear.x = 0.03
        # self.twist.angular.z = 0.74
        # self.cmd_vel_pub.publish(self.twist)

        '''s
        here, deal with the image, and control cozmo
        '''

        cv2.imshow('window',img)
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('follow')
    follow = Follower()
    rospy.spin()
