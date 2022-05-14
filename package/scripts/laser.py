#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image,LaserScan
import numpy as np
from time import sleep
from cv_bridge import CvBridge,CvBridgeError

scan_pub = rospy.Publisher('laz_val', LaserScan, queue_size=50)

# https://github.com/IntelRealSense/realsense-ros/blob/development/realsense2_camera/scripts/show_center_depth.py
class DepthImage:
    def __init__(self, depth_image_topic):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(depth_image_topic, Image, self.imageDepthCallback)
        
    def imageDepthCallback(self, data):
        try:
            # Convert to cv to np array and take central row
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            depth_in_mm = np.array(cv_image)
            line = depth_in_mm[data.height//2]


            # Appropriated from https://gist.github.com/atotto/c47bc69a48ed38e86947b5506b8e0e61
            while not rospy.is_shutdown():
                current_time = rospy.Time.now()

                scan = LaserScan()

                # This is device specific i guess
                scan.header.stamp = current_time
                scan.header.frame_id = 'laser_frame'
                scan.angle_min = -1.57
                scan.angle_max = 1.57
                scan.angle_increment = 3.14 / (data.width//2)
                scan.time_increment = (1.0 / 15) / (data.width//2) # 15: fps i think
                scan.range_min = 0.0 # in mm
                scan.range_max = 25000.0 # in mm

                scan.ranges = []
                scan.intensities = []
                for i in range(0, data.width):
                    scan.ranges.append(line[i])
                    scan.intensities.append(1)

                scan_pub.publish(scan)

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

def listener():
    rospy.init_node('read_and_write_laz', anonymous=True)    
    listener = DepthImage('/camera/depth/image_rect_raw')
    rospy.spin()

if __name__ == '__main__':
    listener()
