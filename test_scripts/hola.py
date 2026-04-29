import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

i = 0

def callback(msg):
    global i
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")

    path = f"/tmp/frame_{i}.jpg"
    cv2.imwrite(path, frame)

    print("Guardada:", path)
    i += 1

def main():
    rospy.init_node("camera_listener")

    rospy.Subscriber("/robot/front_rgbd_camera/color/image_raw", Image, callback)

    rospy.spin()

if __name__ == "__main__":
    main()