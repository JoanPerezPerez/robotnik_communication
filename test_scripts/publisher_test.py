import rospy
from std_msgs.msg import String

rospy.init_node("test_publisher")

pub = rospy.Publisher("/mi_test_topic", String, queue_size=10)

rate = rospy.Rate(1)

while not rospy.is_shutdown():
    msg = String()
    msg.data = "hola desde devcontainer"
    pub.publish(msg)

    print("publicado")
    rate.sleep()