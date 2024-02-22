#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_about_axis


class PublisherNode:
    def __init__(self):
        rospy.init_node('publisher_node', anonymous=True)

        self.odom_publisher = rospy.Publisher('odom', Odometry, queue_size=10)
        self.quaternion_publisher = rospy.Publisher(
            'orientation', Quaternion, queue_size=10)

        self.odom_timer = rospy.Timer(
            rospy.Duration(0.01), self.publish_odom)  # 100 Hz
        self.quaternion_timer = rospy.Timer(
            rospy.Duration(0.02), self.publish_quaternion)  # 50 Hz

        self.current_angle = 0

    def publish_odom(self, event):
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        # Deja la orientación vacía
        self.odom_publisher.publish(odom_msg)

    def publish_quaternion(self, event):
        quaternion_msg = Quaternion()
        # Aumenta el ángulo en 15 grados en cada iteración
        self.current_angle += 15
        if self.current_angle >= 360:
            self.current_angle -= 360

        # Genera un cuaternión para una rotación de 15 grados alrededor del eje Z
        quaternion_msg = quaternion_about_axis(
            self.current_angle * 0.0174533, (0, 0, 1))
        self.quaternion_publisher.publish(quaternion_msg)


if __name__ == '__main__':
    try:
        publisher_node = PublisherNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
