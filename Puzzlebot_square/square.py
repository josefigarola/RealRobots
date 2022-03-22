#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from math import pi


class Square_controller:
    def __init__(self):
        self.__wl = 0.0
        self.__wr = 0.0

        rospy.Subscriber("/wl", Float32, callback=self.__callback_wl)
        rospy.Subscriber("/wr", Float32, callback=self.__callback_wr)
        self.__pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.init_node("square_controller")
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.__stop)

    def __callback_wl(self, wl):
        self.__wl = wl.data

    def __callback_wr(self, wr):
        self.__wr = wr.data


    def main(self):
        distance = 0.0
        angle = 0.0
        current_time = rospy.get_time()
        last_time = rospy.get_time()
        state = 0
        count = 1
        n_sides = 4

        speed_vec = Twist()
        speed_vec.linear.x = 0.0
        speed_vec.linear.y = 0.0
        speed_vec.linear.z = 0.0
        speed_vec.angular.x = 0.0
        speed_vec.angular.y = 0.0
        speed_vec.angular.z = 0.0

        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            delta_time = current_time - last_time
            last_time = current_time

            distance += 0.05 * (self.__wr + self.__wl) *0.5 * delta_time
            angle += 0.05 * (self.__wr - self.__wl) / 0.18 * delta_time
            self.__wr = 0.0
            self.__wl = 0.0

            if state == 0:
                speed_vec.linear.x = 0.5
                speed_vec.angular.z = 0.0

                if distance > 0.5:
                    distance = 0.0

                    if count < n_sides:
                        state = 1
                        count += 1

                    else:
                        state = 2
            elif state == 1:
                speed_vec.linear.x = 0.0
                speed_vec.angular.z = 0.5

                if angle > pi/2:
                    angle = 0.0
                    state = 0
            
            elif state == 2:
                speed_vec.linear.x = 0
                speed_vec.angular.z =0
                rospy.signal_shutdown("Path completed")
            else:
                speed_vec.linear.x = 0
                speed_vec.angular.z =0
            
            self.__pub_cmd_vel.publish(speed_vec)
            self.rate.sleep()
    
    def __stop(self):
        speed_vec = Twist()
        speed_vec.linear.x = 0.0
        speed_vec.linear.y = 0.0
        speed_vec.linear.z = 0.0
        speed_vec.angular.x = 0.0
        speed_vec.angular.y = 0.0
        speed_vec.angular.z = 0.0
        self.__pub_cmd_vel.publish(speed_vec)

if __name__ == "__main__":
    controller = Square_controller()
    controller.main()
