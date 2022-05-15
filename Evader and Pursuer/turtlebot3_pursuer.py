#!/usr/bin/env python

"""Package Imports"""
import rospy
import numpy
import matplotlib.pyplot as plt
import tf
import math
import time
""""Message imports"""
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class pursuer():
    def __init__(self,initial_position_x,initial_position_y,fwd_vel):
        self.scan=[]
        self.turtle_x = 0.0
        self.turtle_y = 0.0
        self.turtle_z = 0.0
        self.turtle_roll = 0.0
        self.turtle_pitch = 0.0
        self.turtle_yaw = 0.0
        self.turtle_w = 0.0
        self.angle_direction=[0,0]
        self.timestamp=[0,0]
        self.forward_velocity = fwd_vel
        self.turtle_x_2 = 0.0
        self.turtle_y_2 = 0.0
    #Get callback data from turtlebot scan
    def scan_callback(self,msg):
        self.scan = msg.ranges
        
        return self.scan

    def odom_pursuer_callback(self,msg): 
        self.turtle_x = msg.pose.pose.position.x
        self.turtle_y = msg.pose.pose.position.y
        self.turtle_z = msg.pose.pose.position.z
        self.turtle_roll = msg.pose.pose.orientation.x
        self.turtle_pitch = msg.pose.pose.orientation.y
        self.turtle_yaw = msg.pose.pose.orientation.z
        self.turtle_w = msg.pose.pose.orientation.w
        return self.turtle_x,self.turtle_y,self.turtle_z,self.turtle_roll,self.turtle_pitch,self.turtle_yaw,self.turtle_w

    def odom_evader_callback(self,msg): 
        self.turtle_x_2 = msg.pose.pose.position.x
        self.turtle_y_2 = msg.pose.pose.position.y
        return self.turtle_x_2,self.turtle_y_2

    #Since LaserScan has 360 elements in list for each degree presumably, locate all elements that is NOT inf and get their index
    def locate_angle(self,scan):
        angle_list=[]
        for i in range(0,len(self.scan)):
            if not self.scan:
                pass
            else:
                if self.scan[i] != float('inf'):
                    angle_list.append(i)
        if angle_list:
            angle=max(angle_list)
            return angle
        else:
            return None

    #Get the derivative of the current and previous angle
    def angle_derivative(self):
        angle_difference = self.angle_direction[1] - self.angle_direction[0]
        time_difference = self.timestamp[1]-self.timestamp[0]
        velocity = angle_difference/time_difference
        return velocity
    
    def current_turtlebot_angles(self):
        angles=tf.transformations.euler_from_quaternion([self.turtle_roll,self.turtle_pitch,self.turtle_yaw,self.turtle_w])
        yaw_turn = angles[2]
        #print(math.degrees(yaw_turn))
        return yaw_turn

    #Handles the proportional navigation which will make the turtlebot head towards the escaping turtlebot
    def proportional_navigation(self,velocity):
        self.proportional_navigation_gain = 0.0002
        desired_anglular_velocity = self.proportional_navigation_gain * velocity
        return desired_anglular_velocity

    def turtleCMD(self,publisher,go_x, turn_z):
        #Proivides a structure to apply cmds and publishes them from Twist()
        twist = Twist()
        twist.linear.x = go_x
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = turn_z
        #print(twist.linear.x,twist.angular.z)
        publisher.publish(twist)
        return twist

    def turtlebot_distance(self,x1,y1,x2,y2):
        distx=abs(x1-x2)
        disty=abs(y1-y2)
        dist=math.sqrt(pow(distx,2)+pow(disty,2))
        return dist

    #Main executable
    def main(self):
        print('Executing Main Script')
        rate = rospy.Rate(50) #hz
        scan_sub=rospy.Subscriber('/turtlebot2/scan',LaserScan,self.scan_callback)
        odom_pursuer_sub=rospy.Subscriber('/turtlebot2/odom',Odometry,self.odom_pursuer_callback)
        odom_evader_sub = rospy.Subscriber("/turtlebot1/odom",Odometry,self.odom_evader_callback)
        
        velocity_publisher = rospy.Publisher('/turtlebot2/cmd_vel', Twist, queue_size=10)
        plot_x_pursuer=[]
        plot_y_pursuer=[]
        plot_x_evader=[]
        plot_y_evader=[]
        collision_point_x=[]
        collision_point_y=[]
        collision_iteration = 0
        while not rospy.is_shutdown():
            self.angle_direction[0]=self.current_turtlebot_angles()
            
            #Gets current LOS angle and timestamp
            angle = self.locate_angle(self.scan)
            self.timestamp[1]=time.time()
            if angle != None:
                self.angle_direction[1] = angle
                plot_x_pursuer.append(self.turtle_x)
                plot_y_pursuer.append(self.turtle_y)
                plot_x_evader.append(self.turtle_x_2)
                plot_y_evader.append(self.turtle_y_2)
                dist=self.turtlebot_distance(self.turtle_x,self.turtle_y,self.turtle_x_2,self.turtle_y_2)
                #print(dist)
                if round(dist,3)<=0.150 and collision_iteration == 0:
                    collision_point_x.append(self.turtle_x)
                    collision_point_y.append(self.turtle_y)
                    collision_iteration += 1
                else:
                    pass
                #print(self.angle_direction)
            else:
                pass
            #Gets derivative of LOS angle 
            velocity = self.angle_derivative()
            
            desired_angular_velocity = self.proportional_navigation(velocity)
            
            self.turtleCMD(velocity_publisher,self.forward_velocity,desired_angular_velocity)
            
            self.timestamp[0] = self.timestamp[1]
        print("Main Ended")
        plt.plot(plot_x_pursuer,plot_y_pursuer,label="Pursuer")
        plt.plot(plot_x_evader,plot_y_evader,label="Evader")
        plt.plot(collision_point_x,collision_point_y,marker=".",markersize="20",label="Collision Point")
        plt.legend()
        plt.xlabel("X-Position [m]")
        plt.ylabel("Y-Position [m]")
        plt.title("Pursuit vs Evasion")
        plt.show()

            
            
        

if __name__=='__main__':
    #Initializing ROS node
    rospy.init_node('turtle3_pursuit')
    #Creating object "turtlebot" referencing "pursuer" class
    turtlebot = pursuer(0,0,0.2)
    try:
        print('Initializing Startup...')
        turtlebot.main()
        
    except rospy.ROSInterruptException:
        
        pass