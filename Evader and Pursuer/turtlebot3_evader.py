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

#Node Class
class Node():
    def __init__(self,xPos,yPos,parentCost,index):
        self.x = xPos
        self.y = yPos
        self.pc = parentCost
        self.i = index

class evader():
    def __init__(self,initial_position_x,initial_position_y,fwd_vel,obst_x,obst_y,x_lim,y_lim,step,diameter,x_init,y_init,x_fin,y_fin,eva_active):
        self.turtle_x = 0.0
        self.turtle_y = 0.0
        self.turtle_z = 0.0
        self.turtle_roll = 0.0
        self.turtle_pitch = 0.0
        self.turtle_yaw = 0.0
        self.turtle_w = 0.0
        self.forward_velocity = fwd_vel
        self.evasion_activation = eva_active

        #Obstacle List
        self.obstacles_x=obst_x
        self.obstacles_y=obst_y
        #Plot Parameters
        self.x_limit = x_lim
        self.y_limit = y_lim
        self.step_size = step
        self.diameter = diameter
        #Start and Goal
        self.start_x=x_init
        self.start_y=y_init
        self.goal_x=x_fin
        self.goal_y=y_fin
        #Settin up dictionaries
        self.unvisited_nodes={}
        self.visited_nodes={}
        self.traceback_nodes={}
        self.visited_traceback_nodes={}
    
    def odom_evader_callback(self,msg): 
        self.turtle_x = msg.pose.pose.position.x
        self.turtle_y = msg.pose.pose.position.y
        self.turtle_z = msg.pose.pose.position.z
        self.turtle_roll = msg.pose.pose.orientation.x
        self.turtle_pitch = msg.pose.pose.orientation.y
        self.turtle_yaw = msg.pose.pose.orientation.z
        self.turtle_w = msg.pose.pose.orientation.w
        return self.turtle_x,self.turtle_y,self.turtle_z,self.turtle_roll,self.turtle_pitch,self.turtle_yaw,self.turtle_w
        
    def odom_pursuer_callback(self,msg): 
        self.turtle_x_2 = msg.pose.pose.position.x
        self.turtle_y_2 = msg.pose.pose.position.y
        self.turtle_z_2 = msg.pose.pose.position.z
        
        return self.turtle_x_2,self.turtle_y_2,self.turtle_z_2

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

    def waypointAngle(self,startX,startY,endX,endY):
        #Detimine the heading angle to reach next waypoint
        xDist = endX-startX
        yDist = endY-startY
        angleWaypoint=math.atan2(yDist,xDist)
        return angleWaypoint

    def turtlebotMove(self,startX,startY,endX,endY,roll,pitch,yaw,w,kp):
        #Angle heading proportional controller
        headingDesired = self.waypointAngle(startX,startY,endX,endY)
        angles=tf.transformations.euler_from_quaternion([roll,pitch,yaw,w])
        yawCurrent=angles[2]
        error=(headingDesired-yawCurrent)*(1)
        yaw_turn = kp*error
        return yaw_turn
    #Takes map and establish nodes in dictionary with the coordinates and index
    def establish_nodes(self):
        self.node_dictionary={}
        index = 0
        x=0
        y=0
        for j in numpy.arange(self.y_limit[0],self.y_limit[1]+self.step_size,self.step_size):
            for i in numpy.arange(self.x_limit[0],self.x_limit[1]+self.step_size,self.step_size):
                self.node_dictionary[index]=Node(x,y,None,index)
                x += self.step_size
                index += 1
            x = 0
            y += self.step_size
        return self.node_dictionary
    
    def get_node(self,x,y):
        #Gets the node from the map
        i=0
        while i < len(self.node_dictionary):
            if x == self.node_dictionary[i].x and y == self.node_dictionary[i].y:
                node = self.node_dictionary[i]
                return node
                break
            else:
                i += 1
            
            
    #Gets parent cost of nodes
    def get_euclidean_distance(self,node1_x,node1_y,node2_x,node2_y):
       
        distance_x = abs(node1_x-node2_x)
        distance_y = abs(node1_y-node2_y)
        euclidean_distance = math.sqrt(pow(distance_x,2)+pow(distance_y,2))
        return euclidean_distance     
            
    #Checks availability of node and returns a true/false
    def check_availability(self,node_x,node_y):   
        #Checking X-boundary
        if node_x > self.x_limit[1]:
            return False
        elif node_x < self.x_limit[0]:
            return False
        else:
            pass
        #Checking Y-boundary
        if node_y > self.y_limit[1]:
            return False
        elif node_y < self.y_limit[0]:
            return False
        else:
            pass
        #Obstacles
        for i in range(0,len(self.obstacles_x)):
            distance = self.get_euclidean_distance(node_x,node_y,self.obstacles_x[i],self.obstacles_y[i])
            if distance < self.diameter:
                return False
                break
            else:
                pass
        
        return True
        
    #Gets the adjacent nodes to current node and determine the cost from the current node
    def get_adjacent_nodes(self,current_node):
        #Check top node availability
        top_node_check = self.check_availability(current_node.x,(current_node.y+self.step_size))
        #Get distance from current node to top node
        if top_node_check == True:
            top_node = self.get_node(current_node.x,(current_node.y+self.step_size))
            if top_node.i in self.visited_nodes:
                top_node=None
            else:
                top_cost=self.get_euclidean_distance(current_node.x,current_node.y,top_node.x,top_node.y)+ current_node.pc
                if self.node_dictionary[top_node.i].pc == None:
                    self.node_dictionary[top_node.i].pc = top_cost
                elif top_cost < self.node_dictionary[top_node.i].pc:
                    self.node_dictionary[top_node.i].pc = top_cost
                else:
                    self.node_dictionary[top_node.i].pc = self.node_dictionary[top_node.i].pc
            
        else:
            top_node=None
        
        #Check top right node availability
        top_right_node_check = self.check_availability((current_node.x+self.step_size),(current_node.y+self.step_size))
        #Get distance from current node to top node
        if top_right_node_check == True:
            top_right_node = self.get_node((current_node.x+self.step_size),(current_node.y+self.step_size))
            if top_right_node.i in self.visited_nodes:
                top_right_node=None
            else:
                top_right_cost=self.get_euclidean_distance(current_node.x,current_node.y,top_right_node.x,top_right_node.y)+ current_node.pc
                if self.node_dictionary[top_right_node.i].pc == None:
                    self.node_dictionary[top_right_node.i].pc = top_right_cost
                elif top_right_cost < self.node_dictionary[top_right_node.i].pc:
                    self.node_dictionary[top_right_node.i].pc = top_right_cost
                else:
                    self.node_dictionary[top_right_node.i].pc = self.node_dictionary[top_right_node.i].pc
        else:
            top_right_node=None
        
        #Check right node availability
        right_node_check = self.check_availability((current_node.x+self.step_size),current_node.y)
        #Get distance from current node to top node
        if right_node_check == True:
            right_node = self.get_node((current_node.x+self.step_size),current_node.y)
            if right_node.i in self.visited_nodes:
                right_node=None
            else:
                right_cost=self.get_euclidean_distance(current_node.x,current_node.y,right_node.x,right_node.y) + current_node.pc
                if self.node_dictionary[right_node.i].pc == None:
                    self.node_dictionary[right_node.i].pc = right_cost
                elif right_cost < self.node_dictionary[right_node.i].pc:
                    self.node_dictionary[right_node.i].pc = right_cost
                else:
                    self.node_dictionary[right_node.i].pc = self.node_dictionary[right_node.i].pc
        else:
            right_node=None
        
        #Check bottom right node availability     
        bottom_right_node_check = self.check_availability((current_node.x+self.step_size),(current_node.y-self.step_size))
        #Get distance from current node to top node
        if bottom_right_node_check == True:
            bottom_right_node = self.get_node((current_node.x+self.step_size),(current_node.y-self.step_size))
            if bottom_right_node.i in self.visited_nodes:
                bottom_right_node=None
            else:
                bottom_right_cost=self.get_euclidean_distance(current_node.x,current_node.y,bottom_right_node.x,bottom_right_node.y)+ current_node.pc
                if self.node_dictionary[bottom_right_node.i].pc == None:
                    self.node_dictionary[bottom_right_node.i].pc = bottom_right_cost
                elif bottom_right_cost < self.node_dictionary[bottom_right_node.i].pc:
                    self.node_dictionary[bottom_right_node.i].pc = bottom_right_cost
                else:
                    self.node_dictionary[bottom_right_node.i].pc = self.node_dictionary[bottom_right_node.i].pc
        else:
            bottom_right_node=None
        
        #Check bottom node availability     
        bottom_node_check = self.check_availability(current_node.x,(current_node.y-self.step_size))
        #Get distance from current node to top node
        if bottom_node_check == True:
            bottom_node = self.get_node(current_node.x,(current_node.y-self.step_size))
            if bottom_node.i in self.visited_nodes:
                bottom_node=None
            else:
                bottom_cost=self.get_euclidean_distance(current_node.x,current_node.y,bottom_node.x,bottom_node.y)+ current_node.pc
                if self.node_dictionary[bottom_node.i].pc == None:
                    self.node_dictionary[bottom_node.i].pc = bottom_cost
                elif bottom_cost < self.node_dictionary[bottom_node.i].pc:
                    self.node_dictionary[bottom_node.i].pc = bottom_cost
                else:
                    self.node_dictionary[bottom_node.i].pc = self.node_dictionary[bottom_node.i].pc
        else:
            bottom_node=None
        
        #Check bottom left node availability     
        bottom_left_node_check = self.check_availability((current_node.x-self.step_size),(current_node.y-self.step_size))
        #Get distance from current node to top node
        if bottom_left_node_check == True:
            bottom_left_node = self.get_node((current_node.x-self.step_size),(current_node.y-self.step_size))
            if bottom_left_node.i in self.visited_nodes:
                bottom_left_node=None
            else:
                bottom_left_cost=self.get_euclidean_distance(current_node.x,current_node.y,bottom_left_node.x,bottom_left_node.y)+ current_node.pc
                if self.node_dictionary[bottom_left_node.i].pc == None:
                    self.node_dictionary[bottom_left_node.i].pc = bottom_left_cost
                elif bottom_left_cost < self.node_dictionary[bottom_left_node.i].pc:
                    self.node_dictionary[bottom_left_node.i].pc = bottom_left_cost
                else:
                    self.node_dictionary[bottom_left_node.i].pc = self.node_dictionary[bottom_left_node.i].pc
        else:
            bottom_left_node=None
        
        #Check left node availability     
        left_node_check = self.check_availability((current_node.x-self.step_size),current_node.y)
        #Get distance from current node to top node
        if left_node_check == True:
            left_node = self.get_node((current_node.x-self.step_size),current_node.y)
            if left_node.i in self.visited_nodes:
                left_node=None
            else:
                left_cost=self.get_euclidean_distance(current_node.x,current_node.y,left_node.x,left_node.y)+ current_node.pc
                if self.node_dictionary[left_node.i].pc == None:
                    self.node_dictionary[left_node.i].pc = left_cost
                elif left_cost < self.node_dictionary[left_node.i].pc:
                    self.node_dictionary[left_node.i].pc = left_cost
                else:
                    self.node_dictionary[left_node.i].pc = self.node_dictionary[left_node.i].pc
        else:
            left_node=None
        
        #Check top left node availability     
        top_left_node_check = self.check_availability((current_node.x-self.step_size),(current_node.y+self.step_size))
        #Get distance from current node to top node
        if top_left_node_check == True:
            top_left_node = self.get_node((current_node.x-self.step_size),(current_node.y+self.step_size))
            if top_left_node.i in self.visited_nodes:
                top_left_node=None
            else:
                top_left_cost=self.get_euclidean_distance(current_node.x,current_node.y,top_left_node.x,top_left_node.y)+ current_node.pc
                if self.node_dictionary[top_left_node.i].pc == None:
                    self.node_dictionary[top_left_node.i].pc = top_left_cost 
                elif top_left_cost < self.node_dictionary[top_left_node.i].pc:
                    self.node_dictionary[top_left_node.i].pc = top_left_cost
                else:
                    self.node_dictionary[top_left_node.i].pc = self.node_dictionary[top_left_node.i].pc
        else:
            top_left_node=None
        
        return top_node,top_right_node,right_node,bottom_right_node,bottom_node,bottom_left_node,left_node,top_left_node
    
    #Gets node from unvisited dictionary based on the least parent cost
    def get_minimum_cost_node(self,current_node):
        t,tr,r,br,b,bl,l,tl=self.get_adjacent_nodes(current_node)
        adj_list = [t,tr,r,br,b,bl,l,tl]
        #Check for any nonetypes in list and deletes it
        if None in adj_list:
            while None in adj_list:
                adj_list.remove(None)  
        for index in range(0,len(adj_list)):
            self.unvisited_nodes[adj_list[index].i] = adj_list[index]
        #Gets node from unvisited dictionary based on the least parent cost
        node_to_visit = min(self.unvisited_nodes,key=lambda X: self.unvisited_nodes[X].pc)
        del adj_list[:]
        return node_to_visit
    
    #Gets adjacent nodes to current node for traceback
    def path_traceback(self,current_node,start_node):
        #Check top node availability
        top_node_check = self.check_availability(current_node.x,(current_node.y+self.step_size))
        #Get distance from current node to top node
        if top_node_check == True:
            top_node = self.get_node(current_node.x,(current_node.y+self.step_size))
            if top_node.i in self.visited_traceback_nodes:
                top_node= None
            elif top_node.i in self.visited_nodes:
                top_node=top_node
            else:
                top_node = None
        else:
            top_node=None
        
        #Check top right node availability
        top_right_node_check = self.check_availability((current_node.x+self.step_size),(current_node.y+self.step_size))
        #Get distance from current node to top node
        if top_right_node_check == True:
            top_right_node = self.get_node((current_node.x+self.step_size),(current_node.y+self.step_size))
            if top_right_node.i in self.visited_traceback_nodes:
                top_right_node=None
            elif top_right_node.i in self.visited_nodes:
                top_right_node=top_right_node
            else:
                top_right_node=None
        else:
            top_right_node=None
        
        #Check right node availability
        right_node_check = self.check_availability((current_node.x+self.step_size),current_node.y)
        #Get distance from current node to top node
        if right_node_check == True:
            right_node = self.get_node((current_node.x+self.step_size),current_node.y)
            if right_node.i in self.visited_traceback_nodes:
                right_node = None
            elif right_node.i in self.visited_nodes:
                right_node=right_node
            else:
                right_node=None
        else:
            right_node=None
        
        #Check bottom right node availability     
        bottom_right_node_check = self.check_availability((current_node.x+self.step_size),(current_node.y-self.step_size))
        #Get distance from current node to top node
        if bottom_right_node_check == True:
            bottom_right_node = self.get_node((current_node.x+self.step_size),(current_node.y-self.step_size))
            if bottom_right_node.i in self.visited_traceback_nodes:
                bottom_right_node = None
            elif bottom_right_node.i in self.visited_nodes:
                bottom_right_node=bottom_right_node
            else:
                bottom_right_node=None
        else:
            bottom_right_node=None
        
        #Check bottom node availability     
        bottom_node_check = self.check_availability(current_node.x,(current_node.y-self.step_size))
        #Get distance from current node to top node
        
        if bottom_node_check == True:
            bottom_node = self.get_node(current_node.x,(current_node.y-self.step_size))
            if bottom_node.i in self.visited_traceback_nodes:
                bottom_node = None
            elif bottom_node.i in self.visited_nodes:
                bottom_node=bottom_node
            else:
                bottom_node=None
        else:
            bottom_node=None
        
        #Check bottom left node availability     
        bottom_left_node_check = self.check_availability((current_node.x-self.step_size),(current_node.y-self.step_size))
        #Get distance from current node to top node
        if bottom_left_node_check == True:
            bottom_left_node = self.get_node((current_node.x-self.step_size),(current_node.y-self.step_size))
            if bottom_left_node.i in self.visited_traceback_nodes:
                bottom_left_node = None
            elif bottom_left_node.i in self.visited_nodes:
                bottom_left_node=bottom_left_node
            else:
                bottom_left_node=None
        else:
            bottom_left_node=None
        
        #Check left node availability     
        left_node_check = self.check_availability((current_node.x-self.step_size),current_node.y)
        #Get distance from current node to top node
        if left_node_check == True:
            left_node = self.get_node((current_node.x-self.step_size),current_node.y)
            if left_node.i in self.visited_traceback_nodes:
                left_node = None
            elif left_node.i in self.visited_nodes:
                left_node=left_node
            else:
                left_node=None
        else:
            left_node=None
        
        #Check top left node availability     
        top_left_node_check = self.check_availability((current_node.x-self.step_size),(current_node.y+self.step_size))
        #Get distance from current node to top node
        if top_left_node_check == True:
            top_left_node = self.get_node((current_node.x-self.step_size),(current_node.y+self.step_size))
            if top_left_node.i in self.visited_traceback_nodes:
                top_left_node = None
            elif top_left_node.i in self.visited_nodes:
                top_left_node=top_left_node
            else:
                top_left_node=None
        else:
            top_left_node=None
            
        return top_node,top_right_node,right_node,bottom_right_node,bottom_node,bottom_left_node,left_node,top_left_node
    
    #Gets the node adjacent to the current node being traced backed based on the minimum parent cost    
    def get_minimum_cost_node_back(self,current_node,start_node):
        t,tr,r,br,b,bl,l,tl=self.path_traceback(current_node,start_node)
        adj_list = [t,tr,r,br,b,bl,l,tl]
        #Check for any nonetypes in list and deletes it
        if None in adj_list:
            while None in adj_list:
                adj_list.remove(None)  
        for index in range(0,len(adj_list)):
            self.traceback_nodes[adj_list[index].i] = adj_list[index]
        #Gets the node adjacent to the current node being traced backed based on the minimum parent cost
        node_to_visit = min(self.traceback_nodes,key=lambda X: self.traceback_nodes[X].pc)
        del adj_list[:]
        return node_to_visit
    
    def get_midpoint(self,x1,y1,x2,y2):
        x_midpoint = (x1+x2)/2
        y_midpoint = (y1+y2)/2
        return x_midpoint,y_midpoint

    def evade(self,waypoints_x,waypoints_y):
        alternating_direction = 0
        offset = 0.5
        evasion_waypoints_x=[]
        evasion_waypoints_y=[]
        evasion_waypoints_x.append(waypoints_x[0])
        evasion_waypoints_y.append(waypoints_y[0])
        for i in range(1,len(waypoints_x)):
            #Use a counter to alternate direction of new point
            if alternating_direction == 0:
                new_waypoint_x = waypoints_x[i]+offset
                new_waypoint_y=waypoints_y[i]+offset
                evasion_waypoints_x.append(new_waypoint_x)
                evasion_waypoints_y.append(new_waypoint_y)
                alternating_direction = 1
            else:
                new_waypoints_x = waypoints_x[i]-offset
                new_waypoint_y=waypoints_y[i]+offset
                evasion_waypoints_x.append(new_waypoint_x)
                evasion_waypoints_y.append(new_waypoint_y)
                alternating_direction = 0
        evasion_waypoints_x.append(waypoints_x[i])
        evasion_waypoints_y.append(waypoints_y[i])

        return evasion_waypoints_x,evasion_waypoints_y

    def turtlebot_distance(self,x1,y1,x2,y2):
        distx=abs(x1-x2)
        disty=abs(y1-y2)
        dist=math.sqrt(pow(distx,2)+pow(disty,2))
        return dist    
            
    
    #Main code
    def main(self):
        rate = rospy.Rate(50) #hz
        #scan_sub=rospy.Subscriber('/turtlebot2/scan',LaserScan,self.scan_callback)
        odom_evader_sub = rospy.Subscriber("/turtlebot1/odom",Odometry,self.odom_evader_callback)
        odom_pursuer_sub=rospy.Subscriber('/turtlebot2/odom',Odometry,self.odom_pursuer_callback)
        velocity_publisher = rospy.Publisher('/turtlebot1/cmd_vel', Twist, queue_size=10)

        #Sets the each coords to the Node class
        self.node_dictionary = self.establish_nodes()
        #Gets starting node
        start_node=self.get_node(self.start_x,self.start_y)
        start_node.pc = 0
        current_node=start_node
        #Gets goal node
        goal_node=self.get_node(self.goal_x,self.goal_y)
        #first iteration
        self.unvisited_nodes[current_node.i] = current_node
        self.visited_nodes[current_node.i] = current_node
        del self.unvisited_nodes[current_node.i]
        
        #Search Section
        while current_node.i != goal_node.i:
            #Get the next node with the least cost
            node_next=self.get_minimum_cost_node(current_node)
            #Sets the current node as the adjacent node with the least cost
            current_node = self.unvisited_nodes[node_next]
            self.visited_nodes[current_node.i] = current_node
            del self.unvisited_nodes[current_node.i]
            if current_node.i == goal_node.i:
                print('goal reached!')
            else:
                print('goal not reached yet!')
                
        #Setting up plots        
        I = []
        J = []
        fig, backTrack =  plt.subplots()
        plt.xlim(self.x_limit[0]-self.step_size,self.x_limit[1]+self.step_size)
        plt.ylim(self.y_limit[0]-self.step_size,self.y_limit[1]+self.step_size)
        plt.title('Dijkstra')
        plt.scatter(self.start_x,self.start_y,color='red')
        plt.scatter(self.goal_x,self.goal_y,color='green')
        plt.scatter(self.obstacles_x,self.obstacles_y)
        plt.xlabel('X-Position in Meters')
        plt.ylabel('Y-Position in Meters')
        plt.grid()
        
        #Traceback Section
        current_node_back = goal_node 
        while current_node_back.i != start_node.i:
            I.append(current_node_back.x)
            J.append(current_node_back.y)
            node_to_trace=self.get_minimum_cost_node_back(current_node_back,start_node)
            current_node_back=self.traceback_nodes[node_to_trace]
            self.visited_traceback_nodes[current_node_back.i] = current_node_back
            del self.traceback_nodes[current_node_back.i]
            if current_node_back.i == start_node.i:
                print('traceback complete')
            else:
                print('traceback in progress')
        
        I.append(start_node.x)
        J.append(start_node.y)
        

        waypointsX = I
        waypointsY = J

        n=110.0
        maxErrorPercent=0.15
        maxErrorDeg=5.0
        kp=(maxErrorPercent/maxErrorDeg)*n
        #Reverse order of waypoints
        waypointsX.reverse()
        waypointsY.reverse()

        if self.evasion_activation ==True:
            waypointsX,waypointsY = self.evade(waypointsX,waypointsY)
            # print(len(waypointsX))
            # plt.plot(waypointsX,waypointsY)
            # plt.show()
        else:
            pass
        waypointsLen=len(waypointsX)
        #print(waypointsX)
        i=0

        plot_x_pursuer=[]
        plot_y_pursuer=[]
        plot_x_evader=[]
        plot_y_evader=[]
        collision_point_x=[]
        collision_point_y=[]
        collision_iteration = 0

        while not rospy.is_shutdown():
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
            print('Waypoint ', i ,' out of ', waypointsLen)
            if i < waypointsLen:
                if self.turtle_x != waypointsX[i] and self.turtle_y!= waypointsY[i]:
                    zTurn=self.turtlebotMove(waypointsX[i-1],waypointsY[i-1],waypointsX[i],waypointsY[i],self.turtle_roll,self.turtle_pitch,self.turtle_yaw,self.turtle_w,kp)
                    self.turtleCMD(velocity_publisher,self.forward_velocity,zTurn)
                    print(round(self.turtle_x,1),round(self.turtle_y,1))
                    print(round(waypointsX[i],1),round(waypointsY[i],1))
                    if round(self.turtle_x,1)==round(waypointsX[i],1) and round(self.turtle_y,1)==round(waypointsY[i],1):
                        i=i+1
                    elif round(self.turtle_x,1) == self.goal_x and round(self.turtle_y,1) == self.goal_y:
                        self.turtleCMD(velocity_publisher,0,0)
                        i = waypointsLen

            else:
                self.turtleCMD(velocity_publisher,0,0)
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
    rospy.init_node('turtle3_evader')
    #Creating object "turtlebot" referencing "pursuer" class
    #Obstacle List
    obstacles_x=[5,5,5,5,5,0,1,2,3,3]
    obstacles_y=[0,1,2,3,4,5,4,3,2,3]
    #Plot Parameters
    x_limit = [0,10]
    y_limit = [0,10]
    step_size=0.5
    diameter = 1
    #Start and Goal
    start_x=2
    start_y=1
    goal_x=7
    goal_y=2
    evasion_active=False
    turtlebot = evader(0,0,0.15,obstacles_x, obstacles_y, x_limit, y_limit, step_size, diameter, start_x, start_y, goal_x, goal_y,evasion_active)
    try:
        print('Initializing Startup...')
        turtlebot.main()
        
    except rospy.ROSInterruptException:
        
        pass