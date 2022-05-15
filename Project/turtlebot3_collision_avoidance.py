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
    def __init__(self,initial_position_x,initial_position_y,fwd_vel,obst_x,obst_y,x_lim,y_lim,step,diameter,x_init,y_init,x_fin,y_fin):
        self.turtle_x = 0.0
        self.turtle_y = 0.0
        self.turtle_z = 0.0
        self.turtle_roll = 0.0
        self.turtle_pitch = 0.0
        self.turtle_yaw = 0.0
        self.turtle_w = 0.0
        self.forward_velocity = fwd_vel
        self.scan = []
        self.waypointsX = []
        self.waypointsY = []
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
    
    def odom_callback(self,msg): 
        self.turtle_x = msg.pose.pose.position.x
        self.turtle_y = msg.pose.pose.position.y
        self.turtle_z = msg.pose.pose.position.z
        self.turtle_roll = msg.pose.pose.orientation.x
        self.turtle_pitch = msg.pose.pose.orientation.y
        self.turtle_yaw = msg.pose.pose.orientation.z
        self.turtle_w = msg.pose.pose.orientation.w
        return self.turtle_x,self.turtle_y,self.turtle_z,self.turtle_roll,self.turtle_pitch,self.turtle_yaw,self.turtle_w
    
    #Get callback data from turtlebot scan
    def scan_callback(self,msg):
        self.scan = msg.ranges
        
        return self.scan
        

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

    #Generates a list of angles that are non-zero
    def locate_angle(self):
        
        angle_list=[]
        for i in range(0,len(self.scan)):
            if not self.scan:
                pass
            else:
                if self.scan[i] != float('inf'):
                    if i in self.left_cone or i in self.right_cone:
                        angle_list.append(i)
                    else:
                        pass
                else:
                    pass
        if angle_list:
            angle=angle_list[int(round((len(angle_list)/2)-1,1))]
            return angle
        else:
            return None

    #Extracts the distance from an angle that is non-zero
    def extract_range(self,angle):
        if angle == None:
            pass
        else:
            distance = round(self.scan[angle],1)
            return distance

    #Converts the polar coords of a detected object and gets the cartesian coords
    def determine_cartesian_coords(self,angle,distance):
        if angle == None:
            return None,None
        else:
            
            x_detection = distance*math.cos(math.radians(angle))
            y_detection = distance*math.sin(math.radians(angle))
            x_obstacle = round(self.turtle_x + x_detection,2)
            y_obstacle = round(self.turtle_y + y_detection,2)
            return x_obstacle,y_obstacle

    #Define "cones" in front of the turtlebot and divides it into two sections (left and right)
    def create_scan_cones(self):
        self.left_cone=[]
        self.right_cone=[]
        for l in range(1,66,1):
            self.left_cone.append(l)
        for r in range(360,294,-1):
            self.right_cone.append(r)

    #Detects whether the oject/obstacle is in the left or right section
    def check_obstacle_in_cones(self,angle,distance):
        left_case = False
        right_case = False
        if (angle in self.left_cone and distance <=1.0) and (angle in self.right_cone and distance <=1.0):
            print('Obstacle on both sides!')
            left_case = True
            right_case = True
            return left_case ,right_case
        elif angle in self.left_cone and distance <=1.0:
            print('Obstacle on my left!')
            left_case = True
            return left_case ,right_case
        elif angle in self.right_cone and distance <=1.0:
            print('Obstacle on my right!')
            right_case = True
            return left_case ,right_case
        
        else:
            print('I do not see an obstacle in front of me...')
            return left_case ,right_case

    #If a object is in one of the two detection sections, turn the other way to avoid it
    def avoid_obstacles(self,left,right,publisher,distance):
        if left == True and right == False:
            #print('Turning right')
            if distance >= 0.75 and distance < 1.75:
                self.turtleCMD(publisher,0.15,-0.20)
            elif distance < 0.75:
                self.turtleCMD(publisher,0.15,-0.25)
        if right == True and left ==False:
            #print('Turning left')
            if distance >= 0.75 and distance < 1.75:
                self.turtleCMD(publisher,0.15,0.20)
            elif distance < 0.75:
                self.turtleCMD(publisher,0.15,0.25)
        if right == True and left == True:
            self.turtleCMD(publsher,0.15,0)
        else:
            pass

    #If waypoints are close to the object/obstacle, remove from list
    def check_waypoints(self,x_obst,y_obst,waypoint_x,waypoint_y,wp_len):
        if x_obst == None:
            pass
        else:
            obst_wp_dist=self.get_euclidean_distance(x_obst,y_obst,waypoint_x,waypoint_y)
            if obst_wp_dist <= 0.65:
                self.waypointsX.remove(waypoint_x)
                self.waypointsY.remove(waypoint_y)
            else:
                pass
            
    #Main code
    def main(self):
        rate = rospy.Rate(50) #hz
        #scan_sub=rospy.Subscriber('/turtlebot2/scan',LaserScan,self.scan_callback)
        odom_sub = rospy.Subscriber("/turtlebot1/odom",Odometry,self.odom_callback)
        velocity_publisher = rospy.Publisher('/turtlebot1/cmd_vel', Twist, queue_size=10)
        scan_sub=rospy.Subscriber('/turtlebot1/scan',LaserScan,self.scan_callback)

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
        

        self.waypointsX = I
        self.waypointsY = J

        n=150.0
        maxErrorPercent=0.15
        maxErrorDeg=5.0
        kp=(maxErrorPercent/maxErrorDeg)*n
        #Reverse order of waypoints
        self.waypointsX.reverse()
        self.waypointsY.reverse()

        waypointsLen=len(self.waypointsX)
        #print(self.waypointsX)
        #print(self.waypointsY)
        i=1


        while not rospy.is_shutdown(): 
            #Gets current LOS angle and timestamp
            self.create_scan_cones()
            angle = self.locate_angle()
            distance = self.extract_range(angle)
            #print('Angle:',angle,'Distance',distance)
            x_obst,y_obst = self.determine_cartesian_coords(angle,distance)
            print(x_obst,y_obst)
            left_case ,right_case = self.check_obstacle_in_cones(angle,distance)
            self.check_waypoints(x_obst,y_obst,self.waypointsX[i],self.waypointsY[i],waypointsLen)
            waypointsLen = len(self.waypointsX)
            print('Waypoint ', i ,' out of ', waypointsLen)
            if i < waypointsLen:
                if self.turtle_x != self.waypointsX[i] and self.turtle_y!= self.waypointsY[i]:
                    if left_case != False or right_case != False:
                        self.avoid_obstacles(left_case,right_case,velocity_publisher,distance)
                        waypointsLen = len(self.waypointsX)
                    else:
                        waypointsLen = len(self.waypointsX)
                        zTurn=self.turtlebotMove(self.turtle_x,self.turtle_y,self.waypointsX[i],self.waypointsY[i],self.turtle_roll,self.turtle_pitch,self.turtle_yaw,self.turtle_w,kp)
                        self.turtleCMD(velocity_publisher,self.forward_velocity,zTurn)
                        #print(round(self.turtle_x,1),round(self.turtle_y,1))
                        #print(round(self.waypointsX[i],1),round(self.waypointsY[i],1))
                        if abs((round(self.turtle_x,1)-round(self.waypointsX[i],1))) <= 0.2 and abs(round(self.turtle_y,1)-round(self.waypointsY[i],1)) <=0.2:
                            if round(self.turtle_x,1) == self.goal_x and round(self.turtle_y,1) == self.goal_y:
                                self.turtleCMD(velocity_publisher,0,0)
                                print('Goal Reached!')
                            else:
                                i=i+1
                        else:
                            continue

            else:
                self.turtleCMD(velocity_publisher,0,0)
        
        


if __name__=='__main__':
    #Initializing ROS node
    rospy.init_node('turtle3_project')
    #Creating object "turtlebot" referencing "pursuer" class
    #Obstacle List
    obstacles_x=[]
    obstacles_y=[]
    #Plot Parameters
    x_limit = [0,10]
    y_limit = [0,10]
    step_size=0.5
    diameter = 1
    #Start and Goal
    start_x=3
    start_y=3
    goal_x=7
    goal_y=8
    
    turtlebot = evader(0,0,0.15,obstacles_x, obstacles_y, x_limit, y_limit, step_size, diameter, start_x, start_y, goal_x, goal_y)
    try:
        print('Initializing Startup...')
        turtlebot.main()
        
    except rospy.ROSInterruptException:
        
        pass