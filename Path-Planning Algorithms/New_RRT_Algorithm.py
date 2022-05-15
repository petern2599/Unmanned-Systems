#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy 
import math
import random
import time

#node class
class Node():
    def __init__(self,xPos,yPos,parentCost,index):
        self.x = xPos
        self.y = yPos
        self.pc = parentCost
        self.i = index
class LineSegment():
    def __init__(self,x_init,y_init,x_fin,y_fin):
        self.x_pos1 = x_init
        self.y_pos1 = y_init
        self.x_pos2 = x_fin
        self.y_pos2 = y_fin

class RRTAlgorithm():
    def __init__(self,obst_x,obst_y,x_lim,y_lim,step,diameter,x_init,y_init,x_fin,y_fin,max_dist):
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
        #RRT Parameters
        self.max_distance = max_dist
        #Dictionaries
        self.existing_nodes={}
        self.unvisited_nodes={}
        self.visited_nodes={}
        self.lines_dictionary={}
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
        i=0
        while i < len(self.node_dictionary):
            if x == self.node_dictionary[i].x and y == self.node_dictionary[i].y:
                node = self.node_dictionary[i]
                return node
                break
            else:
                i += 1
                
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
            
    def get_random_node(self):
        #Gets a random node from dictionary
        node_length=(len(self.node_dictionary))
        random_node=random.randint(0,node_length-1)
        #Get node information
        node_projected=self.node_dictionary[random_node]
        check=self.check_availability(node_projected.x,node_projected.y)
        if check == True:
            node_projected = node_projected
            
            return node_projected
        else:
            node_projected = None
            return node_projected
    
    def get_euclidean_distance(self,node1_x,node1_y,node2_x,node2_y):
        distance_x = abs(node1_x-node2_x)
        distance_y = abs(node1_y-node2_y)
        euclidean_distance = math.sqrt(pow(distance_x,2)+pow(distance_y,2))
        return euclidean_distance    
    
    def find_closest_node(self,projected_node):
        self.distance_dictionary={}
        for i in range(0,len(self.existing_nodes)):
            distance=self.get_euclidean_distance(projected_node.x, projected_node.y, self.existing_nodes[i].x, self.existing_nodes[i].y)
            self.distance_dictionary[i] = distance
        node_to_visit = min(self.distance_dictionary,key=lambda X: self.distance_dictionary[X])
        closest_node = self.existing_nodes[node_to_visit]
        
        return closest_node
    
    def get_euclidean_distance_angle(self,node1_x,node1_y,node2_x,node2_y):
        distance_x = (node2_x-node1_x)
        distance_y = (node2_y-node1_y)
        euclidean_distance = math.sqrt(pow(distance_x,2)+pow(distance_y,2))
        angle = math.atan2(distance_y,distance_x)
        return distance_x,distance_y,euclidean_distance,angle 
    
    def get_new_node(self,closest_node,node_projected):
        #Get new node sample
        dist_x,dist_y,distance,angle=self.get_euclidean_distance_angle(closest_node.x, closest_node.y, node_projected.x, node_projected.y)
        new_node_x = (math.cos(angle)*self.max_distance) + closest_node.x
        new_node_y = math.sin(angle)*self.max_distance + closest_node.y
        new_node_cost = self.max_distance + closest_node.pc
        new_node = Node(new_node_x,new_node_y,new_node_cost,None)
        return new_node
    
    def traceback(self,current_node):
        self.traceback_distance_dictionary={}
        #Gets the distances of all nodes relative to the current node
        for i in range(0,len(self.existing_nodes)):
            if i is not self.visited_nodes:
                distance=self.get_euclidean_distance(current_node.x, current_node.y, self.unvisited_nodes[i].x, self.unvisited_nodes[i].y)
                self.traceback_distance_dictionary[i] = distance
            else:
                distance=100
                self.traceback_distance_dictionary[i] = distance
        self.closest_to_start_dictionary={}
        dummy_node=Node(None,None,100,None)
        #Filters out only the distances that are equal or less than 0.50 from current node
        for j in range(0,len(self.traceback_distance_dictionary)-1):
            if round(self.traceback_distance_dictionary[j],1) <= 0.50:  
                for k in range(1,len(self.lines_dictionary)):
                    if current_node.x == self.lines_dictionary[k].x_pos2 and current_node.y == self.lines_dictionary[k].y_pos2 and self.existing_nodes[j].x == self.lines_dictionary[k].x_pos1 and self.existing_nodes[j].y == self.lines_dictionary[k].y_pos1:
                        self.closest_to_start_dictionary[j] = self.existing_nodes[j]
                    else:
                        self.closest_to_start_dictionary[j] = dummy_node
        #Gets the node with the least parent cost
        node_to_visit = min(self.closest_to_start_dictionary,key=lambda X: self.closest_to_start_dictionary[X].pc)
        #Gets node info
        closest_node=self.existing_nodes[node_to_visit]
        #self.traceback_distance_dictionary.clear()
        #self.closest_to_start_dictionary.clear()
        return closest_node
    
    def execute(self):
        #Sets the each coords to the Node class
        self.node_dictionary = self.establish_nodes()
        #Gets starting node
        new_node=self.get_node(self.start_x,self.start_y)
        self.node_dictionary[new_node.i].pc = 0
        self.node_dictionary[new_node.i].i = 0
        #Gets goal node
        goal_node=self.get_node(self.goal_x,self.goal_y)
        #first iteration
        index=0
        self.existing_nodes[index] = new_node
        #Setting up a iteration limit
        i=0
        imax=100000
        #Setting up plot
        fig, backTrack =  plt.subplots()
        plt.xlim(self.x_limit[0]-self.step_size,self.x_limit[1]+self.step_size)
        plt.ylim(self.y_limit[0]-self.step_size,self.y_limit[1]+self.step_size)
        plt.title('RRT')
        plt.scatter(self.start_x,self.start_y,color='red')
        plt.scatter(self.goal_x,self.goal_y,color='green')
        plt.scatter(self.obstacles_x,self.obstacles_y)
        plt.xlabel('X-Position in Meters')
        plt.ylabel('Y-Position in Meters')
        plt.grid()
        
        while i < imax:
            
            #Project a random node sample
            node_projected=self.get_random_node()
            if node_projected == None:
                pass
            else:
                #Find closest node to projected node
                closest_node = self.find_closest_node(node_projected)
                #Get new node sample
                new_node=self.get_new_node(closest_node,node_projected)
                check = self.check_availability(new_node.x,new_node.y)
                if check == True:
                    index += 1
                    distance=self.get_euclidean_distance(closest_node.x,closest_node.y,goal_node.x,goal_node.y)
                    if distance <= 0.5:
                        new_node.pc=closest_node.pc + distance
                        new_node.x = goal_node.x
                        new_node.y = goal_node.y
                        new_node.i = index
                        self.existing_nodes[index] = new_node
                        #plotting branches
                        plot_x = [closest_node.x,new_node.x]
                        plot_y = [closest_node.y,new_node.y]
                        plt.plot(plot_x,plot_y,color='gray')
                        #storing branches into a dictionary for traceback
                        line_segment = LineSegment(closest_node.x, closest_node.y, new_node.x, new_node.y)
                        self.lines_dictionary[index] = line_segment
                        print('Reached Goal')
                        
                        #Start Traceback
                        current_node_back = goal_node
                        self.unvisited_nodes=self.existing_nodes
                        I=[]
                        J=[]
                        I.append(current_node_back.x)
                        J.append(current_node_back.y)
                        while current_node_back.i != 0:
                            traceback_node=self.traceback(current_node_back)
                            self.visited_nodes[traceback_node.i]=traceback_node
                            I.append(traceback_node.x)
                            J.append(traceback_node.y)
                            current_node_back = traceback_node
                            
                            if current_node_back.i == 0:
                                print("Traceback Complete")
                                print('Total Cost = ', new_node.pc, ' meters')
                            else:
                                print("Traceback in Progress")
                        plt.plot(I,J,color='black')
                        break
                    else:
                        self.existing_nodes[index] = new_node
                        new_node.i=index
                        #plotting branches
                        plot_x = [closest_node.x,new_node.x]
                        plot_y = [closest_node.y,new_node.y]
                        plt.plot(plot_x,plot_y,color='gray')
                        #storing branches into a dictionary for traceback
                        line_segment = LineSegment(closest_node.x, closest_node.y, new_node.x, new_node.y)
                        self.lines_dictionary[index] = line_segment
                        print('Searching for Goal')
                        i += 1
                else:
                    i += 1
                    pass
                
        
        
if __name__ == '__main__':
    #Obstacle List
    obstacles_x=[2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8, 8, 8, 8, 8, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 2, 2, 2, 2, 2, 2, 5, 5, 5, 5, 5, 5, 5, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12]
    obstacles_y=[2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2, 2, 2, 2, 3, 4, 5, 6, 7, 8, 9, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13, 14, 15, 12, 12, 12, 12, 12, 12, 8, 9, 10, 11, 12]
    #Plot Parameters
    x_limit = [0,15]
    y_limit = [0,15]
    step_size=0.5
    #Had to change diameter of obstacles to reach target (previous 1.0 didn't allow much room for RRT to explore)
    diameter = 0.75
    #Start and Goal
    start_x=1
    start_y=1
    goal_x=7
    goal_y=13
    #RRT Parameters
    max_distance = 0.5
    
    a=RRTAlgorithm(obstacles_x, obstacles_y, x_limit, y_limit, step_size, diameter, start_x, start_y, goal_x, goal_y,max_distance)
    start_time = time.time()
    a.execute()
    print('Runtime:', time.time()-start_time , 'seconds')