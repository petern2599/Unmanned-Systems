#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy 
import math
import time 
#node class
class Node():
    def __init__(self,xPos,yPos,parentCost,index):
        self.x = xPos
        self.y = yPos
        self.pc = parentCost
        self.i = index
        
        
class AStarAlgorithm():
    def __init__(self,obst_x,obst_y,x_lim,y_lim,step,diameter,x_init,y_init,x_fin,y_fin):
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
        self.original_cost={}
    #Takes map and establish nodes in dictionary with the coordinates and index
    def establish_nodes(self):
        self.node_dictionary={}
        index = 0
        x=0
        y=0
        for j in numpy.arange(self.y_limit[0],self.y_limit[1]+self.step_size,self.step_size):
            for i in numpy.arange(self.x_limit[0],self.x_limit[1]+self.step_size,self.step_size):
                self.node_dictionary[index]=Node(x,y,None,index)
                x += step_size
                index += 1
            x = 0
            y += step_size
        
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
            
            
    #Gets parent cost of nodes
    def get_euclidean_distance(self,node1_x,node1_y,node2_x,node2_y):
        distance_x = abs(node1_x-node2_x)
        distance_y = abs(node1_y-node2_y)
        euclidean_distance = math.sqrt(pow(distance_x,2)+pow(distance_y,2))
        return euclidean_distance
    def get_heuristic_distance(self,node_x,node_y,goal_x,goal_y):
        distance_x = abs(node_x-goal_x)
        distance_y = abs(node_y-goal_y)
        heuristic_distance = math.sqrt(pow(distance_x,2)+pow(distance_y,2))
        return heuristic_distance         
            
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
        
            
    def get_adjacent_nodes(self,current_node,goal_node):
        #Check top node availability
        top_node_check = self.check_availability(current_node.x,(current_node.y+self.step_size))
        #Get distance from current node to top node
        if top_node_check == True:
            top_node = self.get_node(current_node.x,(current_node.y+self.step_size))
            if top_node.i in self.visited_nodes:
                top_node=None
            else:
                top_cost=self.get_euclidean_distance(current_node.x,current_node.y,top_node.x,top_node.y)+ self.original_cost[current_node.i]
                top_h_cost=self.get_heuristic_distance(top_node.x,top_node.y,goal_node.x,goal_node.y)
                top_cost_total = top_cost + top_h_cost
                if self.node_dictionary[top_node.i].pc == None:
                    self.node_dictionary[top_node.i].pc = top_cost_total
                    self.original_cost[top_node.i]=top_cost
                elif top_cost_total < self.node_dictionary[top_node.i].pc:
                    self.node_dictionary[top_node.i].pc = top_cost_total
                    self.original_cost[top_node.i]=top_cost
                else:
                    self.node_dictionary[top_node.i].pc = self.node_dictionary[top_node.i].pc
                    self.original_cost[top_node.i]=self.original_cost[top_node.i]
            
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
                top_right_cost=self.get_euclidean_distance(current_node.x,current_node.y,top_right_node.x,top_right_node.y)+ self.original_cost[current_node.i]
                top_right_h_cost=self.get_heuristic_distance(top_right_node.x,top_right_node.y,goal_node.x,goal_node.y)
                top_right_cost_total = top_right_cost + top_right_h_cost
                if self.node_dictionary[top_right_node.i].pc == None:
                    self.node_dictionary[top_right_node.i].pc = top_right_cost_total
                    self.original_cost[top_right_node.i] = top_right_cost
                elif top_right_cost_total < self.node_dictionary[top_right_node.i].pc:
                    self.node_dictionary[top_right_node.i].pc = top_right_cost_total
                    self.original_cost[top_right_node.i] = top_right_cost
                else:
                    self.node_dictionary[top_right_node.i].pc = self.node_dictionary[top_right_node.i].pc
                    self.original_cost[top_right_node.i] = self.original_cost[top_right_node.i]
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
                right_cost=self.get_euclidean_distance(current_node.x,current_node.y,right_node.x,right_node.y )+ self.original_cost[current_node.i]
                right_h_cost=self.get_heuristic_distance(right_node.x,right_node.y,goal_node.x,goal_node.y)
                right_cost_total = right_cost + right_h_cost
                if self.node_dictionary[right_node.i].pc == None:
                    self.node_dictionary[right_node.i].pc = right_cost_total
                    self.original_cost[right_node.i] = right_cost
                elif right_cost_total < self.node_dictionary[right_node.i].pc:
                    self.node_dictionary[right_node.i].pc = right_cost_total
                    self.original_cost[right_node.i] = right_cost
                else:
                    self.node_dictionary[right_node.i].pc = self.node_dictionary[right_node.i].pc
                    self.original_cost[right_node.i] = self.original_cost[right_node.i]
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
                bottom_right_cost=self.get_euclidean_distance(current_node.x,current_node.y,bottom_right_node.x,bottom_right_node.y)+ self.original_cost[current_node.i]
                bottom_right_h_cost=self.get_heuristic_distance(bottom_right_node.x,bottom_right_node.y,goal_node.x,goal_node.y)
                bottom_right_cost_total = bottom_right_cost + bottom_right_h_cost
                if self.node_dictionary[bottom_right_node.i].pc == None:
                    self.node_dictionary[bottom_right_node.i].pc = bottom_right_cost_total
                    self.original_cost[bottom_right_node.i] = bottom_right_cost
                elif bottom_right_cost_total < self.node_dictionary[bottom_right_node.i].pc:
                    self.node_dictionary[bottom_right_node.i].pc = bottom_right_cost_total
                    self.original_cost[bottom_right_node.i] = bottom_right_cost
                else:
                    self.node_dictionary[bottom_right_node.i].pc = self.node_dictionary[bottom_right_node.i].pc
                    self.original_cost[bottom_right_node.i] = bottom_right_cost
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
                bottom_cost=self.get_euclidean_distance(current_node.x,current_node.y,bottom_node.x,bottom_node.y)+ self.original_cost[current_node.i]
                bottom_h_cost=self.get_heuristic_distance(bottom_node.x,bottom_node.y,goal_node.x,goal_node.y)
                bottom_cost_total = bottom_cost + bottom_h_cost
                if self.node_dictionary[bottom_node.i].pc == None:
                    self.node_dictionary[bottom_node.i].pc = bottom_cost_total
                    self.original_cost[bottom_node.i] = bottom_cost
                elif bottom_cost_total < self.node_dictionary[bottom_node.i].pc:
                    self.node_dictionary[bottom_node.i].pc = bottom_cost_total
                    self.original_cost[bottom_node.i] = bottom_cost
                else:
                    self.node_dictionary[bottom_node.i].pc = self.node_dictionary[bottom_node.i].pc
                    self.original_cost[bottom_node.i] = self.original_cost[bottom_node.i]
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
                bottom_left_cost=self.get_euclidean_distance(current_node.x,current_node.y,bottom_left_node.x,bottom_left_node.y)+ self.original_cost[current_node.i]
                bottom_left_h_cost=self.get_heuristic_distance(bottom_left_node.x,bottom_left_node.y,goal_node.x,goal_node.y)
                bottom_left_cost_total = bottom_left_cost + bottom_left_h_cost
                if self.node_dictionary[bottom_left_node.i].pc == None:
                    self.node_dictionary[bottom_left_node.i].pc = bottom_left_cost_total
                    self.original_cost[bottom_left_node.i] = bottom_left_cost
                elif bottom_left_cost_total < self.node_dictionary[bottom_left_node.i].pc:
                    self.node_dictionary[bottom_left_node.i].pc = bottom_left_cost_total
                    self.original_cost[bottom_left_node.i] = bottom_left_cost
                else:
                    self.node_dictionary[bottom_left_node.i].pc = self.node_dictionary[bottom_left_node.i].pc
                    self.original_cost[bottom_left_node.i] = self.original_cost[bottom_left_node.i] 
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
                left_cost=self.get_euclidean_distance(current_node.x,current_node.y,left_node.x,left_node.y)+ self.original_cost[current_node.i]
                left_h_cost=self.get_heuristic_distance(left_node.x,left_node.y,goal_node.x,goal_node.y)
                left_cost_total = left_cost + left_h_cost
                if self.node_dictionary[left_node.i].pc == None:
                    self.node_dictionary[left_node.i].pc = left_cost_total
                    self.original_cost[left_node.i] = left_cost
                elif left_cost_total < self.node_dictionary[left_node.i].pc:
                    self.node_dictionary[left_node.i].pc = left_cost_total
                    self.original_cost[left_node.i] = left_cost
                else:
                    self.node_dictionary[left_node.i].pc = self.node_dictionary[left_node.i].pc
                    self.original_cost[left_node.i] = self.original_cost[left_node.i]
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
                top_left_cost=self.get_euclidean_distance(current_node.x,current_node.y,top_left_node.x,top_left_node.y)+ self.original_cost[current_node.i]
                top_left_h_cost=self.get_heuristic_distance(top_left_node.x,top_left_node.y,goal_node.x,goal_node.y)
                top_left_cost_total = top_left_cost + top_left_h_cost
                if self.node_dictionary[top_left_node.i].pc == None:
                    self.node_dictionary[top_left_node.i].pc = top_left_cost_total 
                    self.original_cost[top_left_node.i] = top_left_cost
                elif top_left_cost_total < self.node_dictionary[top_left_node.i].pc:
                    self.node_dictionary[top_left_node.i].pc = top_left_cost_total
                    self.original_cost[top_left_node.i] = top_left_cost
                else:
                    self.node_dictionary[top_left_node.i].pc = self.node_dictionary[top_left_node.i].pc
                    self.original_cost[top_left_node.i] = top_left_cost
        else:
            top_left_node=None
        
        return top_node,top_right_node,right_node,bottom_right_node,bottom_node,bottom_left_node,left_node,top_left_node
    
    def get_minimum_cost_node(self,current_node,goal_node):
        t,tr,r,br,b,bl,l,tl=self.get_adjacent_nodes(current_node,goal_node)
        adj_list = [t,tr,r,br,b,bl,l,tl]
        #Check for any nonetypes in list and deletes it
        if None in adj_list:
            while None in adj_list:
                adj_list.remove(None)  
        for index in range(0,len(adj_list)):
            self.unvisited_nodes[adj_list[index].i] = adj_list[index]
            
        node_to_visit = min(self.unvisited_nodes,key=lambda X: self.unvisited_nodes[X].pc)
        adj_list.clear()
        
        
        return node_to_visit
    
    def path_traceback(self,current_node):
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
        
    def get_minimum_cost_node_back(self,current_node):
        
        t,tr,r,br,b,bl,l,tl=self.path_traceback(current_node)
        adj_list = [t,tr,r,br,b,bl,l,tl]
        #Check for any nonetypes in list and deletes it
        if None in adj_list:
            while None in adj_list:
                adj_list.remove(None)  
        for index in range(0,len(adj_list)):
            self.traceback_nodes[adj_list[index].i] = adj_list[index]
            
        node_to_visit = min(self.traceback_nodes,key=lambda X: self.traceback_nodes[X].pc)
        adj_list.clear()
        
        
        return node_to_visit
    
    def execute(self):
        #Sets the each coords to the Node class
        self.node_dictionary = self.establish_nodes()
        #Gets starting node
        start_node=self.get_node(self.start_x,self.start_y)
        start_node.pc = 0
        current_node=start_node
        self.original_cost[current_node.i] = 0
        #Gets goal node
        goal_node=self.get_node(self.goal_x,self.goal_y)
        #first iteration
        self.unvisited_nodes[current_node.i] = current_node
        self.visited_nodes[current_node.i] = current_node
        del self.unvisited_nodes[current_node.i]
        while current_node.i != goal_node.i:
            #Get the next node with the least cost
            node_next=self.get_minimum_cost_node(current_node,goal_node)
            #Sets the current node as the adjacent node with the least cost
            current_node = self.unvisited_nodes[node_next]
            self.visited_nodes[current_node.i] = current_node
            del self.unvisited_nodes[current_node.i]
            if current_node.i == goal_node.i:
                print('goal reached!')
            else:
                print('goal not reached yet!')
                
        I = []
        J = []
        fig, backTrack =  plt.subplots()
        plt.xlim(self.x_limit[0]-self.step_size,self.x_limit[1]+self.step_size)
        plt.ylim(self.y_limit[0]-self.step_size,self.y_limit[1]+self.step_size)
        plt.title('AStar')
        plt.scatter(self.start_x,self.start_y,color='red')
        plt.scatter(self.goal_x,self.goal_y,color='green')
        plt.scatter(self.obstacles_x,self.obstacles_y)
        plt.xlabel('X-Position in Meters')
        plt.ylabel('Y-Position in Meters')
        plt.grid()
        
        current_node_back = goal_node 
        
        while current_node_back.i != start_node.i:
            I.append(current_node_back.x)
            J.append(current_node_back.y)
            node_to_trace=self.get_minimum_cost_node_back(current_node_back)
            current_node_back=self.traceback_nodes[node_to_trace]
            self.visited_traceback_nodes[current_node_back.i] = current_node_back
            del self.traceback_nodes[current_node_back.i]
            if current_node_back.i == start_node.i:
                print('traceback complete')
                print('Total Cost = ', self.original_cost[goal_node.i], ' meters')
            else:
                print('traceback in progress')
            
        I.append(self.start_x)
        J.append(self.start_y)
        plt.plot(I,J,color='black')
        
        
        
        
        
if __name__ == '__main__':
    #Obstacle List
    obstacles_x=[2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8, 8, 8, 8, 8, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 2, 2, 2, 2, 2, 2, 5, 5, 5, 5, 5, 5, 5, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12]
    obstacles_y=[2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2, 2, 2, 2, 3, 4, 5, 6, 7, 8, 9, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13, 14, 15, 12, 12, 12, 12, 12, 12, 8, 9, 10, 11, 12]
    #Plot Parameters
    x_limit = [0,15]
    y_limit = [0,15]
    step_size=0.5
    diameter = 1
    #Start and Goal
    start_x=1
    start_y=1
    goal_x=7
    goal_y=13
    
    a=AStarAlgorithm(obstacles_x, obstacles_y, x_limit, y_limit, step_size, diameter, start_x, start_y, goal_x, goal_y)
    start_time = time.time()
    a.execute()
    print('Runtime:', time.time()-start_time , 'seconds')
    
        

