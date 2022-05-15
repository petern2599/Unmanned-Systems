#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
from itertools import permutations
import matplotlib.pyplot as plt

#Node Class
class Node:
    def __init__(self,x_position,y_position,index):
        self.x = x_position
        self.y = y_position
        self.i = index
        
#Distance Class        
class Distance:
    def __init__(self,distance,node1,node2):
        self.distance = distance
        self.node_init = node1
        self.node_fin = node2
        
#Total Distance Class    
class Total_Distance:
    def __init__(self,total_distance,permutation):
        self.tot_dist = total_distance
        self.perm_combo = permutation
     
#TSP Class       
class TSP:
    def __init__(self,waypoints_x,waypoints_y):
        self.node_dictionary={}
        self.wp_x = waypoints_x
        self.wp_y = waypoints_y
        self.distance_dictionary={}
        self.total_distances={}
    
    #Inserts the node dictionary of all the 
    def establish_node(self):
        node_list=[]
        self.node_dictionary[0] = Node(self.wp_x[0],self.wp_y[0],0)
        i = 1
        while i < len(self.wp_x):
            self.node_dictionary[i] = Node(self.wp_x[i],self.wp_y[i],i)
            node_list.append(i)
            i += 1
        return node_list
    #Gets distance between nodes
    def get_euclidean_distance(self,node1_x,node1_y,node2_x,node2_y):
        distance_x = abs(node1_x-node2_x)
        distance_y = abs(node1_y-node2_y)
        euclidean_distance = math.sqrt(pow(distance_x,2)+pow(distance_y,2))
        return euclidean_distance
    
    #Establishes distances for each permutation
    def establish_distances(self):
        counter=0
        for i in range(0,len(self.wp_x)):
            for j in range(0,len(self.wp_x)):
                distance=self.get_euclidean_distance(self.wp_x[i], self.wp_y[i], self.wp_x[j], self.wp_y[j])
                distance_list=Distance(distance,i,j)
                self.distance_dictionary[counter] = distance_list
                counter += 1
        return self.distance_dictionary
    
    #Searches nodes and returns the distance between them
    def search_for_distance(self,node_1,node_2):
        for i in range(0,len(self.distance_dictionary)):
            if self.distance_dictionary[i].node_init == node_1 and self.distance_dictionary[i].node_fin==node_2:
                distance = self.distance_dictionary[i].distance
        return distance
    
    #Calculates the distance between each node in each permutation combination
    def calc_total_distances(self,perm):
        distance_values=[]
        counter=0
        for i in list(perm):
            #Gets distance from start to permutation element one
            distance_values.append(self.search_for_distance(0,i[0]))
            j=1
            while j < len(i)-1:
                #Gets distances between each permutation combination
                distance_values.append(self.search_for_distance(i[j], i[j+1]))
                j +=1
            #Gets distance from last permutation element to start
            distance_values.append(self.search_for_distance(i[j],0))
            #Stores total distance and combination in a dictionary
            self.total_distances[counter] = Total_Distance(sum(distance_values), i)
            counter += 1
            distance_values.clear()
        return self.total_distances
    
    #Gets node combination with minimum distance needed to travel
    def get_minimum_cost(self):
        nodes_to_visit = min(self.total_distances,key=lambda X: self.total_distances[X].tot_dist)
        list_of_nodes = self.total_distances[nodes_to_visit].perm_combo
        list_of_nodes = list(list_of_nodes)
        list_of_nodes.insert(0,0)
        list_of_nodes.append(0)
        return list_of_nodes,self.total_distances[nodes_to_visit].tot_dist
    
    #Main code
    def execute(self):
        node_list=self.establish_node()
        self.establish_distances()
        perm=permutations(node_list)
        self.calc_total_distances(perm)
                
        list_of_nodes,tot_dist=self.get_minimum_cost()
        
        print('Best combination of nodes to visit with the least distance is: ', list_of_nodes)
        print('The total distance is: ', tot_dist, ' meters')
        
        plot_x=[]
        plot_y=[]
        for i in range(0,len(list_of_nodes)):
            plot_x.append(self.node_dictionary[list_of_nodes[i]].x)
            plot_y.append(self.node_dictionary[list_of_nodes[i]].y)
            
        plt.figure(1)
        plt.plot(plot_x,plot_y)
        plt.title('TSP')
        plt.xlabel('X-Position in Meters')
        plt.ylabel('Y-Position in Meters')
        plt.text(0,0,'Start/Goal')
        plt.text(2,2,'WP 1')
        plt.text(6,4,'WP 2')
        plt.text(5,3,'WP 3')
        plt.text(3,4,'WP 4')
        plt.grid()
        plt.show()
        
    
if __name__ == '__main__':
    
    node_wp_x = [0,2,5,3,6]
    node_wp_y = [0,2,3,4,4]
    
    a=TSP(node_wp_x,node_wp_y)
    a.execute()
    