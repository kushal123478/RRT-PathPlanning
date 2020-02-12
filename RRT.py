# -*- coding: utf-8 -*-
"""
Created on Thu Nov 22 19:30:04 2018

@author: KUSHAL
"""
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt
from scipy.spatial import distance
from collections import namedtuple
import sys
import math



def extract_data(file_name):
    """This function extraxts the data from the text file and returns the start_vertex, end_vertex
    and the coordinates of the corners of each obstacles"""
    input_file_name = file_name
    
    def sort_data(input_file):  
        """This function extraxts the row data from the imput file"""                        
        with open(input_file,"r") as input_file:    
            data = input_file.readlines()          
            row_data = []                         
            for line in data:                           
                row_data.append(line.split(', ')) 
            return row_data  
                           
    data = sort_data(input_file_name)
    
    start_vertex = (float(data[0][0].split(' ')[0]),float(data[0][0].split(' ')[1]))
    end_vertex = (float(data[1][0].split(' ')[0]),float(data[1][0].split(' ')[1]))
    
    obstacles = {}
    for i in range(2, len(data)):
        lista = []
        for j in range(len(data[i])):
            lista.append( (float(data[i][j].split(' ')[0]),float(data[i][j].split(' ')[1])) )
        obstacles[i-1] = lista
    
    return start_vertex,end_vertex, obstacles

def extract_edges(obstacle): #obstacle is a list of tuples
    edges = {}
    for i in range(len(obstacle)):   
        edges[i] = (obstacle[i-1],obstacle[i])        
    return edges #edges is a dict of edges of each obstacle


def collision_detection(p, poly): 
    """Given a Poly object and a Pt object, this function determines wheter Pt
    collides with Poly or not"""
    _eps = 0.00001
    _huge = sys.float_info.max
    _tiny = sys.float_info.min
     
    def rayintersectseg(p, edge):
        ''' takes a point p=Pt() and an edge of two endpoints a,b=Pt() of a line segment returns boolean
        '''
        a,b = edge
        if a.y > b.y:
            a,b = b,a
        if p.y == a.y or p.y == b.y:
            p = Pt(p.x, p.y + _eps)
     
        intersect = False
     
        if (p.y > b.y or p.y < a.y) or (
            p.x > max(a.x, b.x)):
            return False
     
        if p.x < min(a.x, b.x):
            intersect = True
        else:
            if abs(a.x - b.x) > _tiny:
                m_red = (b.y - a.y) / float(b.x - a.x)
            else:
                m_red = _huge
            if abs(a.x - p.x) > _tiny:
                m_blue = (p.y - a.y) / float(p.x - a.x)
            else:
                m_blue = _huge
            intersect = m_blue >= m_red
        return intersect
     
    def _odd(x): return x%2 == 1
     
    def ispointinside(p, poly):

        return _odd(sum(rayintersectseg(p, edge)
                        for edge in poly.edges ))
    
    detection = ispointinside(p,poly)
    return detection

def overall_collision_detection(pt, polys):
    status = {}
    for poly in polys:
        status[poly] = collision_detection(pt, polys[poly])
    if True in status.values():
        return True
    else:
        return False
              
def random_sample(grid_size):
    """This function returns a random float tuple (x,y) where x and y are independent uniform random variables"""
    g = grid_size
    x_range = g[1] - g[0]

    y_range = g[3] - g[2]

    x_off = g[0]
    y_off = g[2]
    (x,y) = (x_range*np.random.ranf()+x_off,y_range*np.random.ranf()+y_off)    
    return (x,y)

class RRT:
    """class RRT used for all the main RRT operations"""
    def __init__(self,q0):
        self.nodes = {}
        self.nodes[0] = q0
        self.edges = {}
    
    def add_node(self,q):
        if q not in self.nodes:
            self.nodes[len(self.nodes)] = q
    def remove_last_node(self):
        del self.nodes[len(self.nodes)-1]
        
    def add_edge(self,q_exist,q_new):
        self.edges[q_new] = [q_exist, round(distance.euclidean(q_exist,q_new),4)]
    
    def new_random_node(self,grid_size):
        g = grid_size
        node = random_sample(g)
        node_pt = Pt(x = node[0], y = node[1])
 
        if overall_collision_detection(node_pt,polys) == False:
            self.add_node(node)
            return node
    
    def distance(self,point1,point2):
        distance = sqrt((point1[0]- point2[0])**2 + (point1[1] - point2[1])**2)
        return distance    
    
    def nearest_node(self,point):
        min_dist = self.distance((0,0),(point[0]+0.01,point[1]+0.01))
        nnear = 0
        for i in range(0,len(self.nodes.keys())):

            if self.nodes[i] != point:
                if self.distance(self.nodes[i],point) < min_dist:
                    min_dist=self.distance(self.nodes[i],point)
                    nnear = i
        return self.nodes[nnear]

    def edge_collision(self,point_on_tree,random_point):
        new_node = point_on_tree
        dmax = 0.2
        x1 = new_node[0]
        y1 = new_node[1]
        x2 = random_point[0]
        y2 = random_point[1]         
        theta = math.atan2(y2-y1,x2-x1)        
        while True:
            x1 = new_node[0]
            y1 = new_node[1]
            x2 = random_point[0]
            y2 = random_point[1]              
            d = self.distance(new_node,random_point)
            if d>dmax:
                (x1,y1)=(x1+dmax*math.cos(theta),y1+dmax*math.sin(theta))
                new_node = (x1,y1)
                if overall_collision_detection(Pt(new_node[0],new_node[1]),polys) == True:
                    return [new_node,True]
            else:
                return [random_point,False]
                break
    def searh_path(self,end_vertex,start_vertex):
        path = []
        bptr = end_vertex
        path.append(bptr)
        path_cost = 0
        while start_vertex not in path:
            bptr1 = self.edges[bptr][0]
            path_cost += self.edges[bptr][1]
            path.append(bptr1)
            bptr = bptr1
        print('The cost of the computed path is:')    
        print(path_cost)
        print('The path is:')
        path.reverse()
        print(path)
        return path

    


if __name__ == "__main__":
#----------------------------File input---------------------------------------#    
    input_file  = input("Name of the input file::: ")
    x0 = input("Grid size x low limit::: ")
    x1 = input("Grid size x high limit::: ")
    y0 = input("Grid size y low limit::: ")
    y1 = input("Grid size y high limit::: ") 

    [start_vertex, end_vertex, obstacles] = extract_data(input_file)
    grid_size = [float(x0), float(x1), float(y0), float(y1)]    
#--------Extracting the obstacle and start,end points from the file ----------#
    edges_all_obstacles = {}
    for obstacle in obstacles:
        edges_all_obstacles[obstacle] = extract_edges(obstacles[obstacle])
    
#---------------------------Collission checker--------------------------------#    
    Pt = namedtuple('Pt', 'x, y')               # Point
    Edge = namedtuple('Edge', 'a, b')           # Polygon edge from a to b
    Poly = namedtuple('Poly', 'name, edges')    # Polygon
    
#--------------------Creating polygons for obstacles--------------------------#
    
    polys = {}
    for obstacle in edges_all_obstacles:
        edges = []
        for edge in edges_all_obstacles[obstacle]:
            edge_a = edges_all_obstacles[obstacle][edge][0]
            edge_b = edges_all_obstacles[obstacle][edge][1]
            edge = Edge(a = Pt(edge_a[0],edge_a[1]), b = Pt(edge_b[0],edge_b[1]))
            edges.append(edge)
        edges = tuple(edges)
        polys[obstacle] = Poly("name", edges)

#------------------------PLotting Obstacles-----------------------------------#
    plt.figure(num=None, figsize=(12, 12), dpi=100, facecolor='w', edgecolor='k')
    plt.axis(grid_size)
    for obstacle in edges_all_obstacles:
        for edge in edges_all_obstacles[obstacle]:
            edge = edges_all_obstacles[obstacle][edge]
            plt.plot([edge[0][0],edge[1][0]],[edge[0][1],edge[1][1]],lw = 2,c ='k')
#----------------------------RRT Main Algo------------------------------------# 
    Tree = RRT((0,0))
    k = 0
    while(True):
        for i in range(200):    
            random_config = Tree.new_random_node(grid_size)
            if random_config is not None:            
                collision = Tree.edge_collision(Tree.nearest_node(random_config),random_config)
    
                if collision[1] == False:
                    Tree.add_edge(q_exist = Tree.nearest_node(random_config),q_new = collision[0])
                if collision[1] == True:
                    Tree.remove_last_node()
        random_config = end_vertex
        if random_config is not None:            
            collision = Tree.edge_collision(Tree.nearest_node(random_config),random_config)

            if collision[1] == False:
                Tree.add_edge(q_exist = Tree.nearest_node(random_config),q_new = collision[0])
            if collision[1] == True:
                Tree.remove_last_node()
        if end_vertex in Tree.edges:
            print('break')
            break

    print('over')
    path = Tree.searh_path(end_vertex,start_vertex)
#------------------Plotting the tree, path, start and end nodes---------------#    

    for edge in Tree.edges:
        plt.plot([edge[0],Tree.edges[edge][0][0]],[edge[1],Tree.edges[edge][0][1]],lw = 0.5,c = 'b')
    
    plt.plot(start_vertex[0],start_vertex[1],'go')
    plt.plot(end_vertex[0],end_vertex[1],'bo')
    for i in range(0,len(path)-1):
        plt.plot([ path[i][0] , path[i+1][0] ],[ path[i][1] , path[i+1][1] ],lw=1.5,c = 'r')
    plt.show()
    