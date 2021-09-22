# Standard Algorithm Implementation
# Sampling-based Algorithm Informed-RRT*

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from scipy import spatial
import math
from numpy import random
from PIL import Image
from random import randrange

# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.parent = None    # parent node
        self.cost = 0.0       # cost
        
# Class for Informed RRT*
class Informed_RRTstar:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.start = Node(start[0], start[1]) # start node
        self.goal = Node(goal[0], goal[1])    # goal node
        self.vertices = []                    # list of nodes
        self.found = False                    # found flag
        self.goalcost=0.0                     # cost from start to goal 
        

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    
    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###
        distance=math.sqrt((node1.row-node2.row)**2 + (node1.col-node2.col)**2)             # Calculating euclidean distance
        return distance
    
    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if there are obstacles
            False if the new node is valid to be connected
        '''
        # Check obstacle between nodes
        # get all the points in between
        points_between = zip(np.linspace(node1.row, node2.row, dtype=int), 
                             np.linspace(node1.col, node2.col, dtype=int))
        # check if any of these are obstacles
        for point in points_between:
            if self.map_array[point[0]][point[1]] == 0:
                return True
        return False



    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        ### YOUR CODE HERE ###
        a=randrange(100)                                # pick random variable for goal bias 
        # print(a)
        if a<=goal_bias:                                # if value within goal bias pick new point as goal
            y=True
        else:
            y=False
        if y==False:
            p1rows=random.randint(self.size_row)
            p1cols=random.randint(self.size_col)
            new_point=Node(p1rows,p1cols)
            return new_point
        
        else:
            return self.goal

    
    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###
        dist_new_node=[]
        minindex=0
        for i in range(len(self.vertices)):
            distance=self.dis(self.vertices[i],point)           # calculate distance of new point with respect to all nodes and append in dist_new_node
            dist_new_node.append(distance)
        
        minindex=dist_new_node.index(min(dist_new_node))        # find index value of minimum distance to node and return node 
        return self.vertices[minindex]  


    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance 
        '''
        ### YOUR CODE HERE ###
        neighbors=[]
        freeneighbors=[]
        for i in range(len(self.vertices)):     
            distance_rrt=self.dis(new_node,self.vertices[i])
            if distance_rrt<=neighbor_size:                     # find all neighbors within neighbour_size distance from new_node
                neighbors.append(self.vertices[i])

        for i in range(len(neighbors)):                         # find only the neighbouring nodes that do not have obstacles in between 
            collision_1=self.check_collision(neighbors[i],new_node)
            if collision_1==True:
                continue
            else:
                freeneighbors.append(neighbors[i])
        freeneighbors.remove(new_node)
        return freeneighbors                                # Return neighbours that are free from collision 


    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        ### YOUR CODE HERE ###
        if neighbors == []:                         # if no neighbors, no need to rewire
            return

        # Compute the distance from the new node to the neighbor nodes
        distances = [self.dis(new_node, node) for node in neighbors]

        # Rewire the new node
        # compute the least potential cost
        costs = [dist + self.path_cost(self.start, neighbors[i]) for i, dist in enumerate(distances)]
        indices = np.argsort(np.array(costs))
        # check collision and connect the best node to the new node
        for i in indices:
            if not self.check_collision(new_node, neighbors[i]):
                new_node.parent = neighbors[i]
                new_node.cost = distances[i]
                break

        # Rewire new_node's neighbors
        for i, node in enumerate(neighbors):
            # new cost
            new_cost = self.path_cost(self.start, new_node) + distances[i]
            # if new cost is lower
            # and there is no obstacles in between
            if self.path_cost(self.start, node) > new_cost and \
               not self.check_collision(node, new_node):
                node.parent = new_node
                node.cost = distances[i]


    def path_cost(self, start_node, end_node):           #Compute path cost starting from start node to end node
        cost = 0
        curr_node = end_node
        while start_node.row != curr_node.row or start_node.col != curr_node.col:
            # Keep tracing back until finding the start_node 
            # or no path exists
            parent = curr_node.parent
            if parent is None:
                print("Invalid Path")
                return 0
            cost += curr_node.cost
            curr_node = parent
        
        return cost

    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')
        
        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col and cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()

    def angle(self,p1,p2):                          # using angle components to find new node in direction of point from nearest node
        step=10 
        distance_1=self.dis(p1,p2)
        if distance_1 > step:
            distance_1= step
        
        theta= math.atan2(p2.col - p1.col, p2.row - p1.row)
        new_node1=Node((int((p1.row+distance_1*math.cos(theta)))),(int((p1.col+distance_1*math.sin(theta)))))

        return new_node1

    def sampleunitcircle(self):                     # Sample a point with the unit circle
        while True:
            x, y = random.uniform(-1, 1), random.uniform(-1, 1)
            if x ** 2 + y ** 2 < 1:
                a=np.array([[x], [y]])
            
            else:
                continue

            return a

    def ellipse(self,cmax,start,goal,center,cmin):
        while True:                                 # keep running till point which is in the map is returned
            angle=math.atan2(goal.col - start.col, goal.row - start.row)        
            c, s = np.cos(angle), np.sin(angle)     # Rotation matrix of the rotated ellipse calulated using theta above
            C1 = np.array(((c, -s), (s, c)))
            r1=cmax/2                               # major axis 
            r2= (math.sqrt(cmax**2-cmin**2))/2      # minor axis 
            L= np.diag([r1,r2])                     # diagonal matrix to convert ellipse from circle 
            xball=self.sampleunitcircle()

            xr=np.dot(np.dot(C1, L), xball)+center  # final transformation from cirle to rotated and translated ellipse
            xr1=xr[0][0]
            xr2=xr[1][0]
            xrand=Node(int(xr1),int(xr2))
            if xr1<self.size_row and xr2<self.size_col:
                return xrand
            else:
                continue

    def Informed_RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            neighbor_size - the neighbor distance
        
        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # In each step,
        # get a new point, 
        # get its nearest node, 
        # extend the node and check collision to decide whether to add or drop,
        # if added, rewire the node and its neighbors,
        # and check if reach the neighbor region of the goal if the path is not found.

        goal_bias = 4
        step=10
        cmin=self.dis(self.start,self.goal)
        xcenter=np.array([[(self.start.row+self.goal.row)/2],[(self.start.col+self.goal.col)/2]])
        nearnode=[]
        for i in range(n_pts):
            print(i)          
            if self.found==True:                            # if goal is found sample within the ellipse 
                new_p= self.ellipse(self.goalcost,self.start,self.goal,xcenter,cmin)
            else:
                new_p=self.get_new_point(goal_bias)         # else sample randomly on the map 
            if self.map_array[new_p.row][new_p.col]==0:
                continue
            else:
                nearestnode=self.get_nearest_node(new_p)
                newnode=self.angle(nearestnode,new_p)
                collision = self.check_collision(newnode,nearestnode)
                if collision==True:
                    continue
                else:
                    self.vertices.append(newnode)
                    neighbors=self.get_neighbors(newnode,20)
                    self.rewire(newnode,neighbors)
                distancetogoal=self.dis(newnode,self.goal)
                if distancetogoal<=step:
                    self.goal.parent=newnode
                    self.goal.cost=distancetogoal
                    self.vertices.append(self.goal)
                    self.found=True
                    self.goalcost=self.path_cost(self.start,self.goal)      
                    nearnode.append(newnode)

        self.rewire(self.goal,nearnode)                 #rewire again all the nodes close to the goal
         # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goalcost
            print("It took %d nodes to find the current path" %steps)
            print("The path length is %.2f" %length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()
